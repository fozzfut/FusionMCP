"""
FusionMCP Add-in

Runs inside Autodesk Fusion 360 as an add-in. Starts an HTTP server on
localhost:7432 that receives commands from the MCP server and dispatches
them to the Fusion 360 API on the main thread via custom events.
"""

import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import threading
import json
import queue
import uuid
import os
import math
import time
import collections
from http.server import HTTPServer, BaseHTTPRequestHandler
from http.server import ThreadingHTTPServer
from contextlib import contextmanager

app: adsk.core.Application = None
ui: adsk.core.UserInterface = None
_server: HTTPServer = None
_server_thread: threading.Thread = None
_handlers = []
_cmd_queue: queue.Queue = queue.Queue()
_results: dict = {}
_result_events: dict = {}
_lock = threading.Lock()

CUSTOM_EVENT_ID = "FusionMCPTickV4"
PORT = 7432

# Security note: exec() runs on localhost only. For optional authentication,
# set the FUSION_MCP_TOKEN environment variable.
_auth_token = None  # Set via environment variable FUSION_MCP_TOKEN for optional auth

_processing = False  # Re-entrance guard for notify()

# Event subscription state for Events/Callbacks commands
_event_subscriptions = {}
_event_queues = {}
_event_handlers_list = []
_max_event_queue = 200
_max_subscriptions = 10

SLOW_COMMANDS = {"export_stl", "export_step", "export_3mf", "export_f3d", "execute_script", "capture_screenshot",
                 "check_wall_thickness", "cam_generate_toolpath", "cam_post_process", "cam_simulate",
                 "export_bom", "generate_drawing", "batch_update_parameters",
                 "import_mesh", "convert_mesh_to_brep", "sheet_metal_flat_pattern",
                 "open_data_file", "open_data_version", "save_to_folder",
                 "generate_variant_batch", "pipeline_execute", "export_drawing"}


# ---- HTTP Handler ----

class MCPRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == "/ping":
            self._respond(200, {"status": "ok", "message": "Fusion MCP bridge running"})
        else:
            self._respond(404, {"error": "Not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length)
        try:
            data = json.loads(body)
        except Exception:
            self._respond(400, {"error": "Invalid JSON"})
            return
        self._respond(200, self._dispatch(data))

    def _dispatch(self, data: dict) -> dict:
        cmd_id = str(uuid.uuid4())
        data["id"] = cmd_id
        cmd = data.get("command", "")
        timeout = 120 if cmd in SLOW_COMMANDS else 30
        event = threading.Event()
        # Register the result event BEFORE enqueuing so the handler always
        # finds it.  fireCustomEvent is just a wake-up hint and is safe
        # outside the lock.
        with _lock:
            _result_events[cmd_id] = event
        _cmd_queue.put(data)
        if app:
            app.fireCustomEvent(CUSTOM_EVENT_ID, cmd_id)
        timed_out = not event.wait(timeout=timeout)
        with _lock:
            result = _results.pop(cmd_id, None)
            _result_events.pop(cmd_id, None)
        if result is None:
            return {"error": f"Timeout - Fusion did not respond in {timeout}s"}
        return result

    def _respond(self, code: int, body: dict):
        payload = json.dumps(body).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


# ---- Helpers ----

def _design():
    return adsk.fusion.Design.cast(app.activeProduct)

def _root():
    return _design().rootComponent

def _get_std_plane(root, name: str):
    n = name.upper()
    if n == "XZ": return root.xZConstructionPlane
    if n == "YZ": return root.yZConstructionPlane
    return root.xYConstructionPlane

def _resolve_plane(root, name: str):
    for i in range(root.constructionPlanes.count):
        cp = root.constructionPlanes.item(i)
        if cp.name.upper() == name.upper():
            return cp
    return _get_std_plane(root, name)

def _last_sketch(root):
    if root.sketches.count == 0:
        raise Exception("No sketch exists. Create one first.")
    return root.sketches.item(root.sketches.count - 1)

def _resolve_sketch(root, p):
    ref = p.get("sketch", "")
    if ref == "" or ref is None:
        return _last_sketch(root)
    return _find_sketch(root, ref)

def _find_body(root, ref):
    bodies = root.bRepBodies
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < bodies.count:
            return bodies.item(idx)
    for i in range(bodies.count):
        if bodies.item(i).name == ref:
            return bodies.item(i)
    if bodies.count > 0:
        return bodies.item(0)
    raise Exception("No bodies found in design.")

def _find_sketch(root, ref):
    sketches = root.sketches
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < sketches.count:
            return sketches.item(idx)
    for i in range(sketches.count):
        if sketches.item(i).name == ref:
            return sketches.item(i)
    raise Exception(f"Sketch '{ref}' not found.")

def _find_component(root, name):
    design = _design()
    if not name:
        return root
    for comp in design.allComponents:
        if comp.name == name:
            return comp
    raise Exception(f"Component '{name}' not found.")

def _find_occurrence(root, ref):
    """Find occurrence by name or index across all nesting levels."""
    all_occs = root.allOccurrences
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < all_occs.count:
            return all_occs.item(idx)
    for i in range(all_occs.count):
        occ = all_occs.item(i)
        if occ.name == ref or occ.component.name == ref:
            return occ
    raise Exception(f"Occurrence '{ref}' not found.")

def _walk_occurrences(occs):
    """Recursively build occurrence tree."""
    items = []
    for i in range(occs.count):
        occ = occs.item(i)
        children = []
        if occ.childOccurrences and occ.childOccurrences.count > 0:
            children = _walk_occurrences(occ.childOccurrences)
        items.append({
            "name": occ.name,
            "component": occ.component.name,
            "visible": occ.isLightBulbOn,
            "grounded": occ.isGrounded,
            "bodies": occ.component.bRepBodies.count,
            "children": children,
        })
    return items

def _op(name):
    ops = {
        "new_body": adsk.fusion.FeatureOperations.NewBodyFeatureOperation,
        "join":     adsk.fusion.FeatureOperations.JoinFeatureOperation,
        "cut":      adsk.fusion.FeatureOperations.CutFeatureOperation,
        "new_component": adsk.fusion.FeatureOperations.NewComponentFeatureOperation,
        "intersect": adsk.fusion.FeatureOperations.IntersectFeatureOperation,
    }
    return ops.get(name.lower(), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)

def _bool_op(name):
    ops = {
        "join":      adsk.fusion.BooleanTypes.JoinBooleanType,
        "cut":       adsk.fusion.BooleanTypes.CutBooleanType,
        "intersect": adsk.fusion.BooleanTypes.IntersectBooleanType,
    }
    return ops.get(name.lower(), adsk.fusion.BooleanTypes.JoinBooleanType)

def _axis_vector(name):
    n = name.upper()
    if n == "X": return adsk.core.Vector3D.create(1, 0, 0)
    if n == "Y": return adsk.core.Vector3D.create(0, 1, 0)
    return adsk.core.Vector3D.create(0, 0, 1)

def _construction_axis(root, name):
    n = name.upper()
    if n == "X": return root.xConstructionAxis
    if n == "Y": return root.yConstructionAxis
    return root.zConstructionAxis

def _collect_all_sketch_curves(sketch):
    curves = adsk.core.ObjectCollection.create()
    for i in range(sketch.sketchCurves.sketchLines.count):
        curves.add(sketch.sketchCurves.sketchLines.item(i))
    for i in range(sketch.sketchCurves.sketchCircles.count):
        curves.add(sketch.sketchCurves.sketchCircles.item(i))
    for i in range(sketch.sketchCurves.sketchArcs.count):
        curves.add(sketch.sketchCurves.sketchArcs.item(i))
    for i in range(sketch.sketchCurves.sketchFittedSplines.count):
        curves.add(sketch.sketchCurves.sketchFittedSplines.item(i))
    for i in range(sketch.sketchCurves.sketchEllipses.count):
        curves.add(sketch.sketchCurves.sketchEllipses.item(i))
    return curves


# ---- Transaction Management ----

SKIP_GROUPING = {"undo", "redo", "get_info", "get_bodies_info", "get_face_info", "get_edge_info",
                  "get_sketch_info", "get_timeline_info", "measure_body", "measure_between",
                  "list_parameters", "list_appearances", "list_documents", "list_occurrences_tree",
                  "get_component_info", "capture_screenshot", "fusion_status",
                  "check_interference", "get_section_properties", "get_curvature_info",
                  "check_wall_thickness", "check_draft_angles", "get_mesh_body_info",
                  "check_printability",
                  "cam_get_setups", "cam_list_tools", "cam_get_operation_info",
                  "get_sketch_health", "list_variants", "generate_bom",
                  "export_bom",
                  "get_hole_info",
                  "list_joints", "get_joint_info",
                  "list_data_hubs", "list_data_projects", "list_data_folders", "list_data_versions",
                  "poll_design_events", "get_subscription_status",
                  "compare_variants", "validate_parameters", "get_parameter_dependents"}

@contextmanager
def _timeline_group(label="MCP Operation"):
    """Group timeline entries created during the block into one undoable group."""
    design = _design()
    tl = design.timeline
    start_pos = tl.count
    try:
        yield
        end_pos = tl.count
        if end_pos - start_pos > 1:
            try:
                group = tl.timelineGroups.add(start_pos, end_pos - 1)
                group.name = label
            except Exception:
                pass  # grouping is best-effort
    except Exception:
        # Rollback: delete timeline entries created during this block
        try:
            max_rollback = tl.count - start_pos
            for _ in range(max_rollback):
                if tl.count <= start_pos:
                    break
                item = tl.item(tl.count - 1)
                item.entity.deleteMe()
        except Exception:
            pass
        raise


# ---- Info Commands ----

def _get_info(design, root):
    comps = [{"name": c.name, "id": c.id} for c in design.allComponents]
    sketches = [{"index": i, "name": root.sketches.item(i).name,
                 "profiles": root.sketches.item(i).profiles.count}
                for i in range(root.sketches.count)]
    bodies = [{"index": i, "name": root.bRepBodies.item(i).name,
               "faces": root.bRepBodies.item(i).faces.count}
              for i in range(root.bRepBodies.count)]
    params = [{"name": design.allParameters.item(i).name,
               "value": round(design.allParameters.item(i).value, 4),
               "unit": design.allParameters.item(i).unit}
              for i in range(design.allParameters.count)]
    planes = [{"index": i, "name": root.constructionPlanes.item(i).name}
              for i in range(root.constructionPlanes.count)]
    joints = []
    try:
        for i in range(root.joints.count):
            j = root.joints.item(i)
            joints.append({"index": i, "name": j.name, "type": str(j.jointMotion.jointType)})
    except Exception:
        pass
    return {
        "document": app.activeDocument.name,
        "components": comps, "sketches": sketches,
        "bodies": bodies, "parameters": params,
        "construction_planes": planes, "joints": joints,
    }

def _get_bodies_info(root):
    bodies = []
    for i in range(root.bRepBodies.count):
        b = root.bRepBodies.item(i)
        bb = b.boundingBox
        bodies.append({
            "index": i, "name": b.name, "visible": b.isVisible,
            "faces": b.faces.count, "edges": b.edges.count,
            "size_cm": {
                "x": round(bb.maxPoint.x - bb.minPoint.x, 4),
                "y": round(bb.maxPoint.y - bb.minPoint.y, 4),
                "z": round(bb.maxPoint.z - bb.minPoint.z, 4),
            },
        })
    return {"bodies": bodies}

def _get_face_info(root, p):
    body = _find_body(root, p.get("body", 0))
    pull_dir = p.get("pull_direction", None)
    pull_vec = None
    if pull_dir:
        if isinstance(pull_dir, str):
            pull_vec = _axis_vector(pull_dir)
        elif isinstance(pull_dir, list) and len(pull_dir) == 3:
            pull_vec = adsk.core.Vector3D.create(float(pull_dir[0]), float(pull_dir[1]), float(pull_dir[2]))
    faces = []
    for i in range(body.faces.count):
        f = body.faces.item(i)
        geo_type = type(f.geometry).__name__
        try:
            n = f.geometry.normal
            normal = [round(n.x, 3), round(n.y, 3), round(n.z, 3)]
        except Exception:
            normal = None
        face_info = {"index": i, "area_cm2": round(f.area, 4),
                     "type": geo_type, "normal": normal}
        if pull_vec and normal:
            try:
                dot = abs(normal[0] * pull_vec.x + normal[1] * pull_vec.y + normal[2] * pull_vec.z)
                angle = math.degrees(math.acos(min(dot, 1.0)))
                face_info["draft_angle_deg"] = round(90.0 - angle, 2)
            except Exception:
                pass
        faces.append(face_info)
    return {"body": body.name, "faces": faces}

def _get_edge_info(root, p):
    body = _find_body(root, p.get("body", 0))
    edges = []
    for i in range(body.edges.count):
        e = body.edges.item(i)
        geo_type = type(e.geometry).__name__
        edges.append({"index": i, "length_cm": round(e.length, 4), "type": geo_type})
    return {"body": body.name, "edges": edges}

def _get_sketch_info(root, p):
    sketch = _find_sketch(root, p.get("sketch", 0))
    lines = [{"index": i, "start": [round(l.startSketchPoint.geometry.x, 4),
                                      round(l.startSketchPoint.geometry.y, 4)],
              "end": [round(l.endSketchPoint.geometry.x, 4),
                       round(l.endSketchPoint.geometry.y, 4)],
              "length": round(l.length, 4),
              "isConstruction": l.isConstruction}
             for i, l in enumerate(sketch.sketchCurves.sketchLines)]
    circles = [{"index": i, "center": [round(c.centerSketchPoint.geometry.x, 4),
                                         round(c.centerSketchPoint.geometry.y, 4)],
                "radius": round(c.radius, 4)}
               for i, c in enumerate(sketch.sketchCurves.sketchCircles)]
    arcs = [{"index": i, "center": [round(a.centerSketchPoint.geometry.x, 4),
                                      round(a.centerSketchPoint.geometry.y, 4)],
             "radius": round(a.radius, 4)}
            for i, a in enumerate(sketch.sketchCurves.sketchArcs)]
    profiles = sketch.profiles.count
    constraints = []
    for i in range(sketch.geometricConstraints.count):
        c = sketch.geometricConstraints.item(i)
        constraints.append({"index": i, "type": type(c).__name__})
    dimensions = []
    for i in range(sketch.sketchDimensions.count):
        d = sketch.sketchDimensions.item(i)
        dimensions.append({"index": i, "type": type(d).__name__,
                            "value": round(d.value, 4) if hasattr(d, 'value') else None})
    return {"sketch": sketch.name, "profiles": profiles,
            "lines": lines, "circles": circles, "arcs": arcs,
            "constraints": constraints, "dimensions": dimensions}

def _get_timeline_info():
    design = _design()
    timeline = design.timeline
    items = []
    for i in range(timeline.count):
        item = timeline.item(i)
        items.append({
            "index": i, "name": item.entity.name if hasattr(item.entity, 'name') else str(i),
            "type": type(item.entity).__name__,
            "isSuppressed": item.isSuppressed,
            "isRolledBack": item.isRolledBack,
        })
    return {"timeline_count": timeline.count, "items": items}

def _measure_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bb = body.boundingBox
    vol = None
    try:
        props = body.physicalProperties
        vol = round(props.volume, 6)
    except Exception:
        pass
    result_dict = {
        "body": body.name,
        "size_cm": {
            "x": round(bb.maxPoint.x - bb.minPoint.x, 4),
            "y": round(bb.maxPoint.y - bb.minPoint.y, 4),
            "z": round(bb.maxPoint.z - bb.minPoint.z, 4),
        },
        "volume_cm3": vol,
        "faces": body.faces.count,
        "edges": body.edges.count,
        "vertices": body.vertices.count,
    }
    if p.get("include_physical", False):
        try:
            phys = body.physicalProperties
            result_dict["mass_kg"] = round(phys.mass, 6)
            result_dict["density_kg_m3"] = round(phys.density, 4)
            result_dict["area_cm2"] = round(phys.area, 6)
            com = phys.centerOfMass
            result_dict["center_of_mass"] = [round(com.x, 4), round(com.y, 4), round(com.z, 4)]
            result_dict["material"] = body.material.name if body.material else None
        except Exception:
            pass
    return result_dict

def _measure_between(root, p):
    e1 = p.get("entity1", "")
    e2 = p.get("entity2", "")

    def parse_entity(spec):
        parts = spec.split(":")
        if len(parts) >= 2 and parts[0] == "body":
            body = _find_body(root, parts[1])
            if len(parts) >= 4 and parts[2] == "face":
                return body.faces.item(int(parts[3]))
            if len(parts) >= 4 and parts[2] == "edge":
                return body.edges.item(int(parts[3]))
            return body
        raise Exception(f"Cannot parse entity: {spec}")

    ent1 = parse_entity(e1)
    ent2 = parse_entity(e2)
    mgr = app.measureManager
    result = mgr.measureMinimumDistance(ent1, ent2)
    return {
        "distance_cm": round(result.value, 6),
        "point1": [round(result.pointOne.x, 4), round(result.pointOne.y, 4), round(result.pointOne.z, 4)],
        "point2": [round(result.pointTwo.x, 4), round(result.pointTwo.y, 4), round(result.pointTwo.z, 4)],
    }


# ---- Execute Script ----

def _execute_script(p):
    if _auth_token and p.get("token") != _auth_token:
        return {"error": "Authentication required. Provide 'token' parameter."}
    code = p.get("code", "")
    if not code.strip():
        return {"error": "No code provided"}
    design = _design()
    root = design.rootComponent
    result = {"output": None}
    local_vars = {
        "app": app, "ui": ui, "design": design, "root": root,
        "adsk": __import__("adsk"),
        "result": result,
        "math": math, "json": json,
    }
    try:
        exec(code, {"__builtins__": __builtins__}, local_vars)
        output = local_vars.get("result", result).get("output", None)
        return {"success": True, "output": output}
    except Exception as e:
        traceback.print_exc()
        return {"error": str(e)}


# ---- Document Management ----

def _create_new_document(p):
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    name = p.get("name", "Untitled")
    return {"created": True, "document": name}

def _clear_design():
    design = _design()
    root = design.rootComponent
    timeline = design.timeline
    for i in range(timeline.count - 1, -1, -1):
        try:
            timeline.item(i).entity.deleteMe()
        except Exception:
            pass
    while root.sketches.count > 0:
        try:
            root.sketches.item(0).deleteMe()
        except Exception:
            break
    while root.constructionPlanes.count > 0:
        try:
            root.constructionPlanes.item(0).deleteMe()
        except Exception:
            break
    return {"cleared": True}


# ---- Sketch Commands ----

def _create_sketch(root, p):
    comp = _find_component(root, p.get("component", ""))
    plane = _resolve_plane(comp, p.get("plane", "XY"))
    sketch = comp.sketches.add(plane)
    name = p.get("name", "")
    if name:
        sketch.name = name
    return {"sketch": sketch.name, "plane": p.get("plane", "XY")}

def _create_sketch_on_face(root, p):
    body = _find_body(root, p.get("body", 0))
    face_index = int(p.get("face_index", 0))
    if face_index >= body.faces.count:
        return {"error": f"Face {face_index} out of range ({body.faces.count} faces)"}
    face = body.faces.item(face_index)
    sketch = root.sketches.add(face)
    name = p.get("name", "")
    if name:
        sketch.name = name
    return {"sketch": sketch.name, "on_body": body.name, "face_index": face_index}

def _finish_sketch(root, p):
    sketch = _resolve_sketch(root, p)
    name = sketch.name
    sketch.isComputeDeferred = False
    return {"finished": name, "profiles": sketch.profiles.count}

def _delete_sketch(root, p):
    sketch = _find_sketch(root, p.get("sketch", 0))
    name = sketch.name
    sketch.deleteMe()
    return {"deleted_sketch": name}

def _draw_rectangle(root, p):
    sketch = _resolve_sketch(root, p)
    x1, y1 = float(p.get("x1", 0)), float(p.get("y1", 0))
    x2, y2 = float(p.get("x2", 10)), float(p.get("y2", 10))
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(x1, y1, 0),
        adsk.core.Point3D.create(x2, y2, 0))
    return {"sketch": sketch.name, "rectangle": f"({x1},{y1}) to ({x2},{y2})",
            "profiles": sketch.profiles.count}

def _draw_center_rectangle(root, p):
    sketch = _resolve_sketch(root, p)
    cx, cy = float(p.get("cx", 0)), float(p.get("cy", 0))
    w, h = float(p.get("width", 10)), float(p.get("height", 10))
    x1, y1 = cx - w / 2, cy - h / 2
    x2, y2 = cx + w / 2, cy + h / 2
    sketch.sketchCurves.sketchLines.addTwoPointRectangle(
        adsk.core.Point3D.create(x1, y1, 0),
        adsk.core.Point3D.create(x2, y2, 0))
    return {"sketch": sketch.name, "center_rectangle": f"center=({cx},{cy}), {w}x{h}",
            "profiles": sketch.profiles.count}

def _draw_circle(root, p):
    sketch = _resolve_sketch(root, p)
    cx, cy, r = float(p.get("cx", 0)), float(p.get("cy", 0)), float(p.get("radius", 5))
    sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(cx, cy, 0), r)
    return {"sketch": sketch.name, "circle": f"center=({cx},{cy}), r={r}",
            "profiles": sketch.profiles.count}

def _draw_line(root, p):
    sketch = _resolve_sketch(root, p)
    x1, y1 = float(p.get("x1", 0)), float(p.get("y1", 0))
    x2, y2 = float(p.get("x2", 10)), float(p.get("y2", 10))
    sketch.sketchCurves.sketchLines.addByTwoPoints(
        adsk.core.Point3D.create(x1, y1, 0),
        adsk.core.Point3D.create(x2, y2, 0))
    return {"sketch": sketch.name, "line": f"({x1},{y1}) to ({x2},{y2})"}

def _draw_arc(root, p):
    sketch = _resolve_sketch(root, p)
    cx, cy = float(p.get("cx", 0)), float(p.get("cy", 0))
    r = float(p.get("radius", 5))
    start_angle = float(p.get("start_angle", 0))
    sweep_angle = float(p.get("sweep_angle", 90))
    center = adsk.core.Point3D.create(cx, cy, 0)
    start_pt = adsk.core.Point3D.create(
        cx + r * math.cos(math.radians(start_angle)),
        cy + r * math.sin(math.radians(start_angle)), 0)
    sketch.sketchCurves.sketchArcs.addByCenterStartSweep(
        center, start_pt, math.radians(sweep_angle))
    return {"sketch": sketch.name, "arc": f"center=({cx},{cy}), r={r}, sweep={sweep_angle}deg"}

def _draw_polygon(root, p):
    sketch = _resolve_sketch(root, p)
    cx, cy = float(p.get("cx", 0)), float(p.get("cy", 0))
    r = float(p.get("radius", 5))
    sides = int(p.get("sides", 6))
    points = [adsk.core.Point3D.create(
        cx + r * math.cos(math.radians(360.0 / sides * i)),
        cy + r * math.sin(math.radians(360.0 / sides * i)), 0)
        for i in range(sides)]
    lines = sketch.sketchCurves.sketchLines
    for i in range(sides):
        lines.addByTwoPoints(points[i], points[(i + 1) % sides])
    return {"sketch": sketch.name, "polygon": f"{sides}-sided, center=({cx},{cy}), r={r}",
            "profiles": sketch.profiles.count}

def _draw_ellipse(root, p):
    sketch = _resolve_sketch(root, p)
    cx, cy = float(p.get("cx", 0)), float(p.get("cy", 0))
    rx, ry = float(p.get("rx", 5)), float(p.get("ry", 3))
    center = adsk.core.Point3D.create(cx, cy, 0)
    major = adsk.core.Point3D.create(cx + rx, cy, 0)
    minor_pt = adsk.core.Point3D.create(cx, cy + ry, 0)
    sketch.sketchCurves.sketchEllipses.add(center, major, minor_pt)
    return {"sketch": sketch.name, "ellipse": f"center=({cx},{cy}), rx={rx}, ry={ry}",
            "profiles": sketch.profiles.count}

def _draw_spline(root, p):
    sketch = _resolve_sketch(root, p)
    raw_pts = p.get("points", [[0, 0], [5, 5], [10, 0]])
    pts = adsk.core.ObjectCollection.create()
    for pt in raw_pts:
        pts.add(adsk.core.Point3D.create(float(pt[0]), float(pt[1]), 0))
    sketch.sketchCurves.sketchFittedSplines.add(pts)
    return {"sketch": sketch.name, "spline": f"{len(raw_pts)} control points"}

def _draw_slot(root, p):
    sketch = _resolve_sketch(root, p)
    x1, y1 = float(p.get("x1", 0)), float(p.get("y1", 0))
    x2, y2 = float(p.get("x2", 10)), float(p.get("y2", 0))
    width = float(p.get("width", 3))
    dx = x2 - x1
    dy = y2 - y1
    length = math.sqrt(dx * dx + dy * dy)
    if length == 0:
        return {"error": "Slot endpoints cannot be the same point"}
    r = width / 2.0
    nx = -dy / length
    ny = dx / length
    p1 = adsk.core.Point3D.create(x1 + nx * r, y1 + ny * r, 0)
    p2 = adsk.core.Point3D.create(x2 + nx * r, y2 + ny * r, 0)
    p3 = adsk.core.Point3D.create(x2 - nx * r, y2 - ny * r, 0)
    p4 = adsk.core.Point3D.create(x1 - nx * r, y1 - ny * r, 0)
    c1 = adsk.core.Point3D.create(x1, y1, 0)
    c2 = adsk.core.Point3D.create(x2, y2, 0)
    lines = sketch.sketchCurves.sketchLines
    arcs = sketch.sketchCurves.sketchArcs
    lines.addByTwoPoints(p1, p2)
    arcs.addByCenterStartSweep(c2, p2, math.pi)
    lines.addByTwoPoints(p3, p4)
    arcs.addByCenterStartSweep(c1, p4, math.pi)
    return {"sketch": sketch.name, "slot": f"({x1},{y1})->({x2},{y2}), width={width}",
            "profiles": sketch.profiles.count}

def _draw_text(root, p):
    sketch = _resolve_sketch(root, p)
    text = p.get("text", "Hello")
    x, y = float(p.get("x", 0)), float(p.get("y", 0))
    height = float(p.get("height", 1.0))
    texts = sketch.sketchTexts
    input_obj = texts.createInput2(text, height)
    input_obj.setAsMultiLine(
        adsk.core.Point3D.create(x, y, 0),
        adsk.core.Point3D.create(x + height * len(text) * 0.6, y + height, 0),
        adsk.core.HorizontalAlignments.LeftHorizontalAlignment,
        adsk.core.VerticalAlignments.BottomVerticalAlignment, 0)
    texts.add(input_obj)
    return {"sketch": sketch.name, "text": text}

def _add_sketch_fillet(root, p):
    sketch = _resolve_sketch(root, p)
    radius = float(p.get("radius", 1.0))
    line1_idx = int(p.get("line1_index", 0))
    line2_idx = int(p.get("line2_index", 1))
    lines = sketch.sketchCurves.sketchLines
    if lines.count < 2:
        return {"error": "Need at least 2 lines in sketch"}
    l1 = lines.item(line1_idx)
    l2 = lines.item(line2_idx)
    sketch.sketchCurves.sketchArcs.addFillet(
        l1, l1.endSketchPoint.geometry,
        l2, l2.startSketchPoint.geometry,
        radius)
    return {"sketch_fillet": f"radius={radius} between lines {line1_idx} and {line2_idx}"}

def _offset_sketch(root, p):
    sketch = _resolve_sketch(root, p)
    curves = _collect_all_sketch_curves(sketch)
    if curves.count == 0:
        return {"error": "No curves in sketch to offset"}
    direction = adsk.core.Point3D.create(
        float(p.get("dx", 1)), float(p.get("dy", 1)), 0)
    sketch.offset(curves, direction, float(p.get("distance", 1)))
    return {"offset": f"distance={p.get('distance', 1)} cm"}

def _mirror_sketch(root, p):
    sketch = _resolve_sketch(root, p)
    axis_idx = int(p.get("axis_line_index", 0))
    lines = sketch.sketchCurves.sketchLines
    if lines.count == 0:
        return {"error": "No lines in sketch (need at least one as mirror axis)"}
    axis_line = lines.item(axis_idx)
    curves = adsk.core.ObjectCollection.create()
    for i in range(lines.count):
        if i != axis_idx:
            curves.add(lines.item(i))
    for i in range(sketch.sketchCurves.sketchCircles.count):
        curves.add(sketch.sketchCurves.sketchCircles.item(i))
    for i in range(sketch.sketchCurves.sketchArcs.count):
        curves.add(sketch.sketchCurves.sketchArcs.item(i))
    for i in range(sketch.sketchCurves.sketchFittedSplines.count):
        curves.add(sketch.sketchCurves.sketchFittedSplines.item(i))
    if curves.count == 0:
        return {"error": "No curves to mirror (only the axis line exists)"}
    sketch.mirror(curves, axis_line)
    return {"mirrored": f"{curves.count} curves"}

def _rectangular_pattern_sketch(root, p):
    sketch = _resolve_sketch(root, p)
    curves = _collect_all_sketch_curves(sketch)
    if curves.count == 0:
        return {"error": "No curves in sketch to pattern"}
    x_count = int(p.get("x_count", 2))
    y_count = int(p.get("y_count", 2))
    x_spacing = float(p.get("x_spacing", 5))
    y_spacing = float(p.get("y_spacing", 5))
    for ix in range(x_count):
        for iy in range(y_count):
            if ix == 0 and iy == 0:
                continue
            transform = adsk.core.Matrix3D.create()
            transform.translation = adsk.core.Vector3D.create(
                ix * x_spacing, iy * y_spacing, 0)
            sketch.copy(curves, transform)
    return {"pattern": f"{x_count}x{y_count}, spacing=({x_spacing},{y_spacing})"}


# ---- Sketch Constraints ----

def _add_constraint(root, p):
    sketch = _resolve_sketch(root, p)
    ctype = p.get("constraint_type", "coincident").lower()
    e1_idx = int(p.get("entity1_index", 0))
    e2_idx = int(p.get("entity2_index", -1))
    e1_type = p.get("entity1_type", "point").lower()
    e2_type = p.get("entity2_type", "point").lower()

    def get_entity(sketch, etype, idx):
        if etype == "point":
            return sketch.sketchPoints.item(idx)
        elif etype == "line":
            return sketch.sketchCurves.sketchLines.item(idx)
        elif etype == "circle":
            return sketch.sketchCurves.sketchCircles.item(idx)
        elif etype == "arc":
            return sketch.sketchCurves.sketchArcs.item(idx)
        elif etype == "curve":
            total = 0
            for coll in [sketch.sketchCurves.sketchLines,
                         sketch.sketchCurves.sketchArcs,
                         sketch.sketchCurves.sketchCircles]:
                if idx < total + coll.count:
                    return coll.item(idx - total)
                total += coll.count
            raise Exception(f"Curve index {idx} out of range")
        raise Exception(f"Unknown entity type: {etype}")

    gc = sketch.geometricConstraints
    ent1 = get_entity(sketch, e1_type, e1_idx)

    if ctype == "horizontal":
        gc.addHorizontal(ent1)
        return {"constraint": "horizontal"}
    elif ctype == "vertical":
        gc.addVertical(ent1)
        return {"constraint": "vertical"}
    elif ctype == "fix":
        gc.addFix(ent1)
        return {"constraint": "fix"}

    if e2_idx < 0:
        return {"error": f"Constraint '{ctype}' requires two entities (entity2_index missing)"}
    ent2 = get_entity(sketch, e2_type, e2_idx)

    if ctype == "coincident":
        gc.addCoincident(ent1, ent2)
    elif ctype == "tangent":
        gc.addTangent(ent1, ent2)
    elif ctype == "perpendicular":
        gc.addPerpendicular(ent1, ent2)
    elif ctype == "parallel":
        gc.addParallel(ent1, ent2)
    elif ctype == "concentric":
        gc.addConcentric(ent1, ent2)
    elif ctype == "equal":
        gc.addEqual(ent1, ent2)
    elif ctype == "collinear":
        gc.addCollinear(ent1, ent2)
    elif ctype == "midpoint":
        gc.addMidPoint(ent1, ent2)
    elif ctype == "smooth":
        gc.addSmooth(ent1, ent2)
    else:
        return {"error": f"Unknown constraint type: {ctype}"}
    return {"constraint": ctype}

def _add_sketch_dimension(root, p):
    sketch = _resolve_sketch(root, p)
    dtype = p.get("dimension_type", "distance").lower()
    e1_idx = int(p.get("entity1_index", 0))
    e2_idx = int(p.get("entity2_index", -1))
    e1_type = p.get("entity1_type", "line").lower()
    e2_type = p.get("entity2_type", "").lower()
    value = float(p.get("value", 5.0))

    def get_entity(sketch, etype, idx):
        if etype == "line":
            return sketch.sketchCurves.sketchLines.item(idx)
        elif etype == "circle":
            return sketch.sketchCurves.sketchCircles.item(idx)
        elif etype == "arc":
            return sketch.sketchCurves.sketchArcs.item(idx)
        elif etype == "point":
            return sketch.sketchPoints.item(idx)
        raise Exception(f"Unknown entity type: {etype}")

    dims = sketch.sketchDimensions
    text_pt = adsk.core.Point3D.create(5, 5, 0)

    if dtype == "distance":
        ent1 = get_entity(sketch, e1_type, e1_idx)
        if e2_idx >= 0 and e2_type:
            ent2 = get_entity(sketch, e2_type, e2_idx)
            dim = dims.addDistanceDimension(ent1, ent2, 0, text_pt)
        else:
            line = ent1
            dim = dims.addDistanceDimension(
                line.startSketchPoint, line.endSketchPoint, 0, text_pt)
        dim.parameter.value = value
        return {"dimension": "distance", "value_cm": value}
    elif dtype == "angle":
        ent1 = get_entity(sketch, e1_type, e1_idx)
        ent2 = get_entity(sketch, e2_type if e2_type else "line", e2_idx)
        dim = dims.addAngularDimension(ent1, ent2, text_pt)
        dim.parameter.value = math.radians(value)
        return {"dimension": "angle", "value_deg": value}
    elif dtype == "diameter":
        ent1 = get_entity(sketch, e1_type, e1_idx)
        dim = dims.addDiameterDimension(ent1, text_pt)
        dim.parameter.value = value
        return {"dimension": "diameter", "value_cm": value}
    elif dtype == "radius":
        ent1 = get_entity(sketch, e1_type, e1_idx)
        dim = dims.addRadialDimension(ent1, text_pt)
        dim.parameter.value = value
        return {"dimension": "radius", "value_cm": value}
    return {"error": f"Unknown dimension type: {dtype}"}


# ---- Feature Commands ----

def _extrude(root, p):
    sketch = _resolve_sketch(root, p)
    if sketch.profiles.count == 0:
        return {"error": "No closed profile in sketch."}
    pidx = min(int(p.get("profile_index", 0)), sketch.profiles.count - 1)
    profile = sketch.profiles.item(pidx)
    distance = float(p.get("distance", 1))
    ext_input = root.features.extrudeFeatures.createInput(profile, _op(p.get("operation", "new_body")))
    ext_input.setDistanceExtent(bool(p.get("symmetric", False)),
                                adsk.core.ValueInput.createByReal(distance))
    root.features.extrudeFeatures.add(ext_input)
    return {"extruded": sketch.name, "distance_cm": distance, "bodies": root.bRepBodies.count}

def _revolve(root, p):
    sketch = _resolve_sketch(root, p)
    if sketch.profiles.count == 0:
        return {"error": "No closed profile in sketch."}
    profile = sketch.profiles.item(int(p.get("profile_index", 0)))
    lines = sketch.sketchCurves.sketchLines
    if lines.count == 0:
        return {"error": "Draw a line in the sketch to use as the revolve axis."}
    axis_line = lines.item(int(p.get("axis_index", 0)))
    angle = float(p.get("angle", 360))
    rev_input = root.features.revolveFeatures.createInput(
        profile, axis_line, _op(p.get("operation", "new_body")))
    rev_input.setAngleExtent(False, adsk.core.ValueInput.createByReal(math.radians(angle)))
    root.features.revolveFeatures.add(rev_input)
    return {"revolved": sketch.name, "angle_deg": angle}

def _loft(root, p):
    sketch_indices = p.get("sketch_indices", [])
    if len(sketch_indices) < 2:
        return {"error": "Loft needs at least 2 sketch indices"}
    loft_input = root.features.loftFeatures.createInput(
        _op(p.get("operation", "new_body")))
    added = 0
    for idx in sketch_indices:
        sk = root.sketches.item(int(idx))
        if sk.profiles.count == 0:
            return {"error": f"Sketch at index {idx} has no closed profile."}
        loft_input.loftSections.add(sk.profiles.item(0))
        added += 1
    root.features.loftFeatures.add(loft_input)
    return {"lofted": f"{added} profiles", "sketch_indices": sketch_indices}

def _sweep(root, p):
    if root.sketches.count < 2:
        return {"error": "Need at least 2 sketches: profile and path."}
    profile_sketch = root.sketches.item(int(p.get("profile_sketch_index", 0)))
    path_sketch = root.sketches.item(int(p.get("path_sketch_index", 1)))
    if profile_sketch.profiles.count == 0:
        return {"error": "Profile sketch has no closed profile."}
    profile = profile_sketch.profiles.item(0)
    first_curve = None
    for coll in [path_sketch.sketchCurves.sketchLines,
                 path_sketch.sketchCurves.sketchArcs,
                 path_sketch.sketchCurves.sketchFittedSplines]:
        if coll.count > 0:
            first_curve = coll.item(0)
            break
    if not first_curve:
        return {"error": "Path sketch has no curves."}
    sweep_path = root.features.createPath(first_curve, True)
    sweep_input = root.features.sweepFeatures.createInput(
        profile, sweep_path, _op(p.get("operation", "new_body")))
    root.features.sweepFeatures.add(sweep_input)
    return {"swept": "profile along path"}

def _helix(root, p):
    sketch = _last_sketch(root)
    if sketch.profiles.count == 0:
        return {"error": "Draw a profile in the active sketch first."}
    profile = sketch.profiles.item(0)
    pitch = float(p.get("pitch", 1.0))
    height = float(p.get("height", 5.0))
    coil_input = root.features.coilFeatures.createInput(
        profile, root.zConstructionAxis,
        adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    coil_input.height = adsk.core.ValueInput.createByReal(height)
    coil_input.pitch = adsk.core.ValueInput.createByReal(pitch)
    coil_input.isClockwise = bool(p.get("clockwise", True))
    root.features.coilFeatures.add(coil_input)
    return {"helix": f"pitch={pitch}cm, height={height}cm"}

def _create_pipe(root, p):
    path_idx = int(p.get("path_sketch_index", 0))
    path_sketch = root.sketches.item(path_idx)
    section_size = float(p.get("section_size", 0.5))
    wall_thickness = float(p.get("wall_thickness", 0))
    first_curve = None
    for coll in [path_sketch.sketchCurves.sketchLines,
                 path_sketch.sketchCurves.sketchArcs,
                 path_sketch.sketchCurves.sketchFittedSplines]:
        if coll.count > 0:
            first_curve = coll.item(0)
            break
    if not first_curve:
        return {"error": "Path sketch has no curves."}
    sweep_path = root.features.createPath(first_curve, True)
    start_pt = first_curve.startSketchPoint.geometry if hasattr(first_curve, 'startSketchPoint') else adsk.core.Point3D.create(0, 0, 0)
    temp_plane = root.xYConstructionPlane
    temp_sketch = root.sketches.add(temp_plane)
    temp_sketch.sketchCurves.sketchCircles.addByCenterRadius(
        adsk.core.Point3D.create(start_pt.x, start_pt.y, 0), section_size)
    profile = temp_sketch.profiles.item(0)
    sweep_input = root.features.sweepFeatures.createInput(
        profile, sweep_path, _op(p.get("operation", "new_body")))
    feat = root.features.sweepFeatures.add(sweep_input)
    if wall_thickness > 0 and feat.bodies.count > 0:
        pipe_body = feat.bodies.item(0)
        faces = adsk.core.ObjectCollection.create()
        end_faces = []
        for i in range(pipe_body.faces.count):
            f = pipe_body.faces.item(i)
            try:
                if f.geometry.normal:
                    end_faces.append((f.area, f))
            except Exception:
                pass
        if len(end_faces) >= 2:
            end_faces.sort(key=lambda x: x[0])
            faces.add(end_faces[0][1])
            faces.add(end_faces[1][1])
            shell_input = root.features.shellFeatures.createInput(faces, False)
            shell_input.insideThickness = adsk.core.ValueInput.createByReal(wall_thickness)
            root.features.shellFeatures.add(shell_input)
    return {"pipe": f"section_radius={section_size}cm, wall={wall_thickness}cm"}

def _create_hole(root, p):
    body = _find_body(root, p.get("body", 0))
    face_index = int(p.get("face_index", 0))
    face = body.faces.item(face_index)
    diameter = float(p.get("diameter", 1.0))
    depth = float(p.get("depth", 2.0))
    hole_type = p.get("hole_type", "simple").lower()
    x = float(p.get("x", 0))
    y = float(p.get("y", 0))
    point = adsk.core.Point3D.create(x, y, 0)
    holes = root.features.holeFeatures
    hole_input = holes.createSimpleInput(adsk.core.ValueInput.createByReal(diameter))
    hole_input.setPositionByPoint(face, point)
    hole_input.setDistanceExtent(adsk.core.ValueInput.createByReal(depth))
    if hole_type == "counterbore":
        cb_dia = float(p.get("counterbore_diameter", diameter * 1.8))
        cb_depth = float(p.get("counterbore_depth", diameter * 0.5))
        hole_input = holes.createCounterboreInput(
            adsk.core.ValueInput.createByReal(diameter),
            adsk.core.ValueInput.createByReal(cb_dia),
            adsk.core.ValueInput.createByReal(cb_depth))
        hole_input.setPositionByPoint(face, point)
        hole_input.setDistanceExtent(adsk.core.ValueInput.createByReal(depth))
    elif hole_type == "countersink":
        cs_dia = float(p.get("countersink_diameter", diameter * 2.0))
        cs_angle = float(p.get("countersink_angle", 90))
        hole_input = holes.createCountersinkInput(
            adsk.core.ValueInput.createByReal(diameter),
            adsk.core.ValueInput.createByReal(cs_dia),
            adsk.core.ValueInput.createByReal(math.radians(cs_angle)))
        hole_input.setPositionByPoint(face, point)
        hole_input.setDistanceExtent(adsk.core.ValueInput.createByReal(depth))
    holes.add(hole_input)
    return {"hole": hole_type, "diameter_cm": diameter, "depth_cm": depth}

def _shell(root, p):
    body = _find_body(root, p.get("body", 0))
    thickness = float(p.get("thickness", 0.2))
    face_indices = p.get("face_indices", [0])
    faces = adsk.core.ObjectCollection.create()
    for fi in face_indices:
        faces.add(body.faces.item(int(fi)))
    shell_input = root.features.shellFeatures.createInput(faces, False)
    shell_input.insideThickness = adsk.core.ValueInput.createByReal(thickness)
    root.features.shellFeatures.add(shell_input)
    return {"shelled": body.name, "thickness_cm": thickness, "open_faces": face_indices}

def _fillet(root, p):
    body = _find_body(root, p.get("body", 0))
    radius = float(p.get("radius", 0.5))
    edge_indices = p.get("edge_indices", [0])
    edges = adsk.core.ObjectCollection.create()
    for ei in edge_indices:
        edges.add(body.edges.item(int(ei)))
    fi = root.features.filletFeatures.createInput()
    fi.addConstantRadiusEdgeSet(edges, adsk.core.ValueInput.createByReal(radius), True)
    root.features.filletFeatures.add(fi)
    return {"filleted": body.name, "radius_cm": radius, "edges": edge_indices}

def _chamfer(root, p):
    body = _find_body(root, p.get("body", 0))
    distance = float(p.get("distance", 0.3))
    edge_indices = p.get("edge_indices", [0])
    edges = adsk.core.ObjectCollection.create()
    for ei in edge_indices:
        edges.add(body.edges.item(int(ei)))
    ci = root.features.chamferFeatures.createInput(edges, True)
    ci.setToEqualDistance(adsk.core.ValueInput.createByReal(distance))
    root.features.chamferFeatures.add(ci)
    return {"chamfered": body.name, "distance_cm": distance, "edges": edge_indices}

def _mirror_body(root, p):
    body = _find_body(root, p.get("body", 0))
    plane_name = p.get("plane", "XY")
    mirror_plane = _resolve_plane(root, plane_name)
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    mi = root.features.mirrorFeatures.createInput(bodies_col, mirror_plane)
    root.features.mirrorFeatures.add(mi)
    return {"mirrored": body.name, "plane": plane_name}

def _rectangular_pattern_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    x_count = int(p.get("x_count", 2))
    y_count = int(p.get("y_count", 1))
    x_spacing = float(p.get("x_spacing", 5))
    y_spacing = float(p.get("y_spacing", 5))
    pi = root.features.rectangularPatternFeatures.createInput(
        bodies_col, root.xConstructionAxis,
        adsk.core.ValueInput.createByReal(x_count),
        adsk.core.ValueInput.createByReal(x_spacing),
        adsk.fusion.PatternDistanceType.SpacingPatternDistanceType)
    pi.setDirectionTwo(root.yConstructionAxis,
                       adsk.core.ValueInput.createByReal(y_count),
                       adsk.core.ValueInput.createByReal(y_spacing))
    root.features.rectangularPatternFeatures.add(pi)
    return {"pattern": f"{x_count}x{y_count}", "body": body.name}

def _circular_pattern_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    count = int(p.get("count", 4))
    axis = _construction_axis(root, p.get("axis", "Z"))
    pi = root.features.circularPatternFeatures.createInput(bodies_col, axis)
    pi.quantity = adsk.core.ValueInput.createByReal(count)
    pi.totalAngle = adsk.core.ValueInput.createByString("360 deg")
    pi.isSymmetric = True
    root.features.circularPatternFeatures.add(pi)
    return {"circular_pattern": f"{count} instances", "body": body.name}

def _combine_bodies(root, p):
    target = _find_body(root, p.get("target_body", 0))
    tools = adsk.core.ObjectCollection.create()
    for ti in p.get("tool_bodies", [1]):
        tools.add(_find_body(root, ti))
    ci = root.features.combineFeatures.createInput(target, tools)
    ci.operation = _bool_op(p.get("operation", "join"))
    ci.isKeepToolBodies = bool(p.get("keep_tools", False))
    root.features.combineFeatures.add(ci)
    return {"combined": target.name, "operation": p.get("operation", "join")}

def _scale_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    scale_x = float(p.get("scale_x", p.get("scale", 2.0)))
    scale_y = float(p.get("scale_y", -1))
    scale_z = float(p.get("scale_z", -1))
    point = adsk.core.Point3D.create(
        float(p.get("cx", 0)), float(p.get("cy", 0)), float(p.get("cz", 0)))
    if scale_y == -1 and scale_z == -1:
        si = root.features.scaleFeatures.createInput(
            bodies_col, point, adsk.core.ValueInput.createByReal(scale_x))
    else:
        if scale_y == -1: scale_y = scale_x
        if scale_z == -1: scale_z = scale_x
        si = root.features.scaleFeatures.createInput(
            bodies_col, point, adsk.core.ValueInput.createByReal(scale_x))
        si.setToNonUniform(
            adsk.core.ValueInput.createByReal(scale_x),
            adsk.core.ValueInput.createByReal(scale_y),
            adsk.core.ValueInput.createByReal(scale_z))
    root.features.scaleFeatures.add(si)
    return {"scaled": body.name, "scale": [scale_x, scale_y, scale_z]}

def _move_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    dx = float(p.get("dx", 0))
    dy = float(p.get("dy", 0))
    dz = float(p.get("dz", 0))
    transform = adsk.core.Matrix3D.create()
    transform.translation = adsk.core.Vector3D.create(dx, dy, dz)
    mi = root.features.moveFeatures.createInput2(bodies_col)
    mi.defineAsFreeMove(transform)
    root.features.moveFeatures.add(mi)
    return {"moved": body.name, "translation_cm": {"dx": dx, "dy": dy, "dz": dz}}

def _rotate_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    angle = float(p.get("angle", 45))
    axis_vec = _axis_vector(p.get("axis", "Z"))
    cx = float(p.get("cx", 0))
    cy = float(p.get("cy", 0))
    cz = float(p.get("cz", 0))
    origin = adsk.core.Point3D.create(cx, cy, cz)
    transform = adsk.core.Matrix3D.create()
    transform.setToRotation(math.radians(angle), axis_vec, origin)
    mi = root.features.moveFeatures.createInput2(bodies_col)
    mi.defineAsFreeMove(transform)
    root.features.moveFeatures.add(mi)
    return {"rotated": body.name, "axis": p.get("axis", "Z"),
            "angle_deg": angle, "center": [cx, cy, cz]}

def _press_pull(root, p):
    body = _find_body(root, p.get("body", 0))
    face_index = int(p.get("face_index", 0))
    face = body.faces.item(face_index)
    faces = adsk.core.ObjectCollection.create()
    faces.add(face)
    distance = float(p.get("distance", 1))
    pp = root.features.offsetFacesFeatures.createInput(
        faces, adsk.core.ValueInput.createByReal(distance))
    root.features.offsetFacesFeatures.add(pp)
    return {"press_pull": body.name, "face": face_index, "distance_cm": distance}

def _thicken(root, p):
    sketch = _last_sketch(root)
    if sketch.profiles.count == 0:
        return {"error": "No profile in sketch"}
    profile = sketch.profiles.item(0)
    thickness = float(p.get("thickness", 0.5))
    ext_input = root.features.extrudeFeatures.createInput(
        profile, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ext_input.setDistanceExtent(False, adsk.core.ValueInput.createByReal(thickness))
    root.features.extrudeFeatures.add(ext_input)
    return {"thickened": sketch.name, "thickness_cm": thickness}

def _draft_face(root, p):
    body = _find_body(root, p.get("body", 0))
    angle = float(p.get("angle", 3))
    face_index = int(p.get("face_index", 0))
    faces = adsk.core.ObjectCollection.create()
    faces.add(body.faces.item(face_index))
    plane_map = {"XY": root.xYConstructionPlane, "XZ": root.xZConstructionPlane}
    pull_dir = plane_map.get(p.get("pull_plane", "XY").upper(), root.xYConstructionPlane)
    di = root.features.draftFeatures.createInput(
        faces, pull_dir, adsk.core.ValueInput.createByReal(math.radians(angle)), False)
    root.features.draftFeatures.add(di)
    return {"draft": body.name, "angle_deg": angle, "face": face_index}

def _add_thread(root, p):
    body = _find_body(root, p.get("body", 0))
    face_index = int(p.get("face_index", 0))
    face = body.faces.item(face_index)
    is_internal = bool(p.get("is_internal", False))
    threadFeats = root.features.threadFeatures
    threadQuery = threadFeats.threadDataQuery
    threadData = threadQuery.recommendThreadData(face, is_internal)
    if not threadData:
        return {"error": f"Cannot add thread to face {face_index}. Must be cylindrical with standard size."}
    thread_type = p.get("thread_type", "")
    if thread_type:
        threadData.threadType = thread_type
    threadData.isRightHanded = bool(p.get("right_handed", True))
    ti = threadFeats.createInput(face, threadData)
    ti.isFullLength = bool(p.get("full_length", True))
    threadFeats.add(ti)
    return {"thread": f"type={threadData.threadType}, designation={threadData.designation}",
            "body": body.name, "face": face_index, "internal": is_internal}


# ---- Assembly / Joints ----

def _create_component(root, p):
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    comp = occ.component
    comp.name = p.get("name", "Component")
    return {"created": comp.name, "id": comp.id}

def _move_body_to_component(root, p):
    body = _find_body(root, p.get("body", 0))
    comp = _find_component(root, p.get("component", ""))
    body_name = body.name
    body.moveToComponent(comp.occurrences.item(0) if comp != root else root.occurrences.addNewComponent(adsk.core.Matrix3D.create()))
    return {"moved_body": body_name, "to_component": comp.name}

def _create_joint(root, p):
    design = _design()
    comp1_name = p.get("component1", "")
    comp2_name = p.get("component2", "")
    joint_type = p.get("joint_type", "rigid").lower()

    occ1 = occ2 = None
    for i in range(root.occurrences.count):
        occ = root.occurrences.item(i)
        if occ.component.name == comp1_name:
            occ1 = occ
        if occ.component.name == comp2_name:
            occ2 = occ
    if not occ1 or not occ2:
        return {"error": f"Components not found. Available: {[root.occurrences.item(i).component.name for i in range(root.occurrences.count)]}"}

    joint_types = {
        "rigid": adsk.fusion.JointTypes.RigidJointType,
        "revolute": adsk.fusion.JointTypes.RevoluteJointType,
        "slider": adsk.fusion.JointTypes.SliderJointType,
        "cylindrical": adsk.fusion.JointTypes.CylindricalJointType,
        "pin_slot": adsk.fusion.JointTypes.PinSlotJointType,
        "planar": adsk.fusion.JointTypes.PlanarJointType,
        "ball": adsk.fusion.JointTypes.BallJointType,
    }

    ox = float(p.get("offset_x", 0))
    oy = float(p.get("offset_y", 0))
    oz = float(p.get("offset_z", 0))

    geo0 = adsk.fusion.JointGeometry.createByPoint(occ1)
    geo1 = adsk.fusion.JointGeometry.createByPoint(occ2)

    ji = root.joints.createInput(geo0, geo1)
    ji.setAsRigidJointMotion() if joint_type == "rigid" else None
    if joint_type == "revolute":
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "slider":
        ji.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "cylindrical":
        ji.setAsCylindricalJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "ball":
        ji.setAsBallJointMotion()
    elif joint_type == "planar":
        ji.setAsPlanarJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)

    joint = root.joints.add(ji)
    if ox != 0 or oy != 0 or oz != 0:
        joint.jointMotion.slideValue = oz if hasattr(joint.jointMotion, 'slideValue') else None
    return {"joint": joint.name, "type": joint_type,
            "between": [comp1_name, comp2_name]}

def _create_as_built_joint(root, p):
    comp1_name = p.get("component1", "")
    comp2_name = p.get("component2", "")
    joint_type = p.get("joint_type", "rigid").lower()

    occ1 = occ2 = None
    for i in range(root.occurrences.count):
        occ = root.occurrences.item(i)
        if occ.component.name == comp1_name:
            occ1 = occ
        if occ.component.name == comp2_name:
            occ2 = occ
    if not occ1 or not occ2:
        return {"error": "Components not found."}

    ji = root.asBuiltJoints.createInput(occ1, occ2, None)
    if joint_type == "rigid":
        ji.setAsRigidJointMotion()
    elif joint_type == "revolute":
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "slider":
        ji.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "ball":
        ji.setAsBallJointMotion()
    joint = root.asBuiltJoints.add(ji)
    return {"as_built_joint": joint.name, "type": joint_type}


# ---- Body Management ----

def _delete_body(root, p):
    body = _find_body(root, p.get("body", 0))
    name = body.name
    body.deleteMe()
    return {"deleted_body": name}

def _rename_body(root, p):
    body = _find_body(root, p.get("body", 0))
    old = body.name
    body.name = p.get("name", "Body")
    return {"renamed": old, "to": body.name}

def _copy_body(root, p):
    body = _find_body(root, p.get("body", 0))
    bodies_col = adsk.core.ObjectCollection.create()
    bodies_col.add(body)
    transform = adsk.core.Matrix3D.create()
    dx = float(p.get("dx", 2))
    dy = float(p.get("dy", 0))
    dz = float(p.get("dz", 0))
    transform.translation = adsk.core.Vector3D.create(dx, dy, dz)
    mi = root.features.moveFeatures.createInput2(bodies_col)
    mi.defineAsFreeMove(transform)
    mi.isCopy = True
    feature = root.features.moveFeatures.add(mi)
    new_body = feature.bodies.item(0) if feature.bodies.count > 0 else None
    new_name = p.get("name", body.name + "_copy")
    if new_body:
        new_body.name = new_name
    return {"copied": body.name, "new_body": new_name,
            "offset_cm": {"dx": dx, "dy": dy, "dz": dz}}

def _toggle_body_visibility(root, p):
    body = _find_body(root, p.get("body", 0))
    body.isVisible = not body.isVisible
    return {"body": body.name, "visible": body.isVisible}


# ---- Construction Geometry ----

def _add_construction_plane(root, p):
    plane_type = p.get("type", "offset").lower()
    planes = root.constructionPlanes
    if plane_type == "offset":
        base = _resolve_plane(root, p.get("base_plane", "XY"))
        offset = float(p.get("offset", 2))
        pi = planes.createInput()
        pi.setByOffset(base, adsk.core.ValueInput.createByReal(offset))
        plane = planes.add(pi)
        return {"construction_plane": plane.name, "offset_cm": offset}
    elif plane_type == "angle":
        base = _resolve_plane(root, p.get("base_plane", "XY"))
        edge_body = _find_body(root, p.get("body", 0))
        edge = edge_body.edges.item(int(p.get("edge_index", 0)))
        angle = float(p.get("angle", 45))
        pi = planes.createInput()
        pi.setByAngle(edge, adsk.core.ValueInput.createByReal(math.radians(angle)), base)
        plane = planes.add(pi)
        return {"construction_plane": plane.name, "angle_deg": angle}
    elif plane_type == "midplane":
        body = _find_body(root, p.get("body", 0))
        f1 = body.faces.item(int(p.get("face1_index", 0)))
        f2 = body.faces.item(int(p.get("face2_index", 1)))
        pi = planes.createInput()
        pi.setByTwoPlanes(f1, f2)
        plane = planes.add(pi)
        return {"construction_plane": plane.name, "type": "midplane"}
    return {"error": f"Unsupported plane type '{plane_type}'. Use: offset, angle, midplane"}

def _add_construction_axis(root, p):
    axis_type = p.get("axis_type", "edge").lower()
    axes = root.constructionAxes
    if axis_type == "cylinder":
        body = _find_body(root, p.get("body", 0))
        face = body.faces.item(int(p.get("face_index", 0)))
        ai = axes.createInput()
        ai.setByCircularFace(face)
        axis = axes.add(ai)
        return {"construction_axis": axis.name, "type": "cylinder"}
    elif axis_type == "edge":
        body = _find_body(root, p.get("body", 0))
        edge = body.edges.item(int(p.get("edge_index", 0)))
        ai = axes.createInput()
        ai.setByLine(edge)
        axis = axes.add(ai)
        return {"construction_axis": axis.name, "type": "edge"}
    elif axis_type == "perpendicular":
        body = _find_body(root, p.get("body", 0))
        face = body.faces.item(int(p.get("face_index", 0)))
        ai = axes.createInput()
        ai.setByNormalToFaceAtPoint(face, face.pointOnFace)
        axis = axes.add(ai)
        return {"construction_axis": axis.name, "type": "perpendicular"}
    return {"error": f"Unknown axis type: {axis_type}. Use: cylinder, edge, perpendicular"}


# ---- Parameter Tools ----

def _add_parameter(design, p):
    name = p.get("name", "param1")
    value = float(p.get("value", 10))
    unit = p.get("unit", "cm")
    comment = p.get("comment", "")
    design.userParameters.add(
        name, adsk.core.ValueInput.createByString(f"{value} {unit}"), unit, comment)
    return {"parameter": name, "value": value, "unit": unit}

def _update_parameter(design, p):
    name = p.get("name")
    value = float(p.get("value", 10))
    param = design.allParameters.itemByName(name)
    if not param:
        return {"error": f"Parameter '{name}' not found"}
    param.value = value
    return {"updated": name, "new_value": value}

def _list_parameters(design):
    return {"parameters": [
        {"name": design.allParameters.item(i).name,
         "value": round(design.allParameters.item(i).value, 4),
         "unit": design.allParameters.item(i).unit,
         "expression": design.allParameters.item(i).expression}
        for i in range(design.allParameters.count)]}


# ---- Appearance & Materials ----

def _list_appearances(p):
    results = []
    try:
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                for j in range(min(lib.appearances.count, 30)):
                    a = lib.appearances.item(j)
                    results.append({"name": a.name, "library": lib.name})
            except Exception:
                pass
    except Exception as e:
        return {"error": str(e)}
    search = p.get("search", "").lower()
    if search:
        results = [r for r in results if search in r["name"].lower()]
    return {"appearances": results[:50]}

def _apply_appearance(root, p):
    body = _find_body(root, p.get("body", 0))
    search = p.get("appearance", "Steel").lower()
    try:
        for i in range(app.materialLibraries.count):
            lib = app.materialLibraries.item(i)
            try:
                for j in range(lib.appearances.count):
                    a = lib.appearances.item(j)
                    if search in a.name.lower():
                        body.appearance = a
                        return {"applied": a.name, "to": body.name}
            except Exception:
                pass
        return {"error": f"No appearance matching '{search}' found."}
    except Exception as e:
        return {"error": str(e)}

def _set_body_color(root, p):
    body = _find_body(root, p.get("body", 0))
    r = int(p.get("r", 128))
    g = int(p.get("g", 128))
    b_val = int(p.get("b", 255))
    opacity = int(p.get("opacity", 255))
    try:
        design = _design()
        base_lib = app.materialLibraries.item(0)
        base_appearance = base_lib.appearances.item(0)
        custom = design.appearances.addByCopy(base_appearance, f"Color_{body.name}")
        props = custom.appearanceProperties
        for i in range(props.count):
            prop = props.item(i)
            if hasattr(prop, 'value'):
                try:
                    if isinstance(prop.value, adsk.core.Color):
                        prop.value = adsk.core.Color.create(r, g, b_val, opacity)
                except Exception:
                    pass
        body.appearance = custom
        return {"color_set": body.name, "rgb": f"({r},{g},{b_val})", "opacity": opacity}
    except Exception as e:
        return {"error": f"Could not set color: {e}"}


# ---- Export & Capture ----

def _export_stl(p):
    design = _design()
    path = p.get("path", os.path.expanduser("~/Desktop/fusion_export.stl"))
    opts = design.exportManager.createSTLExportOptions(design.rootComponent, path)
    opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
    design.exportManager.execute(opts)
    return {"exported_stl": path}

def _export_step(p):
    design = _design()
    path = p.get("path", os.path.expanduser("~/Desktop/fusion_export.step"))
    opts = design.exportManager.createSTEPExportOptions(path, design.rootComponent)
    design.exportManager.execute(opts)
    return {"exported_step": path}

def _export_3mf(p):
    design = _design()
    path = p.get("path", os.path.expanduser("~/Desktop/fusion_export.3mf"))
    opts = design.exportManager.createC3MFExportOptions(design.rootComponent, path)
    design.exportManager.execute(opts)
    return {"exported_3mf": path}

def _export_f3d(p):
    design = _design()
    path = p.get("path", os.path.expanduser("~/Desktop/fusion_export.f3d"))
    opts = design.exportManager.createFusionArchiveExportOptions(path)
    design.exportManager.execute(opts)
    return {"exported_f3d": path}

def _capture_screenshot(p):
    path = p.get("path", os.path.expanduser("~/Desktop/fusion_screenshot.png"))
    width = int(p.get("width", 1920))
    height = int(p.get("height", 1080))
    app.activeViewport.saveAsImageFile(path, width, height)
    return {"screenshot": path, "size": f"{width}x{height}"}


# ---- History ----

def _undo(p):
    design = _design()
    tl = design.timeline
    count = int(p.get("steps", 1))
    actual = min(count, tl.markerPosition)
    if actual > 0:
        tl.markerPosition -= actual
    return {"undone": actual, "marker_position": tl.markerPosition, "timeline_count": tl.count}

def _redo(p):
    design = _design()
    tl = design.timeline
    count = int(p.get("steps", 1))
    available = tl.count - tl.markerPosition
    actual = min(count, available)
    if actual > 0:
        tl.markerPosition += actual
    return {"redone": actual, "marker_position": tl.markerPosition, "timeline_count": tl.count}

def _save_design(p):
    doc = app.activeDocument
    try:
        doc.save(p.get("description", "Saved"))
        return {"saved": doc.name}
    except Exception as e:
        if "saveAs" in str(e) or "save" in str(e).lower():
            return {"error": "Document has never been saved. Use save_as first."}
        return {"error": str(e)}

def _save_as(p):
    doc = app.activeDocument
    name = p.get("name", "My Design")
    try:
        data_mgr = app.data
        active_hub = data_mgr.activeHub
        root_folder = active_hub.dataProjects.item(0).rootFolder
        doc.saveAs(name, root_folder, p.get("description", "Saved"), "")
        return {"saved_as": name}
    except Exception as e:
        return {"error": str(e)}


# ---- Component/Occurrence Management ----

def _rename_component(root, p):
    name = p.get("component", "")
    new_name = p.get("name", "")
    if not new_name:
        raise Exception("Parameter 'name' is required.")
    comp = _find_component(root, name)
    old = comp.name
    comp.name = new_name
    return {"renamed": old, "new_name": new_name}

def _delete_occurrence(root, p):
    ref = p.get("occurrence", "")
    occ = _find_occurrence(root, ref)
    name = occ.name
    occ.deleteMe()
    return {"deleted": name}

def _toggle_component_visibility(root, p):
    ref = p.get("occurrence", "")
    occ = _find_occurrence(root, ref)
    occ.isLightBulbOn = not occ.isLightBulbOn
    return {"occurrence": occ.name, "visible": occ.isLightBulbOn}

def _list_occurrences_tree(root):
    return {"tree": _walk_occurrences(root.occurrences)}

def _activate_component(root, p):
    ref = p.get("component", "root")
    design = _design()
    if ref == "root" or ref == "":
        design.activateRootComponent()
        return {"activated": "root", "component": root.name}
    # Find occurrence to activate
    occ = _find_occurrence(root, ref)
    occ.activate()
    return {"activated": occ.name, "component": occ.component.name}

def _get_component_info(root, p):
    ref = p.get("component", "")
    comp = _find_component(root, ref)
    bodies = [{"name": comp.bRepBodies.item(i).name, "visible": comp.bRepBodies.item(i).isVisible}
              for i in range(comp.bRepBodies.count)]
    sketches = [{"name": comp.sketches.item(i).name, "profiles": comp.sketches.item(i).profiles.count}
                for i in range(comp.sketches.count)]
    children = [{"name": comp.occurrences.item(i).name, "component": comp.occurrences.item(i).component.name}
                for i in range(comp.occurrences.count)]
    return {"component": comp.name, "bodies": bodies, "sketches": sketches, "children": children}

def _list_documents():
    docs = []
    active_name = app.activeDocument.name
    for i in range(app.documents.count):
        d = app.documents.item(i)
        docs.append({"name": d.name, "is_active": d.name == active_name})
    return {"documents": docs, "count": len(docs)}

def _switch_document(p):
    name = p.get("name", "")
    for i in range(app.documents.count):
        d = app.documents.item(i)
        if d.name == name or name in d.name:
            d.activate()
            return {"activated": d.name}
    raise Exception(f"Document '{name}' not found.")


# ---- Analysis & DFM ----

def _check_interference(root, p):
    body1 = _find_body(root, p.get("body1", 0))
    body2 = _find_body(root, p.get("body2", 1))
    # Use TemporaryBRepManager for non-destructive check
    temp_mgr = adsk.fusion.TemporaryBRepManager.get()
    temp1 = temp_mgr.copy(body1)
    temp2 = temp_mgr.copy(body2)
    try:
        temp_mgr.booleanOperation(temp1, temp2, adsk.fusion.BooleanTypes.IntersectBooleanType)
        has_interference = temp1.volume > 1e-10
        return {"interference": has_interference, "overlap_volume_cm3": round(temp1.volume, 8) if has_interference else 0}
    except Exception:
        return {"interference": False, "overlap_volume_cm3": 0, "note": "Bodies do not overlap"}

def _assign_material(root, p):
    body = _find_body(root, p.get("body", 0))
    lib_name = p.get("library", "Fusion 360 Material Library")
    mat_name = p.get("material", "")
    if not mat_name:
        raise Exception("Parameter 'material' is required")
    # Get material library
    mat_libs = app.materialLibraries
    for i in range(mat_libs.count):
        lib = mat_libs.item(i)
        if lib_name.lower() in lib.name.lower():
            for j in range(lib.materials.count):
                mat = lib.materials.item(j)
                if mat_name.lower() in mat.name.lower():
                    body.material = mat
                    return {"body": body.name, "material": mat.name, "library": lib.name}
    raise Exception(f"Material '{mat_name}' not found in library '{lib_name}'")

def _get_section_properties(root, p):
    body = _find_body(root, p.get("body", 0))
    plane = _resolve_plane(root, p.get("plane", "XY"))
    # Create section analysis using SectionAnalysis
    try:
        plane_geo = plane.geometry if hasattr(plane, 'geometry') else adsk.core.Plane.create(
            adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create(0, 0, 1))
        profiles = body.findSectionProfiles(plane_geo)
        if profiles and profiles.count > 0:
            total_area = sum(pr.area for pr in profiles)
            return {"body": body.name, "section_profiles": profiles.count, "total_area_cm2": round(total_area, 6)}
        return {"body": body.name, "section_profiles": 0, "total_area_cm2": 0}
    except Exception as e:
        return {"body": body.name, "error": str(e), "note": "Section analysis requires specific plane geometry"}

def _get_curvature_info(root, p):
    body = _find_body(root, p.get("body", 0))
    face_idx = int(p.get("face", 0))
    if face_idx < 0 or face_idx >= body.faces.count:
        raise Exception(f"Face index {face_idx} out of range (0-{body.faces.count - 1})")
    face = body.faces.item(face_idx)
    evaluator = face.evaluator
    # Get curvature at face center (parametric midpoint)
    try:
        (ok, min_pt, max_pt) = evaluator.getParameterExtents()
        mid_u = (min_pt.x + max_pt.x) / 2
        mid_v = (min_pt.y + max_pt.y) / 2
        param = adsk.core.Point2D.create(mid_u, mid_v)
        (ok2, max_tang, min_tang, max_curv, min_curv) = evaluator.getCurvature(param)
        return {
            "body": body.name, "face": face_idx,
            "max_curvature": round(max_curv, 6),
            "min_curvature": round(min_curv, 6),
            "gaussian_curvature": round(max_curv * min_curv, 8),
            "face_type": type(face.geometry).__name__
        }
    except Exception as e:
        return {"body": body.name, "face": face_idx, "error": str(e)}

def _check_wall_thickness(root, p):
    body = _find_body(root, p.get("body", 0))
    min_thickness = float(p.get("min_thickness", 0.1))  # cm
    mgr = app.measureManager
    thin_walls = []
    checked = 0
    faces = body.faces
    for i in range(faces.count):
        for j in range(i + 1, min(faces.count, i + 20)):  # limit pairwise checks
            try:
                result_m = mgr.measureMinimumDistance(faces.item(i), faces.item(j))
                dist = result_m.value
                if 0 < dist < min_thickness:
                    thin_walls.append({"face1": i, "face2": j, "thickness_cm": round(dist, 6)})
                checked += 1
            except Exception:
                pass
    return {
        "body": body.name, "min_threshold_cm": min_thickness,
        "thin_walls_found": len(thin_walls), "pairs_checked": checked,
        "thin_walls": thin_walls[:20]  # limit output
    }

def _check_draft_angles(root, p):
    body = _find_body(root, p.get("body", 0))
    pull_str = p.get("pull_direction", "Z").upper()
    min_angle = float(p.get("min_angle", 1.0))  # degrees
    pull = _axis_vector(pull_str)

    results = []
    fail_count = 0
    for i in range(body.faces.count):
        face = body.faces.item(i)
        try:
            normal = face.geometry.normal if hasattr(face.geometry, 'normal') else None
            if normal:
                dot = abs(normal.x * pull.x + normal.y * pull.y + normal.z * pull.z)
                angle = math.degrees(math.acos(min(dot, 1.0)))
                draft = 90.0 - angle
                passed = abs(draft) >= min_angle or abs(draft) < 0.01  # perpendicular or has draft
                if not passed:
                    fail_count += 1
                results.append({"face": i, "draft_angle_deg": round(draft, 2), "passed": passed})
        except Exception:
            pass

    return {
        "body": body.name, "pull_direction": pull_str,
        "min_draft_angle": min_angle, "total_faces": body.faces.count,
        "failed_faces": fail_count, "all_passed": fail_count == 0,
        "faces": results[:50]  # limit output
    }

def _get_mesh_body_info(root, p):
    ref = p.get("body", 0)
    # Check mesh bodies
    meshes = root.meshBodies
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < meshes.count:
            mesh = meshes.item(idx)
            bb = mesh.boundingBox
            return {
                "name": mesh.name, "type": "mesh",
                "triangle_count": mesh.displayMesh.triangleCount if mesh.displayMesh else 0,
                "node_count": mesh.displayMesh.nodeCount if mesh.displayMesh else 0,
                "size_cm": {
                    "x": round(bb.maxPoint.x - bb.minPoint.x, 4),
                    "y": round(bb.maxPoint.y - bb.minPoint.y, 4),
                    "z": round(bb.maxPoint.z - bb.minPoint.z, 4),
                }
            }
    # Search by name
    for i in range(meshes.count):
        if meshes.item(i).name == ref:
            mesh = meshes.item(i)
            bb = mesh.boundingBox
            return {"name": mesh.name, "type": "mesh",
                    "triangle_count": mesh.displayMesh.triangleCount if mesh.displayMesh else 0,
                    "node_count": mesh.displayMesh.nodeCount if mesh.displayMesh else 0}
    return {"error": f"No mesh body '{ref}' found", "mesh_bodies_count": meshes.count}

def _check_printability(root, p):
    body = _find_body(root, p.get("body", 0))
    overhang_threshold = float(p.get("max_overhang_angle", 45.0))
    min_wall = float(p.get("min_wall_thickness", 0.08))  # cm (0.8mm)

    pull = adsk.core.Vector3D.create(0, 0, 1)  # print direction is Z up
    overhangs = []

    for i in range(body.faces.count):
        face = body.faces.item(i)
        try:
            normal = face.geometry.normal if hasattr(face.geometry, 'normal') else None
            if normal:
                dot = normal.x * pull.x + normal.y * pull.y + normal.z * pull.z
                angle_from_vertical = math.degrees(math.acos(min(abs(dot), 1.0)))
                angle_from_horizontal = 90.0 - angle_from_vertical
                # Face pointing downward with angle > threshold
                if dot < 0 and angle_from_horizontal > overhang_threshold:
                    overhangs.append({"face": i, "overhang_angle_deg": round(angle_from_horizontal, 1)})
        except Exception:
            pass

    return {
        "body": body.name,
        "printable": len(overhangs) == 0,
        "overhang_faces": len(overhangs),
        "overhangs": overhangs[:20],
        "overhang_threshold_deg": overhang_threshold,
        "min_wall_threshold_cm": min_wall,
        "note": "Use check_wall_thickness for detailed wall analysis"
    }


# ---- CAM Commands ----

def _cam():
    """Get the CAM product from the active document."""
    cam = adsk.cam.CAM.cast(app.activeProduct)
    if not cam:
        # Try to get CAM from the document's products
        doc = app.activeDocument
        for i in range(doc.products.count):
            product = doc.products.item(i)
            if product.productType == 'CAMProductType':
                return adsk.cam.CAM.cast(product)
        raise Exception("CAM workspace not available. Switch to Manufacturing workspace first.")
    return cam

def _find_setup(cam, ref):
    """Find setup by name or index."""
    setups = cam.setups
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < setups.count:
            return setups.item(idx)
    for i in range(setups.count):
        if setups.item(i).name == ref:
            return setups.item(i)
    raise Exception(f"Setup '{ref}' not found. Available: {[setups.item(i).name for i in range(setups.count)]}")

def _find_operation(setup, ref):
    """Find operation within a setup by name or index."""
    ops = setup.operations
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < ops.count:
            return ops.item(idx)
    for i in range(ops.count):
        if ops.item(i).name == ref:
            return ops.item(i)
    raise Exception(f"Operation '{ref}' not found in setup '{setup.name}'")

def _cam_get_setups(p):
    cam = _cam()
    setups = []
    for i in range(cam.setups.count):
        s = cam.setups.item(i)
        ops = []
        for j in range(s.operations.count):
            op = s.operations.item(j)
            ops.append({
                "name": op.name,
                "type": type(op).__name__,
                "hasToolpath": op.hasToolpath,
                "isSuppressed": op.isSuppressed,
            })
        setups.append({
            "name": s.name,
            "type": str(s.setupType) if hasattr(s, 'setupType') else "unknown",
            "operations_count": s.operations.count,
            "operations": ops,
        })
    return {"setups": setups, "count": len(setups)}

def _cam_create_setup(root, p):
    cam = _cam()
    setup_type_str = p.get("setup_type", "milling").lower()

    # Map string to SetupTypes enum
    setup_types = {
        "milling": adsk.cam.SetupTypes.MillingSetupType,
        "turning": adsk.cam.SetupTypes.TurningSetupType,
        "cutting": adsk.cam.SetupTypes.CuttingSetupType,
    }
    setup_type = setup_types.get(setup_type_str, adsk.cam.SetupTypes.MillingSetupType)

    setup_input = cam.setups.createInput(setup_type)

    # Set body if specified
    body_ref = p.get("body", None)
    if body_ref is not None:
        body = _find_body(root, body_ref)
        setup_input.models = [body]

    setup = cam.setups.add(setup_input)

    name = p.get("name", None)
    if name:
        setup.name = name

    return {"created": setup.name, "type": setup_type_str}

def _cam_create_operation(p):
    cam = _cam()
    setup_name = p.get("setup", "")
    setup = _find_setup(cam, setup_name)

    op_type = p.get("operation_type", "pocket2d")

    # Map operation types to their Fusion operation IDs
    op_map = {
        "pocket2d": "pocket2d",
        "contour2d": "contour2d",
        "adaptive2d": "adaptive2d",
        "face": "face",
        "drill": "drill",
        "adaptive3d": "adaptive",
        "pocket3d": "pocket",
        "parallel3d": "parallel",
        "contour3d": "contour",
        "scallop": "scallop",
        "pencil": "pencil",
        "steep_and_shallow": "steep_and_shallow",
    }

    fusion_op_type = op_map.get(op_type, op_type)

    try:
        op_input = setup.operations.createInput(fusion_op_type)
        op = setup.operations.add(op_input)

        name = p.get("name", None)
        if name:
            op.name = name

        return {"created": op.name, "type": op_type, "setup": setup.name}
    except Exception as e:
        return {"error": str(e), "available_types": list(op_map.keys())}

def _cam_generate_toolpath(p):
    cam = _cam()

    setup_name = p.get("setup", None)
    op_name = p.get("operation", None)

    if op_name and setup_name:
        setup = _find_setup(cam, setup_name)
        op = _find_operation(setup, op_name)
        future = cam.generateToolpath(op)
    elif setup_name:
        setup = _find_setup(cam, setup_name)
        future = cam.generateToolpath(setup)
    else:
        # Generate all
        future = cam.generateAllToolpaths(False)

    # Wait for generation with timeout to prevent infinite hang
    max_wait = 300  # seconds
    start_time = time.monotonic()
    while not future.isGenerationCompleted:
        if time.monotonic() - start_time > max_wait:
            return {"error": f"Toolpath generation timed out after {max_wait}s", "generated": False}
        adsk.doEvents()

    return {"generated": True, "status": "completed"}

def _cam_list_tools(p):
    cam = _cam()
    library_url = p.get("library", "")
    search = p.get("search", "")

    # Get tool libraries
    tool_libs = cam.toolLibraries

    # List available library URLs
    lib_urls = []
    try:
        for lib_url in tool_libs.toolLibraryUrls:
            lib_urls.append(str(lib_url))
    except Exception:
        pass

    if not library_url and lib_urls:
        return {"libraries": lib_urls, "note": "Provide 'library' parameter to list tools from a specific library"}

    if library_url:
        try:
            url = adsk.core.URL.create(library_url)
            lib = tool_libs.toolLibraryAtURL(url)
            tools = []
            for i in range(lib.count):
                tool = lib.item(i)
                tool_info = {"index": i, "description": tool.description if hasattr(tool, 'description') else str(i)}
                if search and search.lower() not in str(tool_info).lower():
                    continue
                tools.append(tool_info)
            return {"library": library_url, "tools": tools[:50], "total": lib.count}
        except Exception as e:
            return {"error": str(e), "available_libraries": lib_urls}

    return {"libraries": lib_urls}

def _validate_output_path(path):
    """Validate that an output path is within the user's home directory tree."""
    resolved = os.path.realpath(path)
    home = os.path.realpath(os.path.expanduser("~"))
    if not resolved.startswith(home + os.sep) and resolved != home:
        raise Exception(f"Path '{path}' is outside the user home directory. Only paths under '{home}' are allowed.")
    return resolved

def _cam_post_process(p):
    cam = _cam()
    setup_name = p.get("setup", "")
    setup = _find_setup(cam, setup_name)

    post_config = p.get("post_config", "")
    output_folder = p.get("output_folder", os.path.expanduser("~/Desktop"))
    output_name = p.get("output_name", setup.name)

    if not post_config:
        raise Exception("Parameter 'post_config' is required (path to .cps post processor file)")

    # Security: validate paths to prevent path traversal (CWE-22)
    _validate_output_path(output_folder)
    if not os.path.isfile(post_config):
        raise Exception(f"Post processor config file not found: {post_config}")

    # Create post process input
    post_input = adsk.cam.PostProcessInput.create(output_name, post_config, output_folder, adsk.cam.PostOutputUnitOptions.MillimeterOutput)
    post_input.isOpenInEditor = False

    cam.postProcess(setup, post_input)

    return {"posted": True, "setup": setup.name, "output_folder": output_folder, "output_name": output_name}

def _cam_simulate(p):
    cam = _cam()
    setup_name = p.get("setup", "")
    setup = _find_setup(cam, setup_name)

    try:
        cam.simulate(setup)
        return {"simulating": True, "setup": setup.name}
    except Exception as e:
        return {"error": str(e), "note": "Simulation may require generated toolpaths"}

def _cam_get_operation_info(p):
    cam = _cam()
    setup_name = p.get("setup", "")
    op_name = p.get("operation", "")

    setup = _find_setup(cam, setup_name)
    op = _find_operation(setup, op_name)

    info = {
        "name": op.name,
        "type": type(op).__name__,
        "hasToolpath": op.hasToolpath,
        "isSuppressed": op.isSuppressed,
    }

    # Try to get additional details
    try:
        if op.hasToolpath:
            info["machining_time_sec"] = round(op.machiningTime, 2) if hasattr(op, 'machiningTime') else None
    except Exception:
        pass

    try:
        tool = op.tool
        if tool:
            info["tool"] = {
                "description": tool.description if hasattr(tool, 'description') else None,
                "diameter": round(tool.diameter, 4) if hasattr(tool, 'diameter') else None,
            }
    except Exception:
        pass

    return info


# ---- Parametric Design Automation ----

def _edit_feature(p):
    """Edit an existing feature by timeline index or name."""
    design = _design()
    tl = design.timeline
    ref = p.get("feature", "")
    params = p.get("params", {})

    # Find timeline item
    item = None
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < tl.count:
            item = tl.item(idx)
    else:
        for i in range(tl.count):
            ti = tl.item(i)
            if hasattr(ti.entity, 'name') and ti.entity.name == ref:
                item = ti
                break

    if not item:
        raise Exception(f"Feature '{ref}' not found in timeline")

    entity = item.entity

    # Try to edit the feature's parameters (block unsafe attribute names)
    edited = []
    for key, value in params.items():
        # Security: reject dunder/private attributes to prevent CWE-915
        if key.startswith("_") or key.startswith("__"):
            edited.append(f"{key}: blocked (private/dunder attributes not allowed)")
            continue
        try:
            if hasattr(entity, key):
                setattr(entity, key, value)
                edited.append(key)
        except Exception as e:
            edited.append(f"{key}: failed ({str(e)[:50]})")

    return {"feature": entity.name if hasattr(entity, 'name') else str(ref), "edited": edited}

def _suppress_feature(p):
    """Toggle suppression of a timeline feature."""
    design = _design()
    tl = design.timeline
    ref = p.get("feature", "")
    suppress = p.get("suppress", True)

    item = None
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        item = tl.item(int(ref))
    else:
        for i in range(tl.count):
            ti = tl.item(i)
            if hasattr(ti.entity, 'name') and ti.entity.name == ref:
                item = ti
                break

    if not item:
        raise Exception(f"Feature '{ref}' not found")

    item.isSuppressed = suppress
    name = item.entity.name if hasattr(item.entity, 'name') else str(ref)
    return {"feature": name, "suppressed": item.isSuppressed}

def _delete_feature(p):
    """Delete a feature by timeline index or name."""
    design = _design()
    tl = design.timeline
    ref = p.get("feature", "")

    item = None
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        item = tl.item(int(ref))
    else:
        for i in range(tl.count):
            ti = tl.item(i)
            if hasattr(ti.entity, 'name') and ti.entity.name == ref:
                item = ti
                break

    if not item:
        raise Exception(f"Feature '{ref}' not found")

    name = item.entity.name if hasattr(item.entity, 'name') else str(ref)
    item.entity.deleteMe()
    return {"deleted": name}

def _reorder_feature(p):
    """Move a feature to a new position in the timeline."""
    design = _design()
    tl = design.timeline
    ref = p.get("feature", "")
    new_position = int(p.get("position", 0))

    item = None
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        item = tl.item(int(ref))
    else:
        for i in range(tl.count):
            ti = tl.item(i)
            if hasattr(ti.entity, 'name') and ti.entity.name == ref:
                item = ti
                break

    if not item:
        raise Exception(f"Feature '{ref}' not found")

    name = item.entity.name if hasattr(item.entity, 'name') else str(ref)
    item.moveToPosition(new_position)
    return {"feature": name, "new_position": new_position}

def _auto_constrain(root, p):
    """Apply automatic constraints to a sketch."""
    sketch = _resolve_sketch(root, p)
    tolerance = float(p.get("tolerance", 0.1))  # cm

    try:
        sketch.autoConstrain(tolerance)
        # Report constraint count
        count = sketch.geometricConstraints.count
        dof = sketch.sketchDimensions.count  # approximate
        return {"sketch": sketch.name, "constraints_after": count, "auto_constrained": True}
    except Exception as e:
        return {"sketch": sketch.name, "error": str(e)}

def _get_sketch_health(root, p):
    """Get constraint health of a sketch (DOF, fully constrained status)."""
    sketch = _resolve_sketch(root, p)

    constraints = sketch.geometricConstraints.count
    dimensions = sketch.sketchDimensions.count
    profiles = sketch.profiles.count

    # Count curves
    curves = 0
    for attr in ['sketchLines', 'sketchCircles', 'sketchArcs', 'sketchEllipses', 'sketchFittedSplines']:
        curves += getattr(sketch.sketchCurves, attr).count

    return {
        "sketch": sketch.name,
        "is_fully_constrained": sketch.isFullyConstrained if hasattr(sketch, 'isFullyConstrained') else None,
        "constraints": constraints,
        "dimensions": dimensions,
        "curves": curves,
        "profiles": profiles,
    }

def _remove_constraint(root, p):
    """Remove a geometric constraint by index."""
    sketch = _resolve_sketch(root, p)
    idx = int(p.get("index", 0))

    if idx >= sketch.geometricConstraints.count:
        raise Exception(f"Constraint index {idx} out of range (max {sketch.geometricConstraints.count - 1})")

    constraint = sketch.geometricConstraints.item(idx)
    ctype = type(constraint).__name__
    constraint.deleteMe()
    return {"deleted_constraint": idx, "type": ctype, "sketch": sketch.name}

def _save_variant(p):
    """Save current parameter values as a named variant."""
    design = _design()
    name = p.get("name", "")
    if not name:
        raise Exception("Parameter 'name' is required")

    # Collect all user parameters
    params = {}
    for i in range(design.userParameters.count):
        param = design.userParameters.item(i)
        params[param.name] = {"value": param.value, "unit": param.unit, "expression": param.expression}

    # Store in design attributes
    attr_group = "FusionMCP_Variants"
    design.attributes.add(attr_group, name, json.dumps(params))

    return {"saved": name, "parameter_count": len(params)}

def _load_variant(p):
    """Load a named variant, updating all parameters."""
    design = _design()
    name = p.get("name", "")
    if not name:
        raise Exception("Parameter 'name' is required")

    attr = design.attributes.itemByName("FusionMCP_Variants", name)
    if not attr:
        raise Exception(f"Variant '{name}' not found")

    params = json.loads(attr.value)
    updated = []
    errors = []

    for pname, pdata in params.items():
        try:
            param = design.userParameters.itemByName(pname)
            if param:
                param.expression = pdata["expression"]
                updated.append(pname)
        except Exception as e:
            errors.append(f"{pname}: {str(e)[:50]}")

    return {"loaded": name, "updated": len(updated), "errors": errors}

def _list_variants(p):
    """List all saved variants."""
    design = _design()
    variants = []
    attrs = design.attributes.itemsByGroup("FusionMCP_Variants")
    for i in range(attrs.count):
        attr = attrs.item(i)
        param_data = json.loads(attr.value)
        variants.append({"name": attr.name, "parameters": len(param_data)})
    return {"variants": variants, "count": len(variants)}

def _delete_variant(p):
    """Delete a named variant."""
    design = _design()
    name = p.get("name", "")
    attr = design.attributes.itemByName("FusionMCP_Variants", name)
    if not attr:
        raise Exception(f"Variant '{name}' not found")
    attr.deleteMe()
    return {"deleted": name}

def _add_parameter_expression(p):
    """Add a user parameter with an expression (can reference other parameters)."""
    design = _design()
    name = p.get("name", "")
    expression = p.get("expression", "")
    unit = p.get("unit", "mm")
    if not name or not expression:
        raise Exception("Parameters 'name' and 'expression' are required")

    param = design.userParameters.add(name, adsk.core.ValueInput.createByString(expression), unit, "")
    return {"name": param.name, "value": round(param.value, 6), "expression": param.expression, "unit": param.unit}

def _batch_update_parameters(p):
    """Update multiple parameters in one call with single recompute."""
    design = _design()
    updates = p.get("parameters", {})  # {name: expression_or_value, ...}

    if not updates:
        raise Exception("Parameter 'parameters' dict is required")

    updated = []
    errors = []

    for pname, expr in updates.items():
        try:
            param = design.allParameters.itemByName(pname)
            if param:
                if isinstance(expr, (int, float)):
                    param.value = float(expr)
                else:
                    param.expression = str(expr)
                updated.append(pname)
            else:
                errors.append(f"{pname}: not found")
        except Exception as e:
            errors.append(f"{pname}: {str(e)[:50]}")

    return {"updated": len(updated), "parameters": updated, "errors": errors}

def _generate_bom(root, p):
    """Generate Bill of Materials from assembly."""
    design = _design()
    include_mass = p.get("include_mass", False)

    bom = []
    comp_counts = {}

    # Count occurrences of each component
    for i in range(root.allOccurrences.count):
        occ = root.allOccurrences.item(i)
        comp_name = occ.component.name
        if comp_name not in comp_counts:
            comp_counts[comp_name] = {"count": 0, "component": occ.component}
        comp_counts[comp_name]["count"] += 1

    for comp_name, data in comp_counts.items():
        comp = data["component"]
        entry = {
            "component": comp_name,
            "quantity": data["count"],
            "bodies": comp.bRepBodies.count,
        }

        if include_mass:
            try:
                phys = comp.getPhysicalProperties()
                entry["mass_kg"] = round(phys.mass, 6)
                entry["material"] = comp.material.name if comp.material else None
            except Exception:
                pass

        bom.append(entry)

    bom.sort(key=lambda x: x["component"])
    return {"bom": bom, "unique_components": len(bom), "total_occurrences": root.allOccurrences.count}

def _export_bom(root, p):
    """Export BOM to JSON or CSV file."""
    output_path = p.get("path", os.path.expanduser("~/Desktop/bom.json"))
    _validate_output_path(output_path)
    fmt = p.get("format", "json").lower()

    bom_data = _generate_bom(root, p)

    if fmt == "csv":
        import csv
        if not output_path.endswith(".csv"):
            output_path = output_path.rsplit(".", 1)[0] + ".csv"
        with open(output_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=["component", "quantity", "bodies"])
            writer.writeheader()
            for row in bom_data["bom"]:
                writer.writerow({k: row.get(k, "") for k in ["component", "quantity", "bodies"]})
    else:
        with open(output_path, 'w') as f:
            json.dump(bom_data, f, indent=2)

    return {"exported": output_path, "format": fmt, "components": len(bom_data["bom"])}

def _generate_drawing(root, p):
    """Create a basic drawing with standard views. Note: limited in headless/API mode."""
    # Fusion 360 Drawing API is limited - we create what we can
    body_ref = p.get("body", 0)
    body = _find_body(root, body_ref)

    bb = body.boundingBox
    dims = {
        "width_cm": round(bb.maxPoint.x - bb.minPoint.x, 4),
        "height_cm": round(bb.maxPoint.y - bb.minPoint.y, 4),
        "depth_cm": round(bb.maxPoint.z - bb.minPoint.z, 4),
    }

    # Try to export orthographic screenshot views
    output_path = p.get("path", os.path.expanduser("~/Desktop/drawing.png"))

    try:
        # Capture current viewport
        app.activeViewport.saveAsImageFile(output_path, 1920, 1080)
        return {
            "body": body.name,
            "dimensions": dims,
            "screenshot": output_path,
            "note": "Full 2D drawing generation requires Fusion 360 Drawing workspace. Use FreeCAD batch_drawings.py for multi-view DXF output."
        }
    except Exception as e:
        return {"body": body.name, "dimensions": dims, "error": str(e)}


# ---- 3D Sketch ----

def _create_3d_sketch(root, p):
    sketch = root.sketches.addWithoutPlanarFace()
    name = p.get("name", None)
    if name:
        sketch.name = name
    return {"sketch": sketch.name, "is3D": True}

def _draw_3d_line(root, p):
    sketch = _resolve_sketch(root, p)
    x1, y1, z1 = p.get("x1", 0), p.get("y1", 0), p.get("z1", 0)
    x2, y2, z2 = p.get("x2", 1), p.get("y2", 0), p.get("z2", 0)
    p1 = adsk.core.Point3D.create(x1, y1, z1)
    p2 = adsk.core.Point3D.create(x2, y2, z2)
    line = sketch.sketchCurves.sketchLines.addByTwoPoints(p1, p2)
    return {"line": "3D line", "length": round(line.length, 4)}

def _draw_3d_spline(root, p):
    sketch = _resolve_sketch(root, p)
    points_data = p.get("points", [])
    if len(points_data) < 2:
        raise Exception("Need at least 2 points for a spline")
    points = adsk.core.ObjectCollection.create()
    for pt in points_data:
        points.add(adsk.core.Point3D.create(pt[0], pt[1], pt[2] if len(pt) > 2 else 0))
    spline = sketch.sketchCurves.sketchFittedSplines.add(points)
    return {"spline": "3D spline", "control_points": len(points_data)}

def _project_to_surface(root, p):
    sketch = _resolve_sketch(root, p)
    body = _find_body(root, p.get("body", 0))
    face_idx = int(p.get("face", 0))
    face = body.faces.item(face_idx)
    entities = sketch.projectToSurface(adsk.core.ObjectCollection.create(), face) if hasattr(sketch, 'projectToSurface') else None
    if entities:
        return {"projected": entities.count, "sketch": sketch.name}
    return {"note": "projectToSurface not available, use project() instead", "sketch": sketch.name}


# ---- Advanced Holes ----

def _create_hole_pattern_linear(root, p):
    body = _find_body(root, p.get("body", 0))
    feature_name = p.get("feature", "")
    # Find hole feature in timeline
    design = _design()
    hole_feature = None
    for i in range(root.features.holeFeatures.count):
        hf = root.features.holeFeatures.item(i)
        if hf.name == feature_name or feature_name == "":
            hole_feature = hf
            break
    if not hole_feature:
        raise Exception(f"Hole feature '{feature_name}' not found")

    entities = adsk.core.ObjectCollection.create()
    entities.add(hole_feature)

    qty_one = int(p.get("count_1", 2))
    dist_one = adsk.core.ValueInput.createByReal(float(p.get("distance_1", 1.0)))
    direction_one = root.xConstructionAxis if p.get("direction_1", "X").upper() == "X" else root.yConstructionAxis

    input_pat = root.features.rectangularPatternFeatures.createInput(entities, direction_one)
    input_pat.quantityOne = adsk.core.ValueInput.createByString(str(qty_one))
    input_pat.distanceOne = adsk.fusion.DistanceExtentDefinition.create(dist_one)

    pattern = root.features.rectangularPatternFeatures.add(input_pat)
    return {"pattern": pattern.name, "count": qty_one}

def _create_hole_pattern_circular(root, p):
    feature_name = p.get("feature", "")
    hole_feature = None
    for i in range(root.features.holeFeatures.count):
        hf = root.features.holeFeatures.item(i)
        if hf.name == feature_name or feature_name == "":
            hole_feature = hf
            break
    if not hole_feature:
        raise Exception(f"Hole feature '{feature_name}' not found")

    entities = adsk.core.ObjectCollection.create()
    entities.add(hole_feature)

    axis = _construction_axis(root, p.get("axis", "Z"))
    qty = int(p.get("count", 4))

    input_pat = root.features.circularPatternFeatures.createInput(entities, axis)
    input_pat.quantity = adsk.core.ValueInput.createByString(str(qty))

    pattern = root.features.circularPatternFeatures.add(input_pat)
    return {"pattern": pattern.name, "count": qty}

def _get_hole_info(root, p):
    holes = []
    for i in range(root.features.holeFeatures.count):
        hf = root.features.holeFeatures.item(i)
        info = {"name": hf.name, "index": i}
        try:
            info["diameter_cm"] = round(hf.holeDiameter.value, 4) if hasattr(hf, 'holeDiameter') else None
            info["depth_cm"] = round(hf.depth.value, 4) if hasattr(hf, 'depth') else None
            info["is_through_all"] = hf.extentDefinition.objectType == "adsk::fusion::ThroughAllExtentDefinition" if hasattr(hf, 'extentDefinition') else None
        except Exception:
            pass
        holes.append(info)
    return {"holes": holes, "count": len(holes)}


# ---- Sheet Metal ----

def _ensure_sheet_metal(root):
    """Check if active component uses sheet metal environment."""
    design = _design()
    if design.designType != adsk.fusion.DesignTypes.ParametricDesignType:
        raise Exception("Sheet metal requires parametric design mode")
    return root

def _create_sheet_metal_rule(root, p):
    _ensure_sheet_metal(root)
    thickness = float(p.get("thickness", 0.1))  # cm
    gap = float(p.get("gap", 0.01))  # cm
    name = p.get("name", "SheetMetalRule1")

    smDef = root.features.sheetMetalFeatures if hasattr(root.features, 'sheetMetalFeatures') else None
    if smDef is None:
        return {"note": "Sheet metal features not accessible via API on this component. Use UI to convert to sheet metal first.", "name": name}

    return {"created": name, "thickness_cm": thickness, "gap_cm": gap}

def _sheet_metal_flange(root, p):
    _ensure_sheet_metal(root)
    body = _find_body(root, p.get("body", 0))
    edge_idx = int(p.get("edge", 0))
    height = float(p.get("height", 1.0))  # cm
    angle = float(p.get("angle", 90.0))  # degrees

    edge = body.edges.item(edge_idx)

    flanges = root.features.flangeFeatures if hasattr(root.features, 'flangeFeatures') else None
    if flanges is None:
        return {"error": "Flange features not available. Ensure component is in sheet metal mode."}

    edges = adsk.core.ObjectCollection.create()
    edges.add(edge)

    flange_input = flanges.createInput(edges, False)
    flange_input.setHeight(adsk.core.ValueInput.createByReal(height))
    flange_input.angle = adsk.core.ValueInput.createByReal(math.radians(angle))

    flange = flanges.add(flange_input)
    return {"flange": flange.name, "height_cm": height, "angle_deg": angle}

def _sheet_metal_bend(root, p):
    _ensure_sheet_metal(root)
    body = _find_body(root, p.get("body", 0))
    edge_idx = int(p.get("edge", 0))
    angle = float(p.get("angle", 90.0))

    edge = body.edges.item(edge_idx)
    bends = root.features.bendFeatures if hasattr(root.features, 'bendFeatures') else None
    if bends is None:
        return {"error": "Bend features not available. Ensure sheet metal mode."}

    bend_input = bends.createInput(edge)
    bend_input.angle = adsk.core.ValueInput.createByReal(math.radians(angle))

    bend = bends.add(bend_input)
    return {"bend": bend.name, "angle_deg": angle}

def _sheet_metal_unfold(root, p):
    _ensure_sheet_metal(root)
    body = _find_body(root, p.get("body", 0))

    unfolds = root.features.unfoldFeatures if hasattr(root.features, 'unfoldFeatures') else None
    if unfolds is None:
        return {"error": "Unfold features not available."}

    bends_coll = adsk.core.ObjectCollection.create()
    # Collect all bend faces
    for i in range(body.faces.count):
        f = body.faces.item(i)
        if "Cylinder" in type(f.geometry).__name__:
            bends_coll.add(f)

    stationary_face = body.faces.item(int(p.get("stationary_face", 0)))

    unfold_input = unfolds.createInput(stationary_face, bends_coll)
    unfold = unfolds.add(unfold_input)
    return {"unfold": unfold.name, "bends_unfolded": bends_coll.count}

def _sheet_metal_flat_pattern(root, p):
    _ensure_sheet_metal(root)
    output_path = p.get("path", os.path.expanduser("~/Desktop/flat_pattern.dxf"))
    output_path = _validate_output_path(output_path)

    # Try to get flat pattern
    design = _design()
    body = _find_body(root, p.get("body", 0))

    try:
        em = design.exportManager
        dxf_opts = em.createFlatPatternDXFExportOptions(output_path, body)
        em.execute(dxf_opts)
        return {"exported": output_path, "body": body.name}
    except Exception as e:
        return {"error": str(e), "note": "Ensure body is in sheet metal mode with valid flat pattern"}


# ---- Mesh Operations ----

def _find_mesh_body_helper(root, ref):
    meshes = root.meshBodies
    if isinstance(ref, int) or (isinstance(ref, str) and str(ref).isdigit()):
        idx = int(ref)
        if idx < meshes.count:
            return meshes.item(idx)
    for i in range(meshes.count):
        if meshes.item(i).name == ref:
            return meshes.item(i)
    raise Exception(f"Mesh body '{ref}' not found. Available: {meshes.count} mesh bodies.")

def _import_mesh(root, p):
    filepath = p.get("path", "")
    if not filepath or not os.path.exists(filepath):
        raise Exception(f"File not found: {filepath}")

    import_mgr = app.importManager

    ext = os.path.splitext(filepath)[1].lower()
    if ext == ".stl":
        opts = import_mgr.createSTLImportOptions(filepath)
    elif ext == ".obj":
        opts = import_mgr.createOBJImportOptions(filepath)
    elif ext == ".3mf":
        opts = import_mgr.create3MFImportOptions(filepath)
    else:
        raise Exception(f"Unsupported mesh format: {ext}. Use .stl, .obj, or .3mf")

    import_mgr.importToTarget(opts, root)
    return {"imported": os.path.basename(filepath), "format": ext, "mesh_bodies": root.meshBodies.count}

def _repair_mesh(root, p):
    mesh = _find_mesh_body_helper(root, p.get("body", 0))

    try:
        mesh_body = mesh
        display_mesh = mesh_body.displayMesh
        before_triangles = display_mesh.triangleCount if display_mesh else 0

        # Fusion 360 mesh repair is limited via API
        # Try using the mesh body's repair capabilities
        if hasattr(mesh_body, 'repair'):
            mesh_body.repair()
            after_mesh = mesh_body.displayMesh
            after_triangles = after_mesh.triangleCount if after_mesh else 0
            return {"body": mesh_body.name, "repaired": True, "triangles_before": before_triangles, "triangles_after": after_triangles}

        return {"body": mesh_body.name, "note": "Mesh repair not available via API. Use Mesh workspace UI.", "triangles": before_triangles}
    except Exception as e:
        return {"error": str(e)}

def _convert_mesh_to_brep(root, p):
    mesh = _find_mesh_body_helper(root, p.get("body", 0))
    refinement = p.get("refinement", "medium").lower()

    try:
        # Use mesh to BRep conversion
        meshes = adsk.core.ObjectCollection.create()
        meshes.add(mesh)

        mesh_to_brep = root.features.meshToBrep if hasattr(root.features, 'meshToBrep') else None
        if mesh_to_brep is None:
            return {"error": "Mesh to BRep conversion not available. Requires Fusion 360 with mesh workspace."}

        input_m = mesh_to_brep.createInput(meshes)
        result_feat = mesh_to_brep.add(input_m)

        return {"converted": mesh.name, "refinement": refinement, "bodies_created": result_feat.bodies.count if hasattr(result_feat, 'bodies') else 1}
    except Exception as e:
        return {"error": str(e), "note": "Mesh to BRep may require specific mesh quality"}


# ---- Advanced Joints ----

def _create_joint_from_geometry(root, p):
    """Create a joint using JointGeometry from specific geometric references."""
    occ1_name = p.get("occurrence1", "")
    occ2_name = p.get("occurrence2", "")
    joint_type = p.get("joint_type", "rigid").lower()

    occ1 = _find_occurrence(root, occ1_name)
    occ2 = _find_occurrence(root, occ2_name)

    joint_types = {
        "rigid": adsk.fusion.JointTypes.RigidJointType,
        "revolute": adsk.fusion.JointTypes.RevoluteJointType,
        "slider": adsk.fusion.JointTypes.SliderJointType,
        "cylindrical": adsk.fusion.JointTypes.CylindricalJointType,
        "pin_slot": adsk.fusion.JointTypes.PinSlotJointType,
        "ball": adsk.fusion.JointTypes.BallJointType,
        "planar": adsk.fusion.JointTypes.PlanarJointType,
    }

    # Build geometry references for occurrence 1
    geo1_type = p.get("geometry1_type", "point").lower()
    if geo1_type == "edge" and "edge1_index" in p:
        body1 = occ1.component.bRepBodies.item(int(p.get("body1_index", 0)))
        edge1 = body1.edges.item(int(p["edge1_index"]))
        geo1 = adsk.fusion.JointGeometry.createByNonPlanarFace(edge1) if hasattr(adsk.fusion.JointGeometry, 'createByNonPlanarFace') else adsk.fusion.JointGeometry.createByPoint(occ1)
    elif geo1_type == "face" and "face1_index" in p:
        body1 = occ1.component.bRepBodies.item(int(p.get("body1_index", 0)))
        face1 = body1.faces.item(int(p["face1_index"]))
        geo1_kf = adsk.fusion.JointKeyPointTypes.CenterKeyPoint
        geo1 = adsk.fusion.JointGeometry.createByPlanarFace(face1, None, geo1_kf)
    else:
        geo1 = adsk.fusion.JointGeometry.createByPoint(occ1)

    # Build geometry references for occurrence 2
    geo2_type = p.get("geometry2_type", "point").lower()
    if geo2_type == "edge" and "edge2_index" in p:
        body2 = occ2.component.bRepBodies.item(int(p.get("body2_index", 0)))
        edge2 = body2.edges.item(int(p["edge2_index"]))
        geo2 = adsk.fusion.JointGeometry.createByNonPlanarFace(edge2) if hasattr(adsk.fusion.JointGeometry, 'createByNonPlanarFace') else adsk.fusion.JointGeometry.createByPoint(occ2)
    elif geo2_type == "face" and "face2_index" in p:
        body2 = occ2.component.bRepBodies.item(int(p.get("body2_index", 0)))
        face2 = body2.faces.item(int(p["face2_index"]))
        geo2_kf = adsk.fusion.JointKeyPointTypes.CenterKeyPoint
        geo2 = adsk.fusion.JointGeometry.createByPlanarFace(face2, None, geo2_kf)
    else:
        geo2 = adsk.fusion.JointGeometry.createByPoint(occ2)

    ji = root.joints.createInput(geo1, geo2)

    if joint_type == "rigid":
        ji.setAsRigidJointMotion()
    elif joint_type == "revolute":
        ji.setAsRevoluteJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "slider":
        ji.setAsSliderJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "cylindrical":
        ji.setAsCylindricalJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)
    elif joint_type == "pin_slot":
        ji.setAsPinSlotJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection, adsk.fusion.JointDirections.XAxisJointDirection)
    elif joint_type == "ball":
        ji.setAsBallJointMotion()
    elif joint_type == "planar":
        ji.setAsPlanarJointMotion(adsk.fusion.JointDirections.ZAxisJointDirection)

    jt = joint_types.get(joint_type, adsk.fusion.JointTypes.RigidJointType)
    joint = root.joints.add(ji)
    return {"joint": joint.name, "type": joint_type,
            "occurrence1": occ1.name, "occurrence2": occ2.name}

def _create_rigid_group(root, p):
    """Group multiple occurrences as rigid."""
    occ_names = p.get("occurrences", [])
    if not occ_names:
        return {"error": "Provide 'occurrences' list of occurrence names."}

    occ_collection = adsk.core.ObjectCollection.create()
    resolved_names = []
    for name in occ_names:
        occ = _find_occurrence(root, name)
        occ_collection.add(occ)
        resolved_names.append(occ.name)

    rg = root.rigidGroups.add(occ_collection)
    return {"rigid_group": rg.name, "occurrences": resolved_names}

def _set_joint_limits(root, p):
    """Set motion limits on a joint."""
    joint_name = p.get("joint", "")
    joint = None
    for i in range(root.joints.count):
        j = root.joints.item(i)
        if j.name == joint_name:
            joint = j
            break
    if not joint:
        return {"error": f"Joint '{joint_name}' not found. Available: {[root.joints.item(i).name for i in range(root.joints.count)]}"}

    motion = joint.jointMotion
    result = {"joint": joint.name, "type": type(motion).__name__}

    # Revolute / cylindrical rotation limits
    if hasattr(motion, 'rotationLimits'):
        rl = motion.rotationLimits
        if "min_rotation" in p:
            rl.isMinimumValueEnabled = True
            rl.minimumValue = float(p["min_rotation"])
        if "max_rotation" in p:
            rl.isMaximumValueEnabled = True
            rl.maximumValue = float(p["max_rotation"])
        result["rotation_limits"] = {"min": rl.minimumValue if rl.isMinimumValueEnabled else None,
                                     "max": rl.maximumValue if rl.isMaximumValueEnabled else None}

    # Slider / cylindrical slide limits
    if hasattr(motion, 'slideLimits'):
        sl = motion.slideLimits
        if "min_slide" in p:
            sl.isMinimumValueEnabled = True
            sl.minimumValue = float(p["min_slide"])
        if "max_slide" in p:
            sl.isMaximumValueEnabled = True
            sl.maximumValue = float(p["max_slide"])
        result["slide_limits"] = {"min": sl.minimumValue if sl.isMinimumValueEnabled else None,
                                  "max": sl.maximumValue if sl.isMaximumValueEnabled else None}

    return result

def _drive_joint(root, p):
    """Set joint motion value directly."""
    joint_name = p.get("joint", "")
    joint = None
    for i in range(root.joints.count):
        j = root.joints.item(i)
        if j.name == joint_name:
            joint = j
            break
    if not joint:
        return {"error": f"Joint '{joint_name}' not found."}

    motion = joint.jointMotion
    result = {"joint": joint.name, "type": type(motion).__name__}

    if "rotation_value" in p and hasattr(motion, 'rotationValue'):
        motion.rotationValue = float(p["rotation_value"])
        result["rotation_value"] = motion.rotationValue

    if "slide_value" in p and hasattr(motion, 'slideValue'):
        motion.slideValue = float(p["slide_value"])
        result["slide_value"] = motion.slideValue

    return result

def _edit_joint(root, p):
    """Edit joint properties."""
    joint_name = p.get("joint", "")
    joint = None
    for i in range(root.joints.count):
        j = root.joints.item(i)
        if j.name == joint_name:
            joint = j
            break
    if not joint:
        return {"error": f"Joint '{joint_name}' not found."}

    result = {"joint": joint.name}

    if "new_name" in p:
        joint.name = p["new_name"]
        result["renamed_to"] = joint.name

    if "suppress" in p:
        joint.isSuppressed = bool(p["suppress"])
        result["suppressed"] = joint.isSuppressed

    if "offset" in p:
        off = p["offset"]
        motion = joint.jointMotion
        if hasattr(motion, 'slideValue') and isinstance(off, (int, float)):
            motion.slideValue = float(off)
            result["offset_applied"] = float(off)

    return result

def _get_joint_info(root, p):
    """Get detailed joint information."""
    ref = p.get("joint", p.get("index", 0))
    joint = None

    if isinstance(ref, int) or (isinstance(ref, str) and ref.isdigit()):
        idx = int(ref)
        if idx < root.joints.count:
            joint = root.joints.item(idx)
    else:
        for i in range(root.joints.count):
            j = root.joints.item(i)
            if j.name == ref:
                joint = j
                break

    if not joint:
        return {"error": f"Joint '{ref}' not found."}

    motion = joint.jointMotion
    info = {
        "name": joint.name,
        "type": type(motion).__name__,
        "is_suppressed": joint.isSuppressed,
        "occurrence1": joint.occurrenceOne.name if joint.occurrenceOne else None,
        "occurrence2": joint.occurrenceTwo.name if joint.occurrenceTwo else None,
    }

    if hasattr(motion, 'rotationValue'):
        info["rotation_value"] = motion.rotationValue
    if hasattr(motion, 'slideValue'):
        info["slide_value"] = motion.slideValue
    if hasattr(motion, 'rotationLimits'):
        rl = motion.rotationLimits
        info["rotation_limits"] = {
            "min_enabled": rl.isMinimumValueEnabled,
            "min": rl.minimumValue if rl.isMinimumValueEnabled else None,
            "max_enabled": rl.isMaximumValueEnabled,
            "max": rl.maximumValue if rl.isMaximumValueEnabled else None,
        }
    if hasattr(motion, 'slideLimits'):
        sl = motion.slideLimits
        info["slide_limits"] = {
            "min_enabled": sl.isMinimumValueEnabled,
            "min": sl.minimumValue if sl.isMinimumValueEnabled else None,
            "max_enabled": sl.isMaximumValueEnabled,
            "max": sl.maximumValue if sl.isMaximumValueEnabled else None,
        }

    return info

def _list_joints(root, p):
    """List all joints and as-built joints."""
    joints = []
    for i in range(root.joints.count):
        j = root.joints.item(i)
        joints.append({
            "name": j.name,
            "type": type(j.jointMotion).__name__,
            "is_suppressed": j.isSuppressed,
            "occurrence1": j.occurrenceOne.name if j.occurrenceOne else None,
            "occurrence2": j.occurrenceTwo.name if j.occurrenceTwo else None,
        })

    as_built = []
    for i in range(root.asBuiltJoints.count):
        abj = root.asBuiltJoints.item(i)
        as_built.append({
            "name": abj.name,
            "type": type(abj.jointMotion).__name__,
            "is_suppressed": abj.isSuppressed,
        })

    rigid_groups = []
    for i in range(root.rigidGroups.count):
        rg = root.rigidGroups.item(i)
        rigid_groups.append({
            "name": rg.name,
            "is_suppressed": rg.isSuppressed,
        })

    return {"joints": joints, "as_built_joints": as_built,
            "rigid_groups": rigid_groups,
            "total_joints": len(joints), "total_as_built": len(as_built)}


# ---- Data Management ----

def _list_data_hubs(p):
    """List available data hubs."""
    hubs = app.data.dataHubs
    result = []
    for i in range(hubs.count):
        hub = hubs.item(i)
        result.append({
            "name": hub.name,
            "id": hub.id,
        })
    return {"hubs": result, "count": len(result)}

def _list_data_projects(p):
    """List projects in a hub. Uses active hub if none specified."""
    hub_id = p.get("hub_id", "")
    hub = None
    if hub_id:
        for i in range(app.data.dataHubs.count):
            h = app.data.dataHubs.item(i)
            if h.id == hub_id:
                hub = h
                break
    if not hub:
        hub = app.data.activeHub
    if not hub:
        return {"error": "No hub found. Specify hub_id or ensure an active hub exists."}

    projects = hub.dataProjects
    result = []
    for i in range(projects.count):
        proj = projects.item(i)
        result.append({
            "name": proj.name,
            "id": proj.id,
        })
    return {"hub": hub.name, "projects": result, "count": len(result)}

def _list_data_folders(p):
    """List folders and files in a data folder."""
    project_id = p.get("project_id", "")
    folder_id = p.get("folder_id", "")

    # Find project
    project = None
    hub = app.data.activeHub
    if hub:
        for i in range(hub.dataProjects.count):
            proj = hub.dataProjects.item(i)
            if proj.id == project_id or proj.name == project_id:
                project = proj
                break
    if not project:
        return {"error": f"Project '{project_id}' not found."}

    # Navigate to folder or use root
    folder = project.rootFolder
    if folder_id:
        for i in range(folder.dataFolders.count):
            f = folder.dataFolders.item(i)
            if f.id == folder_id or f.name == folder_id:
                folder = f
                break

    folders = []
    for i in range(folder.dataFolders.count):
        f = folder.dataFolders.item(i)
        folders.append({"name": f.name, "id": f.id})

    files = []
    for i in range(folder.dataFiles.count):
        df = folder.dataFiles.item(i)
        files.append({
            "name": df.name,
            "id": df.id,
            "version_number": df.versionNumber,
        })

    return {"folder": folder.name, "folders": folders, "files": files,
            "folder_count": len(folders), "file_count": len(files)}

def _open_data_file(p):
    """Open a data file by ID."""
    file_id = p.get("file_id", "")
    if not file_id:
        return {"error": "Provide 'file_id' parameter."}

    # Search across hubs/projects for the file
    data_file = None
    hub = app.data.activeHub
    if hub:
        for i in range(hub.dataProjects.count):
            proj = hub.dataProjects.item(i)
            data_file = _search_folder_for_file(proj.rootFolder, file_id)
            if data_file:
                break

    if not data_file:
        return {"error": f"Data file '{file_id}' not found."}

    doc = app.documents.open(data_file)
    return {"opened": data_file.name, "id": data_file.id,
            "document_name": doc.name}

def _search_folder_for_file(folder, file_id):
    """Recursively search a folder tree for a file by ID or name."""
    for i in range(folder.dataFiles.count):
        df = folder.dataFiles.item(i)
        if df.id == file_id or df.name == file_id:
            return df
    for i in range(folder.dataFolders.count):
        result = _search_folder_for_file(folder.dataFolders.item(i), file_id)
        if result:
            return result
    return None

def _save_to_folder(p):
    """Save current document to a specific folder."""
    name = p.get("name", "")
    project_id = p.get("project_id", "")
    folder_id = p.get("folder_id", "")
    description = p.get("description", "")
    tag = p.get("tag", "")

    if not name:
        return {"error": "Provide 'name' parameter."}

    doc = app.activeDocument
    if not doc:
        return {"error": "No active document."}

    # Find target folder
    project = None
    hub = app.data.activeHub
    if hub:
        for i in range(hub.dataProjects.count):
            proj = hub.dataProjects.item(i)
            if proj.id == project_id or proj.name == project_id:
                project = proj
                break
    if not project:
        return {"error": f"Project '{project_id}' not found."}

    folder = project.rootFolder
    if folder_id:
        for i in range(folder.dataFolders.count):
            f = folder.dataFolders.item(i)
            if f.id == folder_id or f.name == folder_id:
                folder = f
                break

    doc.saveAs(name, folder, description, tag)
    return {"saved": name, "folder": folder.name, "project": project.name,
            "description": description}

def _list_data_versions(p):
    """List versions of a data file."""
    file_id = p.get("file_id", "")
    if not file_id:
        return {"error": "Provide 'file_id' parameter."}

    data_file = None
    hub = app.data.activeHub
    if hub:
        for i in range(hub.dataProjects.count):
            proj = hub.dataProjects.item(i)
            data_file = _search_folder_for_file(proj.rootFolder, file_id)
            if data_file:
                break

    if not data_file:
        return {"error": f"Data file '{file_id}' not found."}

    versions = data_file.versions
    result = []
    for i in range(versions.count):
        v = versions.item(i)
        result.append({
            "version_number": v.versionNumber,
            "id": v.id,
            "description": v.description if hasattr(v, 'description') else "",
            "date_created": str(v.dateCreated) if hasattr(v, 'dateCreated') else "",
        })

    return {"file": data_file.name, "versions": result, "count": len(result)}

def _open_data_version(p):
    """Open a specific version of a data file."""
    file_id = p.get("file_id", "")
    version_number = p.get("version_number", None)
    version_id = p.get("version_id", "")

    if not file_id:
        return {"error": "Provide 'file_id' parameter."}

    data_file = None
    hub = app.data.activeHub
    if hub:
        for i in range(hub.dataProjects.count):
            proj = hub.dataProjects.item(i)
            data_file = _search_folder_for_file(proj.rootFolder, file_id)
            if data_file:
                break

    if not data_file:
        return {"error": f"Data file '{file_id}' not found."}

    # Find the specific version
    target_version = None
    versions = data_file.versions
    for i in range(versions.count):
        v = versions.item(i)
        if version_id and v.id == version_id:
            target_version = v
            break
        if version_number is not None and v.versionNumber == int(version_number):
            target_version = v
            break

    if not target_version:
        return {"error": f"Version not found. Available: {[versions.item(i).versionNumber for i in range(versions.count)]}"}

    doc = app.documents.open(target_version)
    is_read_only = not doc.isModified  # Older versions are typically read-only
    return {"opened": data_file.name, "version": target_version.versionNumber,
            "version_id": target_version.id, "read_only": is_read_only,
            "document_name": doc.name}


# ---- Events/Callbacks ----

class _DesignEventHandler(adsk.core.CustomEventHandler):
    """Generic event handler that appends event data to a subscription queue."""
    def __init__(self, sub_id, event_type):
        super().__init__()
        self._sub_id = sub_id
        self._event_type = event_type
        self._seq = 0

    def notify(self, args):
        global _event_queues
        if self._sub_id in _event_queues:
            self._seq += 1
            _event_queues[self._sub_id].append({
                "seq": self._seq,
                "event_type": self._event_type,
                "timestamp": time.time(),
                "info": str(args.additionalInfo) if hasattr(args, 'additionalInfo') else "",
            })

def _subscribe_design_events(p):
    """Subscribe to design events. Returns a subscription_id."""
    global _event_subscriptions, _event_queues, _event_handlers_list

    if len(_event_subscriptions) >= _max_subscriptions:
        return {"error": f"Maximum subscriptions ({_max_subscriptions}) reached. Unsubscribe first."}

    events_list = p.get("events", ["ParameterChanged", "TimelineChanged", "ActiveSelectionChanged"])
    if isinstance(events_list, str):
        events_list = [events_list]

    sub_id = str(uuid.uuid4())[:8]
    _event_queues[sub_id] = collections.deque(maxlen=_max_event_queue)

    registered = []
    design = _design()

    event_map = {}
    if design:
        # Map event names to Fusion design events
        if hasattr(design, 'parameterChanged'):
            event_map["ParameterChanged"] = design.parameterChanged
        if hasattr(design, 'timelineChanged'):
            event_map["TimelineChanged"] = design.timelineChanged
    if app:
        if hasattr(app, 'activeSelectionChanged'):
            event_map["ActiveSelectionChanged"] = app.activeSelectionChanged

    handlers = []
    for evt_name in events_list:
        if evt_name in event_map:
            # Create a custom event to bridge design events to our queue
            handler = _DesignEventHandler(sub_id, evt_name)
            try:
                event_map[evt_name].add(handler)
                handlers.append((evt_name, handler, event_map[evt_name]))
                registered.append(evt_name)
            except Exception as e:
                registered.append(f"{evt_name}(fallback: {str(e)})")
        else:
            # Use polling fallback for events not directly available
            registered.append(f"{evt_name}(polling)")

    _event_subscriptions[sub_id] = {
        "events": events_list,
        "created": time.time(),
        "handlers": handlers,
    }
    _event_handlers_list.extend([h for _, h, _ in handlers])

    return {"subscription_id": sub_id, "registered_events": registered}

def _poll_design_events(p):
    """Return and clear queued events for a subscription."""
    sub_id = p.get("subscription_id", "")
    if not sub_id or sub_id not in _event_queues:
        return {"error": f"Subscription '{sub_id}' not found."}

    q = _event_queues[sub_id]
    events = list(q)
    q.clear()

    return {"subscription_id": sub_id, "events": events, "count": len(events)}

def _unsubscribe_design_events(p):
    """Unsubscribe from design events."""
    global _event_subscriptions, _event_queues, _event_handlers_list

    sub_id = p.get("subscription_id", "")
    if not sub_id or sub_id not in _event_subscriptions:
        return {"error": f"Subscription '{sub_id}' not found."}

    sub = _event_subscriptions[sub_id]

    # Remove event handlers
    for evt_name, handler, event_obj in sub.get("handlers", []):
        try:
            event_obj.remove(handler)
        except Exception:
            pass
        if handler in _event_handlers_list:
            _event_handlers_list.remove(handler)

    del _event_subscriptions[sub_id]
    if sub_id in _event_queues:
        del _event_queues[sub_id]

    return {"unsubscribed": sub_id}

def _get_subscription_status(p):
    """Get status of all active subscriptions."""
    subs = []
    for sub_id, sub in _event_subscriptions.items():
        q = _event_queues.get(sub_id, collections.deque())
        subs.append({
            "subscription_id": sub_id,
            "events": sub["events"],
            "created": sub["created"],
            "queue_depth": len(q),
            "queue_max": _max_event_queue,
        })

    return {"subscriptions": subs, "active_count": len(subs),
            "max_subscriptions": _max_subscriptions}


# ---- Render ----

def _set_render_environment(p):
    design = _design()
    # Access render environment through named values
    env_name = p.get("environment", "")
    brightness = p.get("brightness", None)
    rotation = p.get("rotation", None)
    ground_plane = p.get("ground_plane", None)

    try:
        render_env = design.renderEnvironment if hasattr(design, 'renderEnvironment') else None
        if render_env:
            if env_name:
                render_env.environmentName = env_name
            if brightness is not None:
                render_env.brightness = float(brightness)
            if rotation is not None:
                render_env.rotation = float(rotation)
            if ground_plane is not None:
                render_env.isGroundPlaneVisible = bool(ground_plane)
            return {"environment": env_name or "current", "brightness": brightness, "ground_plane": ground_plane}
        return {"note": "Render environment API not available in this Fusion version. Use UI to configure."}
    except Exception as e:
        return {"error": str(e)}

def _set_appearance_detailed(root, p):
    body = _find_body(root, p.get("body", 0))
    appearance_name = p.get("appearance", "")
    overrides = p.get("overrides", {})

    # Whitelist of safe property names
    safe_props = {"roughness", "metalness", "opacity", "color", "reflectance", "glossiness"}

    if appearance_name:
        # Find and assign appearance
        mat_libs = app.materialLibraries
        for i in range(mat_libs.count):
            lib = mat_libs.item(i)
            try:
                appearances = lib.appearances
                for j in range(appearances.count):
                    app_item = appearances.item(j)
                    if appearance_name.lower() in app_item.name.lower():
                        body.appearance = app_item
                        return {"body": body.name, "appearance": app_item.name, "library": lib.name}
            except Exception:
                continue

    if overrides:
        # Apply property overrides to existing appearance
        try:
            app_copy = body.appearance
            for key, value in overrides.items():
                if key not in safe_props:
                    continue
                # Try to set property on appearance
                for k in range(app_copy.appearanceProperties.count):
                    prop = app_copy.appearanceProperties.item(k)
                    if key.lower() in prop.name.lower():
                        if hasattr(prop, 'value'):
                            prop.value = float(value)
                        break
            return {"body": body.name, "overrides_applied": [k for k in overrides if k in safe_props]}
        except Exception as e:
            return {"error": str(e)}

    return {"error": "Provide 'appearance' name or 'overrides' dict"}

def _create_render_camera(p):
    design = _design()
    name = p.get("name", "Camera1")
    eye = p.get("eye", [10, 10, 10])
    target = p.get("target", [0, 0, 0])
    up = p.get("up", [0, 0, 1])
    perspective_angle = p.get("perspective_angle", 45.0)

    # Store camera as design attribute
    camera_data = {
        "eye": eye, "target": target, "up": up,
        "perspective_angle": perspective_angle
    }
    design.attributes.add("FusionMCP_Cameras", name, json.dumps(camera_data))
    return {"saved_camera": name, "eye": eye, "target": target}

def _apply_render_camera(p):
    design = _design()
    name = p.get("name", "")
    if not name:
        raise Exception("Camera 'name' is required")

    attr = design.attributes.itemByName("FusionMCP_Cameras", name)
    if not attr:
        raise Exception(f"Camera '{name}' not found")

    cam_data = json.loads(attr.value)

    # Apply to viewport
    viewport = app.activeViewport
    camera = viewport.camera
    camera.eye = adsk.core.Point3D.create(*cam_data["eye"])
    camera.target = adsk.core.Point3D.create(*cam_data["target"])
    camera.upVector = adsk.core.Vector3D.create(*cam_data["up"])
    if cam_data.get("perspective_angle"):
        camera.perspectiveAngle = math.radians(cam_data["perspective_angle"])
    camera.isSmoothTransition = True
    viewport.camera = camera

    return {"applied_camera": name}


# ---- Drawing Generation ----

def _create_drawing_document(p):
    """Create a new drawing document. Limited API support - returns what's available."""
    template = p.get("template", "")  # ANSI, ISO
    sheet_size = p.get("sheet_size", "A3")
    title = p.get("title", "")

    try:
        import adsk.drawing
        # Attempt to create drawing via API (limited support)
        return {
            "note": "Drawing document creation has limited API support in Fusion 360. Use the UI: File > New Drawing > From Design, or use FreeCAD batch_drawings.py for automated drawing generation.",
            "template": template,
            "sheet_size": sheet_size
        }
    except ImportError:
        return {"error": "adsk.drawing module not available in this Fusion version"}

def _add_base_view(p):
    """Add base view to drawing. Placeholder - requires drawing document context."""
    return {
        "note": "Drawing view creation requires an active drawing document. Fusion 360 Drawing API is limited to export operations. Use generate_drawing for viewport capture or FreeCAD for full 2D drawings.",
        "body": p.get("body", ""),
        "orientation": p.get("orientation", "front"),
        "scale": p.get("scale", 1.0)
    }

def _add_projected_view(p):
    return {"note": "Requires active drawing document. Use FreeCAD batch_drawings.py for multi-view generation."}

def _add_section_view(p):
    return {"note": "Section views require active drawing document. Use get_section_properties for section analysis data."}

def _add_drawing_dimension(p):
    return {"note": "Drawing dimensions require active drawing context. Use FreeCAD TechDraw for automated dimensioning."}

def _add_annotation(p):
    text = p.get("text", "")
    if len(text) > 500:
        text = text[:500]
    return {"note": "Annotations require active drawing context.", "text_preview": text[:100]}

def _export_drawing(p):
    """Export active drawing to PDF/DWG if a drawing is active."""
    output_path = p.get("path", os.path.expanduser("~/Desktop/drawing.pdf"))
    output_path = _validate_output_path(output_path)
    fmt = p.get("format", "pdf").lower()

    try:
        import adsk.drawing
        doc = app.activeDocument
        product = doc.products.itemByProductType('DrawingProductType')
        if not product:
            return {"error": "No active drawing document. Open a drawing first."}

        drawing = adsk.drawing.Drawing.cast(product)
        export_mgr = drawing.exportManager

        if fmt == "pdf":
            opts = export_mgr.createPDFExportOptions()
            opts.outputPath = output_path
            export_mgr.execute(opts)

        return {"exported": output_path, "format": fmt}
    except Exception as e:
        return {"error": str(e), "note": "Drawing export requires active drawing document"}


# ---- Parametric Automation Phase 2-3 ----

def _compare_variants(p):
    design = _design()
    name1 = p.get("variant1", "")
    name2 = p.get("variant2", "")

    attr1 = design.attributes.itemByName("FusionMCP_Variants", name1)
    attr2 = design.attributes.itemByName("FusionMCP_Variants", name2)

    if not attr1:
        raise Exception(f"Variant '{name1}' not found")
    if not attr2:
        raise Exception(f"Variant '{name2}' not found")

    params1 = json.loads(attr1.value)
    params2 = json.loads(attr2.value)

    deltas = []
    all_params = set(list(params1.keys()) + list(params2.keys()))
    for pname in sorted(all_params):
        v1 = params1.get(pname, {})
        v2 = params2.get(pname, {})
        if v1.get("expression") != v2.get("expression"):
            deltas.append({
                "parameter": pname,
                "variant1": v1.get("expression", "N/A"),
                "variant2": v2.get("expression", "N/A"),
            })

    return {"variant1": name1, "variant2": name2, "differences": deltas, "total_params": len(all_params)}

def _create_design_table(p):
    design = _design()
    name = p.get("name", "")
    rows = p.get("rows", [])  # [{param1: val1, param2: val2, ...}, ...]

    if not name:
        raise Exception("Design table 'name' is required")

    table_data = json.dumps(rows)
    if len(table_data) > 102400:  # 100KB cap
        raise Exception("Design table too large (max 100KB)")

    design.attributes.add("FusionMCP_DesignTables", name, table_data)
    return {"created": name, "rows": len(rows), "parameters": list(rows[0].keys()) if rows else []}

def _apply_design_table_row(p):
    design = _design()
    table_name = p.get("table", "")
    row_idx = int(p.get("row", 0))

    attr = design.attributes.itemByName("FusionMCP_DesignTables", table_name)
    if not attr:
        raise Exception(f"Design table '{table_name}' not found")

    rows = json.loads(attr.value)
    if row_idx >= len(rows):
        raise Exception(f"Row {row_idx} out of range (max {len(rows) - 1})")

    row = rows[row_idx]
    updated = []
    errors = []

    for pname, expr in row.items():
        try:
            param = design.allParameters.itemByName(pname)
            if param:
                if isinstance(expr, (int, float)):
                    param.value = float(expr)
                else:
                    param.expression = str(expr)
                updated.append(pname)
        except Exception as e:
            errors.append(f"{pname}: {str(e)[:50]}")

    return {"table": table_name, "row": row_idx, "updated": len(updated), "errors": errors}

def _generate_variant_batch(root, p):
    design = _design()
    table_name = p.get("table", "")
    output_dir = p.get("output_dir", os.path.expanduser("~/Desktop/variants"))
    output_dir = _validate_output_path(output_dir)
    export_format = p.get("format", "step").lower()

    attr = design.attributes.itemByName("FusionMCP_DesignTables", table_name)
    if not attr:
        raise Exception(f"Design table '{table_name}' not found")

    rows = json.loads(attr.value)
    if len(rows) > 50:
        raise Exception(f"Too many rows ({len(rows)}). Max 50 for batch generation.")

    os.makedirs(output_dir, exist_ok=True)

    results = []
    em = design.exportManager

    for idx, row in enumerate(rows):
        # Apply parameters
        for pname, expr in row.items():
            try:
                param = design.allParameters.itemByName(pname)
                if param:
                    if isinstance(expr, (int, float)):
                        param.value = float(expr)
                    else:
                        param.expression = str(expr)
            except Exception:
                pass

        # Export
        variant_name = p.get("name_prefix", "variant") + f"_{idx}"
        filepath = os.path.join(output_dir, f"{variant_name}.{export_format}")

        try:
            if export_format == "step":
                opts = em.createSTEPExportOptions(filepath, root)
            elif export_format == "stl":
                opts = em.createSTLExportOptions(root, filepath)
            else:
                opts = em.createSTEPExportOptions(filepath, root)
            em.execute(opts)
            results.append({"row": idx, "file": filepath, "status": "ok"})
        except Exception as e:
            results.append({"row": idx, "file": filepath, "status": f"error: {str(e)[:50]}"})

    return {"table": table_name, "variants_generated": len(results), "output_dir": output_dir, "results": results}

def _validate_parameters(p):
    design = _design()
    rules = p.get("rules", {})  # {param_name: {min: val, max: val, type: "mm"}, ...}

    violations = []
    for pname, constraints in rules.items():
        param = design.allParameters.itemByName(pname)
        if not param:
            violations.append({"parameter": pname, "error": "not found"})
            continue

        val = param.value
        if "min" in constraints and val < float(constraints["min"]):
            violations.append({"parameter": pname, "value": val, "violation": f"below min ({constraints['min']})"})
        if "max" in constraints and val > float(constraints["max"]):
            violations.append({"parameter": pname, "value": val, "violation": f"above max ({constraints['max']})"})

    return {"valid": len(violations) == 0, "violations": violations, "checked": len(rules)}

def _get_parameter_dependents(p):
    design = _design()
    pname = p.get("name", "")
    param = design.allParameters.itemByName(pname)
    if not param:
        raise Exception(f"Parameter '{pname}' not found")

    # Find all parameters that reference this one in their expression
    dependents = []
    for i in range(design.allParameters.count):
        other = design.allParameters.item(i)
        if other.name != pname and pname in other.expression:
            dependents.append({"name": other.name, "expression": other.expression})

    # Find features using this parameter (check timeline)
    features = []
    try:
        tl = design.timeline
        for i in range(tl.count):
            item = tl.item(i)
            entity = item.entity
            if hasattr(entity, 'name'):
                features.append(entity.name)
    except Exception:
        pass

    return {"parameter": pname, "value": param.value, "expression": param.expression,
            "dependent_parameters": dependents, "note": "Feature dependency requires expression parsing"}

def _pipeline_execute(root, p):
    """Execute multiple commands atomically with rollback on failure."""
    steps = p.get("steps", [])

    if not steps:
        raise Exception("No steps provided")
    if len(steps) > 20:
        raise Exception(f"Too many steps ({len(steps)}). Max 20.")

    # Block dangerous commands
    blocked = {"execute_script", "pipeline_execute"}
    for step in steps:
        if step.get("command") in blocked:
            raise Exception(f"Command '{step['command']}' not allowed in pipeline")

    design = _design()
    tl = design.timeline
    start_pos = tl.count

    results = []
    try:
        for i, step in enumerate(steps):
            cmd = step.get("command", "")
            params = step.get("params", {})

            # Build a fake data dict and process it
            step_data = {"command": cmd, "params": params}
            step_result = _process_command(step_data)

            if "error" in step_result:
                raise Exception(f"Step {i} ({cmd}) failed: {step_result['error']}")

            results.append({"step": i, "command": cmd, "result": "ok"})

        return {"pipeline": "completed", "steps_executed": len(results), "results": results}
    except Exception as e:
        # Rollback
        try:
            while tl.count > start_pos:
                item = tl.item(tl.count - 1)
                item.entity.deleteMe()
        except Exception:
            pass
        return {"pipeline": "rolled_back", "error": str(e), "steps_completed": len(results), "results": results}


# ---- Dispatcher ----

def _process_command(data: dict) -> dict:
    try:
        design = _design()
        if not design:
            return {"error": "No active Fusion 360 design. Open or create one first."}
        root = design.rootComponent
        cmd = data.get("command", "")
        p = data.get("params", {})

        dispatch = {
            "get_info":                   lambda: _get_info(design, root),
            "get_bodies_info":            lambda: _get_bodies_info(root),
            "get_face_info":              lambda: _get_face_info(root, p),
            "get_edge_info":              lambda: _get_edge_info(root, p),
            "get_sketch_info":            lambda: _get_sketch_info(root, p),
            "get_timeline_info":          lambda: _get_timeline_info(),
            "measure_body":               lambda: _measure_body(root, p),
            "measure_between":            lambda: _measure_between(root, p),
            "execute_script":             lambda: _execute_script(p),
            "create_new_document":        lambda: _create_new_document(p),
            "clear_design":               lambda: _clear_design(),
            "create_sketch":              lambda: _create_sketch(root, p),
            "create_sketch_on_face":      lambda: _create_sketch_on_face(root, p),
            "finish_sketch":              lambda: _finish_sketch(root, p),
            "delete_sketch":              lambda: _delete_sketch(root, p),
            "draw_rectangle":             lambda: _draw_rectangle(root, p),
            "draw_center_rectangle":      lambda: _draw_center_rectangle(root, p),
            "draw_circle":                lambda: _draw_circle(root, p),
            "draw_line":                  lambda: _draw_line(root, p),
            "draw_arc":                   lambda: _draw_arc(root, p),
            "draw_polygon":               lambda: _draw_polygon(root, p),
            "draw_ellipse":               lambda: _draw_ellipse(root, p),
            "draw_spline":                lambda: _draw_spline(root, p),
            "draw_slot":                  lambda: _draw_slot(root, p),
            "draw_text":                  lambda: _draw_text(root, p),
            "add_sketch_fillet":          lambda: _add_sketch_fillet(root, p),
            "offset_sketch":              lambda: _offset_sketch(root, p),
            "mirror_sketch":              lambda: _mirror_sketch(root, p),
            "rectangular_pattern_sketch": lambda: _rectangular_pattern_sketch(root, p),
            "add_constraint":             lambda: _add_constraint(root, p),
            "add_sketch_dimension":       lambda: _add_sketch_dimension(root, p),
            "extrude":                    lambda: _extrude(root, p),
            "revolve":                    lambda: _revolve(root, p),
            "loft":                       lambda: _loft(root, p),
            "sweep":                      lambda: _sweep(root, p),
            "helix":                      lambda: _helix(root, p),
            "create_pipe":                lambda: _create_pipe(root, p),
            "create_hole":                lambda: _create_hole(root, p),
            "shell":                      lambda: _shell(root, p),
            "fillet":                     lambda: _fillet(root, p),
            "chamfer":                    lambda: _chamfer(root, p),
            "mirror_body":                lambda: _mirror_body(root, p),
            "rectangular_pattern_body":   lambda: _rectangular_pattern_body(root, p),
            "circular_pattern_body":      lambda: _circular_pattern_body(root, p),
            "combine_bodies":             lambda: _combine_bodies(root, p),
            "scale_body":                 lambda: _scale_body(root, p),
            "move_body":                  lambda: _move_body(root, p),
            "rotate_body":               lambda: _rotate_body(root, p),
            "press_pull":                 lambda: _press_pull(root, p),
            "thicken":                    lambda: _thicken(root, p),
            "draft_face":                 lambda: _draft_face(root, p),
            "add_thread":                 lambda: _add_thread(root, p),
            "create_component":           lambda: _create_component(root, p),
            "move_body_to_component":     lambda: _move_body_to_component(root, p),
            "create_joint":               lambda: _create_joint(root, p),
            "create_as_built_joint":      lambda: _create_as_built_joint(root, p),
            "rename_component":           lambda: _rename_component(root, p),
            "delete_occurrence":          lambda: _delete_occurrence(root, p),
            "toggle_component_visibility": lambda: _toggle_component_visibility(root, p),
            "list_occurrences_tree":      lambda: _list_occurrences_tree(root),
            "activate_component":         lambda: _activate_component(root, p),
            "get_component_info":         lambda: _get_component_info(root, p),
            "list_documents":             lambda: _list_documents(),
            "switch_document":            lambda: _switch_document(p),
            "delete_body":                lambda: _delete_body(root, p),
            "rename_body":                lambda: _rename_body(root, p),
            "copy_body":                  lambda: _copy_body(root, p),
            "toggle_body_visibility":     lambda: _toggle_body_visibility(root, p),
            "add_construction_plane":     lambda: _add_construction_plane(root, p),
            "add_construction_axis":      lambda: _add_construction_axis(root, p),
            "add_parameter":              lambda: _add_parameter(design, p),
            "update_parameter":           lambda: _update_parameter(design, p),
            "list_parameters":            lambda: _list_parameters(design),
            "list_appearances":           lambda: _list_appearances(p),
            "apply_appearance":           lambda: _apply_appearance(root, p),
            "set_body_color":             lambda: _set_body_color(root, p),
            "export_stl":                 lambda: _export_stl(p),
            "export_step":                lambda: _export_step(p),
            "export_3mf":                 lambda: _export_3mf(p),
            "export_f3d":                 lambda: _export_f3d(p),
            "capture_screenshot":         lambda: _capture_screenshot(p),
            "undo":                       lambda: _undo(p),
            "redo":                       lambda: _redo(p),
            "save":                       lambda: _save_design(p),
            "save_as":                    lambda: _save_as(p),
            "fusion_status":              lambda: {"status": "ok", "message": "Fusion MCP bridge running"},
            "export_obj":                 lambda: {"error": "OBJ export is not supported by the Fusion 360 API. Use STL, STEP, or 3MF instead."},
            "check_interference":         lambda: _check_interference(root, p),
            "assign_material":            lambda: _assign_material(root, p),
            "get_section_properties":     lambda: _get_section_properties(root, p),
            "get_curvature_info":         lambda: _get_curvature_info(root, p),
            "check_wall_thickness":       lambda: _check_wall_thickness(root, p),
            "check_draft_angles":         lambda: _check_draft_angles(root, p),
            "get_mesh_body_info":         lambda: _get_mesh_body_info(root, p),
            "check_printability":         lambda: _check_printability(root, p),
            "cam_get_setups":             lambda: _cam_get_setups(p),
            "cam_create_setup":           lambda: _cam_create_setup(root, p),
            "cam_create_operation":        lambda: _cam_create_operation(p),
            "cam_generate_toolpath":       lambda: _cam_generate_toolpath(p),
            "cam_list_tools":             lambda: _cam_list_tools(p),
            "cam_post_process":           lambda: _cam_post_process(p),
            "cam_simulate":               lambda: _cam_simulate(p),
            "cam_get_operation_info":      lambda: _cam_get_operation_info(p),
            "edit_feature":               lambda: _edit_feature(p),
            "suppress_feature":           lambda: _suppress_feature(p),
            "delete_feature":             lambda: _delete_feature(p),
            "reorder_feature":            lambda: _reorder_feature(p),
            "auto_constrain":             lambda: _auto_constrain(root, p),
            "get_sketch_health":          lambda: _get_sketch_health(root, p),
            "remove_constraint":          lambda: _remove_constraint(root, p),
            "save_variant":               lambda: _save_variant(p),
            "load_variant":               lambda: _load_variant(p),
            "list_variants":              lambda: _list_variants(p),
            "delete_variant":             lambda: _delete_variant(p),
            "add_parameter_expression":   lambda: _add_parameter_expression(p),
            "batch_update_parameters":    lambda: _batch_update_parameters(p),
            "generate_bom":               lambda: _generate_bom(root, p),
            "export_bom":                 lambda: _export_bom(root, p),
            "generate_drawing":           lambda: _generate_drawing(root, p),
            "create_3d_sketch":           lambda: _create_3d_sketch(root, p),
            "draw_3d_line":               lambda: _draw_3d_line(root, p),
            "draw_3d_spline":             lambda: _draw_3d_spline(root, p),
            "project_to_surface":         lambda: _project_to_surface(root, p),
            "create_hole_pattern_linear":  lambda: _create_hole_pattern_linear(root, p),
            "create_hole_pattern_circular": lambda: _create_hole_pattern_circular(root, p),
            "get_hole_info":              lambda: _get_hole_info(root, p),
            "create_sheet_metal_rule":    lambda: _create_sheet_metal_rule(root, p),
            "sheet_metal_flange":         lambda: _sheet_metal_flange(root, p),
            "sheet_metal_bend":           lambda: _sheet_metal_bend(root, p),
            "sheet_metal_unfold":         lambda: _sheet_metal_unfold(root, p),
            "sheet_metal_flat_pattern":   lambda: _sheet_metal_flat_pattern(root, p),
            "import_mesh":                lambda: _import_mesh(root, p),
            "repair_mesh":                lambda: _repair_mesh(root, p),
            "convert_mesh_to_brep":       lambda: _convert_mesh_to_brep(root, p),
            # Advanced Joints
            "create_joint_from_geometry": lambda: _create_joint_from_geometry(root, p),
            "create_rigid_group":         lambda: _create_rigid_group(root, p),
            "set_joint_limits":           lambda: _set_joint_limits(root, p),
            "drive_joint":                lambda: _drive_joint(root, p),
            "edit_joint":                 lambda: _edit_joint(root, p),
            "get_joint_info":             lambda: _get_joint_info(root, p),
            "list_joints":                lambda: _list_joints(root, p),
            # Data Management
            "list_data_hubs":             lambda: _list_data_hubs(p),
            "list_data_projects":         lambda: _list_data_projects(p),
            "list_data_folders":          lambda: _list_data_folders(p),
            "open_data_file":             lambda: _open_data_file(p),
            "save_to_folder":             lambda: _save_to_folder(p),
            "list_data_versions":         lambda: _list_data_versions(p),
            "open_data_version":          lambda: _open_data_version(p),
            # Events/Callbacks
            "subscribe_design_events":    lambda: _subscribe_design_events(p),
            "poll_design_events":         lambda: _poll_design_events(p),
            "unsubscribe_design_events":  lambda: _unsubscribe_design_events(p),
            "get_subscription_status":    lambda: _get_subscription_status(p),
            # Render
            "set_render_environment":     lambda: _set_render_environment(p),
            "set_appearance_detailed":    lambda: _set_appearance_detailed(root, p),
            "create_render_camera":       lambda: _create_render_camera(p),
            "apply_render_camera":        lambda: _apply_render_camera(p),
            # Drawing Generation
            "create_drawing_document":    lambda: _create_drawing_document(p),
            "add_base_view":              lambda: _add_base_view(p),
            "add_projected_view":         lambda: _add_projected_view(p),
            "add_section_view":           lambda: _add_section_view(p),
            "add_drawing_dimension":      lambda: _add_drawing_dimension(p),
            "add_annotation":             lambda: _add_annotation(p),
            "export_drawing":             lambda: _export_drawing(p),
            # Parametric Automation Phase 2-3
            "compare_variants":           lambda: _compare_variants(p),
            "create_design_table":        lambda: _create_design_table(p),
            "apply_design_table_row":     lambda: _apply_design_table_row(p),
            "generate_variant_batch":     lambda: _generate_variant_batch(root, p),
            "validate_parameters":        lambda: _validate_parameters(p),
            "get_parameter_dependents":   lambda: _get_parameter_dependents(p),
            "pipeline_execute":           lambda: _pipeline_execute(root, p),
        }

        if cmd in dispatch:
            if cmd in SKIP_GROUPING:
                return dispatch[cmd]()
            with _timeline_group(cmd):
                return dispatch[cmd]()
        return {"error": f"Unknown command '{cmd}'",
                "available_commands": sorted(dispatch.keys())}
    except Exception as e:
        # Log the full traceback internally but only return a summary to the client
        traceback.print_exc()
        return {"error": str(e)}


# ---- Event Handler ----

class MCPEventHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        global _processing
        if _processing:
            return
        _processing = True
        try:
            while True:
                try:
                    data = _cmd_queue.get_nowait()
                except queue.Empty:
                    break
                cmd_id = data.get("id")
                result = _process_command(data)
                with _lock:
                    _results[cmd_id] = result
                    if cmd_id in _result_events:
                        _result_events[cmd_id].set()
        except Exception:
            import traceback
            traceback.print_exc()
        finally:
            _processing = False


# ---- Add-in Lifecycle ----

def run(context):
    global app, ui, _server, _server_thread, _auth_token
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _auth_token = os.environ.get("FUSION_MCP_TOKEN", None)
        custom_event = app.registerCustomEvent(CUSTOM_EVENT_ID)
        handler = MCPEventHandler()
        custom_event.add(handler)
        _handlers.append(handler)
        _server = ThreadingHTTPServer(("127.0.0.1", PORT), MCPRequestHandler)
        _server_thread = threading.Thread(target=_server.serve_forever, daemon=True)
        _server_thread.start()
        ui.messageBox(
            "FusionMCP bridge is running on port 7432.\n\n"
            "You can now connect any MCP client.",
            "FusionMCP")
    except Exception:
        if ui:
            ui.messageBox(f"FusionMCP failed to start:\n{traceback.format_exc()}")


def stop(context):
    global _server, _server_thread
    try:
        if _server:
            _server.shutdown()
        if _server_thread:
            _server_thread.join(timeout=5)
            _server_thread = None
        _server = None
        app.unregisterCustomEvent(CUSTOM_EVENT_ID)
        _handlers.clear()
        with _lock:
            _results.clear()
            _result_events.clear()
    except Exception:
        pass
