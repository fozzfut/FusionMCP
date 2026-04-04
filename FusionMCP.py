"""
FusionMCP Add-in

Runs inside Autodesk Fusion 360 as an add-in. Starts an HTTP server on
localhost:7432 that receives commands from the MCP server and dispatches
them to the Fusion 360 API on the main thread via custom events.
"""

import adsk.core
import adsk.fusion
import traceback
import threading
import json
import queue
import uuid
import os
import math
from http.server import HTTPServer, BaseHTTPRequestHandler

app: adsk.core.Application = None
ui: adsk.core.UserInterface = None
_server: HTTPServer = None
_server_thread: threading.Thread = None
_handlers = []
_cmd_queue: queue.Queue = queue.Queue()
_results: dict = {}
_result_events: dict = {}

CUSTOM_EVENT_ID = "FusionMCPTickV4"
PORT = 7432


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
        event = threading.Event()
        _result_events[cmd_id] = event
        _cmd_queue.put(data)
        if app:
            app.fireCustomEvent(CUSTOM_EVENT_ID, cmd_id)
        event.wait(timeout=30)
        result = _results.pop(cmd_id, {"error": "Timeout - Fusion did not respond in 30s"})
        _result_events.pop(cmd_id, None)
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
    except:
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
    faces = []
    for i in range(body.faces.count):
        f = body.faces.item(i)
        geo_type = type(f.geometry).__name__
        try:
            n = f.geometry.normal
            normal = [round(n.x, 3), round(n.y, 3), round(n.z, 3)]
        except:
            normal = None
        faces.append({"index": i, "area_cm2": round(f.area, 4),
                       "type": geo_type, "normal": normal})
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
    except:
        pass
    return {
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
    except Exception:
        return {"error": traceback.format_exc()}


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
        except:
            pass
    while root.sketches.count > 0:
        try:
            root.sketches.item(0).deleteMe()
        except:
            break
    while root.constructionPlanes.count > 0:
        try:
            root.constructionPlanes.item(0).deleteMe()
        except:
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
            except:
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
            except:
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
            except:
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
                except:
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
    count = int(p.get("steps", 1))
    for _ in range(count):
        app.executeTextCommand("Commands.Undo")
    return {"undone": count}

def _redo(p):
    count = int(p.get("steps", 1))
    for _ in range(count):
        app.executeTextCommand("Commands.Redo")
    return {"redone": count}

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
            "export_obj":                 lambda: {"error": "OBJ export is not supported by the Fusion 360 API. Use STL, STEP, or 3MF instead."},
        }

        if cmd in dispatch:
            return dispatch[cmd]()
        return {"error": f"Unknown command '{cmd}'",
                "available_commands": sorted(dispatch.keys())}
    except Exception:
        return {"error": traceback.format_exc()}


# ---- Event Handler ----

class MCPEventHandler(adsk.core.CustomEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            while not _cmd_queue.empty():
                data = _cmd_queue.get_nowait()
                cmd_id = data.get("id")
                result = _process_command(data)
                _results[cmd_id] = result
                if cmd_id in _result_events:
                    _result_events[cmd_id].set()
        except Exception:
            pass


# ---- Add-in Lifecycle ----

def run(context):
    global app, ui, _server, _server_thread
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        custom_event = app.registerCustomEvent(CUSTOM_EVENT_ID)
        handler = MCPEventHandler()
        custom_event.add(handler)
        _handlers.append(handler)
        _server = HTTPServer(("127.0.0.1", PORT), MCPRequestHandler)
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
    global _server
    try:
        if _server:
            _server.shutdown()
        app.unregisterCustomEvent(CUSTOM_EVENT_ID)
        for h in _handlers:
            del h
        _handlers.clear()
    except Exception:
        pass
