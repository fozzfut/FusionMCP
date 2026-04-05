# FusionMCP

MCP (Model Context Protocol) bridge for Autodesk Fusion 360. Runs as a Fusion 360 add-in, exposing an HTTP API on `localhost:7432` that MCP clients (Claude Code, etc.) can use to control Fusion 360 programmatically.

## Features

- **Sketching**: rectangles, circles, lines, arcs, polygons, ellipses, splines, slots, text, constraints, dimensions
- **3D Modeling**: extrude, revolve, loft, sweep, helix, pipe, hole, shell, fillet, chamfer, draft, thread
- **Body Operations**: move, rotate, scale, mirror, copy, combine, pattern (rectangular/circular)
- **Components & Assembly**: rename, delete, toggle visibility, activate (edit-in-place), list occurrence tree, create joints
- **Document Management**: list open documents, switch between documents
- **Parameters**: create, update, list user/model parameters
- **Appearance**: list/apply appearances, set body color
- **Export**: STL, STEP, 3MF, F3D, screenshots
- **Undo/Redo**: timeline-based undo/redo with automatic operation grouping
- **Scripting**: execute arbitrary Python code inside Fusion 360 (optional auth via `FUSION_MCP_TOKEN`)

## Installation

1. Copy the `FusionMCP` folder to your Fusion 360 add-ins directory:
   - **Windows**: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\`
   - **macOS**: `~/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/`
2. In Fusion 360, go to **Tools > Add-Ins > Scripts and Add-Ins**
3. Enable **FusionMCP** and check "Run on Startup"
4. Configure your MCP client to connect to `http://localhost:7432`

## MCP Client Configuration

Add to your MCP client config (e.g., Claude Code `settings.json`):

```json
{
  "mcpServers": {
    "fusion360": {
      "url": "http://localhost:7432"
    }
  }
}
```

## Security

The server listens only on `127.0.0.1` (localhost). For additional security on `execute_script`, set the `FUSION_MCP_TOKEN` environment variable before starting Fusion 360. When set, all `execute_script` calls must include a matching `token` parameter.

## API

All commands are sent via POST to `http://localhost:7432` with a JSON body:

```json
{
  "command": "command_name",
  "params": { ... }
}
```

### Available Commands (~80)

| Category | Commands |
|---|---|
| Info | `fusion_status`, `get_info`, `get_bodies_info`, `get_face_info`, `get_edge_info`, `get_sketch_info`, `get_timeline_info`, `measure_body`, `measure_between` |
| Sketch | `create_sketch`, `create_sketch_on_face`, `finish_sketch`, `delete_sketch`, `draw_rectangle`, `draw_center_rectangle`, `draw_circle`, `draw_line`, `draw_arc`, `draw_polygon`, `draw_ellipse`, `draw_spline`, `draw_slot`, `draw_text`, `add_sketch_fillet`, `offset_sketch`, `mirror_sketch`, `rectangular_pattern_sketch`, `add_constraint`, `add_sketch_dimension` |
| 3D Features | `extrude`, `revolve`, `loft`, `sweep`, `helix`, `create_pipe`, `create_hole`, `shell`, `fillet`, `chamfer`, `press_pull`, `thicken`, `draft_face`, `add_thread` |
| Bodies | `delete_body`, `rename_body`, `copy_body`, `move_body`, `rotate_body`, `scale_body`, `mirror_body`, `combine_bodies`, `toggle_body_visibility`, `rectangular_pattern_body`, `circular_pattern_body` |
| Components | `rename_component`, `delete_occurrence`, `toggle_component_visibility`, `list_occurrences_tree`, `activate_component`, `get_component_info`, `create_component`, `move_body_to_component` |
| Assembly | `create_joint`, `create_as_built_joint` |
| Documents | `list_documents`, `switch_document`, `create_new_document`, `clear_design`, `save`, `save_as` |
| Parameters | `add_parameter`, `update_parameter`, `list_parameters` |
| Appearance | `list_appearances`, `apply_appearance`, `set_body_color` |
| Construction | `add_construction_plane`, `add_construction_axis` |
| Export | `export_stl`, `export_step`, `export_3mf`, `export_f3d`, `capture_screenshot` |
| History | `undo`, `redo` |
| Script | `execute_script` |

### Transaction Management

All mutating commands are automatically wrapped in timeline groups for atomic undo/redo. Read-only commands (queries, listings) skip grouping for performance. The `undo` command uses `timeline.markerPosition` for reliable rollback.

### Timeouts

- Export and script commands: 120 seconds
- All other commands: 30 seconds

## Architecture

```
Claude Code / MCP Client
    |
    | HTTP POST (JSON)
    v
ThreadingHTTPServer (:7432)         [worker thread]
    |
    | queue + custom event
    v
MCPEventHandler.notify()            [Fusion main thread]
    |
    | _process_command() -> dispatch table
    v
Fusion 360 API (adsk.fusion)
```

Key design decisions (documented as ADRs in swarm-kb):
- **ADR-e7d6450a**: Flat commands over generic CRUD for component management
- **ADR-a9033d5e**: Context manager over manual begin/commit for transactions
- **ADR-03e47f13**: Three-phase pragmatic refactoring over immediate module split

## Roadmap

- [x] Component/occurrence management (rename, delete, visibility, activate)
- [x] Full assembly tree listing with hierarchy
- [x] Document management (list, switch)
- [x] Improved undo/redo via timeline markers
- [x] Automatic transaction grouping
- [x] Thread safety (locks, ThreadingHTTPServer)
- [ ] Path-based addressing (`Component/SubComp/Body` syntax)
- [ ] CAM workspace integration
- [ ] 2D Drawing generation
- [ ] Interference detection & mass properties
- [ ] Sketch trim/extend/project
- [ ] Batch operations

## License

MIT
