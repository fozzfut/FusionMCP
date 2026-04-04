# FusionMCP

MCP (Model Context Protocol) bridge for Autodesk Fusion 360. Runs as a Fusion 360 add-in, exposing an HTTP API on `localhost:7432` that MCP clients (Claude Code, etc.) can use to control Fusion 360 programmatically.

## Features

- **Sketching**: rectangles, circles, lines, arcs, polygons, ellipses, splines, slots, text, constraints, dimensions
- **3D Modeling**: extrude, revolve, loft, sweep, helix, pipe, hole, shell, fillet, chamfer, draft, thread
- **Body Operations**: move, rotate, scale, mirror, copy, combine, pattern (rectangular/circular)
- **Components & Assembly**: create components, joints, as-built joints
- **Parameters**: create, update, list user/model parameters
- **Appearance**: list/apply appearances, set body color
- **Export**: STL, STEP, 3MF, F3D, screenshots
- **Scripting**: execute arbitrary Python code inside Fusion 360

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

## API

All commands are sent via POST to `http://localhost:7432` with a JSON body:

```json
{
  "command": "command_name",
  "parameters": { ... }
}
```

### Available Commands

See the source code for the full list of ~70 commands. Key categories:

| Category | Commands |
|---|---|
| Info | `get_info`, `get_bodies_info`, `get_face_info`, `get_edge_info`, `get_sketch_info`, `get_timeline_info` |
| Sketch | `create_sketch`, `draw_*`, `add_constraint`, `add_sketch_dimension` |
| 3D | `extrude`, `revolve`, `loft`, `sweep`, `shell`, `fillet`, `chamfer`, ... |
| Bodies | `delete_body`, `rename_body`, `copy_body`, `move_body`, `combine_bodies` |
| Assembly | `create_component`, `create_joint`, `create_as_built_joint` |
| Export | `export_stl`, `export_step`, `export_3mf`, `export_f3d`, `capture_screenshot` |
| Script | `execute_script` - run arbitrary Fusion 360 Python API code |

## Roadmap

- [ ] Component/occurrence management (rename, delete, visibility, activate)
- [ ] Full assembly tree listing with hierarchy
- [ ] Document management (open, close, switch, rename)
- [ ] Improved undo/redo via transactions
- [ ] CAM workspace integration
- [ ] 2D Drawing generation
- [ ] Interference detection & mass properties
- [ ] Sketch trim/extend/project

## License

MIT
