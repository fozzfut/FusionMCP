# FusionMCP Roadmap

## ADR Decisions (swarm-kb debates)

### ADR-ddb6d137: CAM Workspace (5-0 unanimous)
8 commands for full CNC machining pipeline: setup, operations, toolpaths, G-code.

### ADR-9305d6f8: Geometry Analysis & DFM (5-0 unanimous)  
2 extensions + 8 new commands for interference, materials, DFM checks.

### ADR-480b09d5: Parametric Design Automation (5-0 unanimous)
29 commands in 3 phases: feature editing, variants, BOM, drawings.

## Implementation Status

### Wave 1: Analysis & DFM (~10 commands)
- [ ] Extend `measure_body` with physical properties
- [ ] Extend `get_face_info` with draft angles
- [ ] `check_interference`
- [ ] `assign_material`
- [ ] `get_section_properties`
- [ ] `get_curvature_info`
- [ ] `check_wall_thickness`
- [ ] `check_draft_angles`
- [ ] `get_mesh_body_info`
- [ ] `check_printability`

### Wave 2: CAM (~8 commands)
- [ ] `cam_get_setups`
- [ ] `cam_create_setup`
- [ ] `cam_create_operation`
- [ ] `cam_generate_toolpath`
- [ ] `cam_list_tools`
- [ ] `cam_post_process`
- [ ] `cam_simulate`
- [ ] `cam_get_operation_info`

### Wave 3: Parametric Phase 1 (~16 commands)
- [ ] `edit_feature`
- [ ] `suppress_feature`
- [ ] `delete_feature`
- [ ] `reorder_feature`
- [ ] `auto_constrain`
- [ ] `get_sketch_health`
- [ ] `remove_constraint`
- [ ] `save_variant` / `load_variant` / `list_variants` / `delete_variant`
- [ ] `add_parameter_expression`
- [ ] `batch_update_parameters`
- [ ] `generate_bom`
- [ ] `export_bom`
- [ ] `generate_drawing`

### Wave 4: Parametric Phase 2-3 (future)
- [ ] Design tables, variant comparison, drawing details
- [ ] Cost rollup, parameter validation, pipeline execute
