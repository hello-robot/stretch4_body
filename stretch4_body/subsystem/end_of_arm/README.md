# End-of-arm tools

A tool is **a directory containing a `tool_params.yaml`**. That is true whether the tool ships with
the robot or you wrote it yourself — the same loader (`eoa_tool_config.py`) finds both, in the same
format, and registers them the same way.

Built-in tools live here, one directory per tool:

```
subsystem/end_of_arm/
    eoa_tool_config.py                  discovery + loading, for built-in and user tools alike
    end_of_arm.py                       the EndOfArm chain base class
    end_of_arm_tools.py                 the built-in EndOfArm subclasses
    wrist_pitch.py wrist_roll.py wrist_yaw.py   drivers shared by every DW4 tool
    stretch_gripper.py parallel_gripper.py      the two built-in gripper drivers
    eoa_wrist_dw4_tool_nil/tool_params.yaml
    eoa_wrist_dw4_tool_sg4/tool_params.yaml
    ...
```

Your own tools go in the fleet's `user_tools` directory instead — `$HELLO_FLEET_PATH/user_tools/`,
or `~/stretch_user/user_tools/` when that variable is unset. `stretch_add_user_tool` (from the
`stretch4_urdf` package) scaffolds one for you; `stretch4_urdf/SE4_tools/user_tool.md` is the
step-by-step guide.

**A tool's kinematics are not here.** The URDF, meshes and `collision_mesh_config.yaml` live in the
`stretch4_urdf` package, at `stretch4_urdf/SE4_tools/<tool_name>/`, under the same tool name. A tool
is one directory in each of the two packages: shape over there, behavior over here.

## Reading the examples

Start with **`eoa_wrist_dw4_tool_pg4`** or **`eoa_wrist_dw4_tool_sg4`**. Both are written out in
full, so you can copy one into your own tool directory and edit it top to bottom. PG4 additionally
shows `collision_mgmt`, which a tool that hangs below the wrist needs so the lift brakes before the
tool reaches the base.

**`eoa_wrist_dw4_tool_tablet`** is the contrast: a passive tool with no actuator, which inherits the
bare-wrist baseline and states only its deltas. That is what your tool gets by default.

## The format

| Key | Meaning |
|---|---|
| `inherits` | Tool name to build on. **Omit it** and you get `eoa_wrist_dw4_tool_nil` — the 3-DOF wrist, its stow pose and its three device entries. `null` opts out entirely. |
| `metadata` | `name` and `description` for the `stretch_configure_tool` picker, plus an optional `sort_key` (lower sorts first; the default puts you after the built-ins). Never inherited. |
| `py_class_name` / `py_module_name` | Your `EndOfArm` subclass. Built-ins use a dotted module path because they live in the package; **your tool uses a bare name** like `end_of_arm`, resolved against your own tool directory. |
| `client_class_name` / `client_module_name` | Optional `EndOfArmClient` subclass, same bare-name rule. |
| `metadata_class_name` / `metadata_module_name` | Optional `ToolMetadata` subclass, if the generic unit conversions do not fit your hardware. |
| `stow` | Per-joint stow targets, including your tool's own joint. |
| `homing` | Optional per-joint positions held during homing. |
| `devices` | One entry per servo on the wrist bus. **Key order sets the order motors are added to the Feetech chain.** `device_params` names a dict in `robot_params_SE4.py` (e.g. `SE4_wrist_pitch_DW4`); a custom servo omits it and inlines its motor parameters instead. |
| `collision_mgmt` | Brake distances and collision pairs against the robot body. |
| `self_collision_mujoco` | `exclusions` are link pairs that touch by design and must not be reported as self-collisions. Link names must match your URDF. |
| `ros` | Extra ROS command groups this tool contributes. Omit if it has none. |

A user tool of the same name as a built-in **shadows** it, and the loader says so on stdout.

### Heads Up

* If your tool defines a custom `ToolMetadata` subclass, your driver and your metadata module will reference each other: the driver
uses the metadata class to convert command units to actuator radians, and the metadata imports the driver class as for its property. To 
avoid circularity, import the driver module from inside the `driver_class` property of the ToolMetadata subclass.

* ROS params will reject a new value if it differs from the existing type. Be mindful of integer vs. float values (`0` and `0.0` are not interchangeable). If the new value is rejected, the parameter will stay set to the old value. 

* PyYAML reads bare `on`, `off`, `yes`, `no` as booleans. Quote them if you want them as strings.

* `1e6` parses as a string, not a number. Write `1.0e+6` for the float equivalent.

