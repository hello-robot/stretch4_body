# Adding a Custom End-of-Arm Tool

A step-by-step guide for adding a new custom tool For further details, see
[Custom User End-of-Arm Tools](../README.md#custom-user-end-of-arm-tools) in the top-level
README.

## 1. Create a Folder for Your Tool

Custom tools should be placed in your fleet's `user_tools` directory, `<HELLO_FLEET_PATH>/user_tools/`

Option 1 (Recommended):

- Clone the [stretch4_tool_share](https://github.com/hello-robot/stretch4_tool_share) repository into your
robot's `repos` folder (or another local directory).

  ```bash
  git clone https://github.com/hello-robot/stretch4_tool_share.git ~/repos/stretch4_tool_share
  ```

- Add a new directory inside the repository for your tool. The name will be your tool's name.

  ```bash
  cd ~/repos/stretch4_tool_share
  mkdir my_new_tool
  ```

- Create a symlink for your tool into your robot's user_tools directory

  ```bash
  ln -s ~/repos/stretch4_tool_share/my_new_tool ~/stretch_user/user_tools/my_new_tool
  ```


Option 2:

- Navigate to your robot's `user_tools` directory, and create a new directory for your tool

  ```bash
  cd ~/stretch_user/user_tools
  mkdir my_new_tool
  ```

## 2. Copy in the URDF and Mesh Files

Copy your tool's URDF file into your tool's directory, and create a new mesh directory with all STL files used to visualize your tool.

### URDF

Your URDF's root link must be named `quick_connect_interface_link`. If your CAD export names the
root something else, add `quick_connect_interface_link` as a new root and tie it to your old root
with a fixed identity joint:

```xml
<link name="quick_connect_interface_link"/>

<joint name="quick_connect_interface_joint" type="fixed">
  <parent link="quick_connect_interface_link"/>
  <child link="my_old_root_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### Collision meshes

If you don't already have collision meshes, `process_new_user_tool` can generate collision meshes
automatically from the original mesh files. It's a dev/preprocessing tool, not an installed
command, so it's run as a module:

```bash
python3 -m stretch4_urdf.utils.preprocessing.process_new_user_tool <path-to-your-tool-directory>
```

This writes a `collision_mesh_config.yaml` covering every visual mesh link, generates each
collision mesh next to its visual mesh, and rewrites your URDF's `<collision>` tags and mesh
paths to match. Different decimation ratios or shapes can be specified in
`collision_mesh_config.yaml`, next to your URDF. Otherwise, it will default to 90% reduction.

## 3. Controlling Your New Tool

A tool is made of three independently configured pieces, each optional depending on your tool. Work out which ones your tool actually needs before writing any custom code:

**Do you need a custom driver?**
- Yes → your tool has its own actuator/motor(s) on the wrist bus. Continue below.
- No → your tool is passive. Skip the rest of this step, and go to step 4.

**Do you need custom metadata?**
- Yes → your motor's motion relates to the gripper's physical motion through a linkage or other nonlinear transmission that a single linear scale can't describe. Write a `ToolMetadata` subclass (Path B, below).
- No → your tool has a linear relationship between the motor's motion and the tool's motion. Linear motion is handled by default through YAML keys alone (Path A, below) — no custom metadata needed.

**Do you need a custom client?**
- Yes → your tool has more than a single degree of freedom, and the built-in `move_to()`/`move_by()`/`pose()` and status reads are not sufficient.
- No → your tool has a single degree of freedom (like stretch's two built in grippers).

#### Driver:

Write a Python driver class for your tool's actuator(s). Hello Robot already has a custom `FeetechSMHello` superclass to use with Feetech motors. Add your driver file and class name to `tool_params.yaml`:

```yaml
driver_module_name: my_new_tool_driver
driver_class_name: MyNewToolDriver
```

#### Metadata:

- **Path A — linear tools.** Send motor commands directly, with no gearbox nonlinearity or
  linkage, by adding these keys to `tool_params.yaml`:

  ```yaml
  tool_joints: ['my_finger_left_joint', 'my_finger_right_joint']  # URDF joint names
  tool_links: ['my_finger_left_link', 'my_finger_right_link']     # URDF link names
  primary_joint: 'my_finger_left_joint'   # optional, defaults to the first tool_joints entry

  actuator_command_range: [0.0, 100.0]    # (min, max) in move_to()/move_by()'s own units
  aperture_range: [0.0, 0.08]             # physical fingertip opening bounds (meters)

  urdf_to_actuator_scale: 100.0           # command = urdf * this; optional, defaults to 1.0
  position_tolerance: 0.002               # "arrived" threshold (URDF units); optional, defaults to 2% of range
  ```

- **Path B — nonlinear tools.** If your motor's motion relates to the gripper's physical motion
  through a linkage or other nonlinear transmission and a single linear scale can't describe it,
  write your own `ToolMetadata` subclass and register it in `tool_params.yaml`:

  ```yaml
  metadata_module_name: my_new_tool_metadata
  metadata_class_name: MyNewToolMetadata
  ```

  ```python
  from stretch4_body.utils.tool_metadata import ToolMetadata

  class MyNewToolMetadata(ToolMetadata):
      ...  # tool_name, tool_joints, tool_links, client_class, driver_class

      @property
      def actuator_range(self) -> tuple[float, float]:
          """(min, max) servo angle (radians)."""

      @property
      def command_range(self) -> tuple[float, float]:
          """(min, max) in whatever units your move_to()/move_by() actually accept."""

      def urdf_to_command(self, urdf: float) -> float:
          """URDF joint value -> your move_to()/move_by()'s own units."""

      def command_to_urdf(self, command: float) -> float:
          """Your move_to()/move_by()'s own units -> URDF joint value."""

      def command_to_actuator(self, command: float) -> float:
          """Your move_to()/move_by()'s own units -> servo angle (radians)."""

      def actuator_to_command(self, actuator: float) -> float:
          """Servo angle (radians) -> your move_to()/move_by()'s own units."""

      def aperture_to_actuator(self, aperture: float) -> float:
          """Physical fingertip opening (meters) -> servo angle (radians)."""

      def actuator_to_aperture(self, actuator: float) -> float:
          """Servo angle (radians) -> physical fingertip opening (meters)."""

      def status_to_metadata(self, status: dict) -> dict:
          """Raw hardware status -> {'aperture_m', 'finger_rad', 'finger_effort', 'finger_vel'}."""
  ```

  See `ParallelGripperMetadata` and `StretchGripperMetadata` in `stretch4_body/utils/tool_metadata.py`
  (in the [stretch4_body](https://github.com/hello-robot/stretch4_body) repository) for complete
  worked examples.

#### Client:

Write a Python client class for your tool, subclassing `WristJointClient`. The generic
`ToolJointClient` already covers a single degree of freedom tool, using the poses and conversions
defined in the metadata — write your own only if your tool needs more than
`move_to()`/`move_by()`/`pose()` and status reads. Add your client file and class name to
`tool_params.yaml`:

```yaml
client_module_name: my_new_tool_client
client_class_name: MyNewToolClient
```

#### Tool Parameters:

Add the remaining unique attributes of your tool:

```yaml
id: 24                              # unique id on the wrist bus
range_deg: [0, 187]                 # mechanical travel
homing_to_neg_limit: 1
homing_pwm: -80                     # sign sets the homing direction
flip_encoder_polarity: 1
```

## 4. Configure Optional Behavior

As needed, add to `tool_params.yaml`:

Each tool can define a set of custom poses for the roll, pitch, yaw, and any tool joints, including the robot's stow and home poses:
- `stow` — per-joint stow targets
- `homing` — where the wrist joints are left after homing
- `pose_models` — named joint poses.

```yaml
stow:
  wrist_pitch: 0.0            # radians; unspecified joints default to 0
  wrist_roll: 0.0
  wrist_yaw: 3.14
  my_finger_left_joint: 0.0   # command units; unspecified joints default to 0

homing:
  wrist_pitch: 0.0            # actuator units; unspecified joints default to 0
  wrist_roll: -0.4            # hold at -0.4 so the pitch joint can home to its hardstop without colliding
  wrist_yaw: 0.0
  my_finger_left_joint: 0.0

pose_models:
  - name: tool_pose
    timestamp: 0.0
    joints:
      wrist_yaw:
        position: 3.14          # radians, same as stow above
        velocity: 0.0
        effort: 0.0
      my_finger_left_joint:
        position: 0.90          # command units, same as tool_joints/stow above
        velocity: 0.0
        effort: 0.0
      my_finger_right_joint:
        position: 0.90
        velocity: 0.0
        effort: 0.0
```

Tools may also have custom collision configurations:
- `collision_mgmt` — brake distances/pairs against the robot body, for a tool that can reach it.
- `self_collision_mujoco` — exclusions for tool link pairs that are expected to touch.

```yaml
collision_mgmt:
  k_brake_distance:
    wrist_pitch: 0.25   # multiplies this joint's required braking distance
  collision_pairs:
    my_finger_left_link_TO_base_link:
      link_pts: my_finger_left_link
      link_cube: base_link
      detect_as: pts
  joints:
    lift:
      - motion_dir: neg
        collision_pair: my_finger_left_link_TO_base_link

self_collision_mujoco:
  exclusions:
    - [my_finger_left_link, my_finger_right_link]   # allowed to touch/overlap
  ignore_links: [my_camera_link]                    # skipped by the collision engine entirely
  k_brake_distance:
    wrist_pitch: 1.1                                 # proportionally increased braking distance
```

## 5. Validation

#### Config Check:

`stretch_check_user_tool` confirms that the directory is set-up properly and `tool_params.yaml` provides all required information:

```bash
stretch_check_user_tool my_new_tool
```

#### Switch to the Tool:

`stretch_configure_tool` is the configuration tool that switches the robot to a custom tool,
offered as an entry in its menu:

```bash
stretch_configure_tool
```

Your tool appears in the numbered list alongside the built-ins, under a display name derived from
its directory name (`my_new_tool` shows as "My New Tool").

#### Test the tool:

1. **Gamepad teleop.** Start `stretch_body_server`, then:

   ```bash
   stretch_gamepad_teleop
   ```

   In Joint Control mode, the `A` and `B` buttons close and open your tool's primary actuated
   joint — this goes through the generic `gripper` alias, so it works the same way regardless of
   which tool is attached.

2. **Collision visualization.** With the server still running:

   ```bash
   stretch_collision_viz
   ```

   This renders the MuJoCo self-collision model, including your new tool's links. A link
   currently in collision is highlighted orange. Move the tool through its range and confirm:
   pairs you listed in `self_collision_mujoco`'s `exclusions` never turn orange (they're allowed
   to touch), and no other pair of your tool's links turns orange unexpectedly.

3. **ROS2 driver.** Launch the driver:

   ```bash
   ros2 launch stretch_core stretch_driver.launch.py
   ```

   Echo the robot joint states, and see that your tool joints are included:

   ```bash
   ros2 topic echo /joint_states
   ```

   Then command it directly through `follow_joint_trajectory`, using the generic `gripper_joint`
   name rather than your tool's own joint name (the driver will accept either):

   ```bash
   ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
     "{trajectory: {joint_names: [gripper_joint], points: [{positions: [<pos>], time_from_start: {sec: 2}}]}}"
   ```

   Pick `<pos>` inside your tool's URDF joint range (the `urdf` unit from
   [Configuring Unit Conversions](../README.md#2-configuring-unit-conversions) in the README), ROS commands use URDF units.
