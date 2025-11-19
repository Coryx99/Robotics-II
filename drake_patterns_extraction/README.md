# Drake Simulation Pattern Extraction

**Complete pattern analysis of the Robotics-II-fork repository for LLM training**

---

## 📋 Executive Summary

This repository contains a comprehensive extraction of patterns, templates, and best practices from a Drake robotics simulation codebase. The extracted patterns are designed to serve as training data for Large Language Models (LLMs) to generate complete Drake simulations from natural language descriptions.

### Key Statistics

- **40 URDF files** analyzed (17 serial manipulators, 5 grippers, 2 humanoids, 2 mobile bases, 1 quadruped)
- **43 SDF files** analyzed (11 objects, 21 world/scene files)
- **6 Python tutorial scripts** analyzed (3 with IK, 4 with controllers, 2 with trajectories)
- **227 joints** catalogued (216 revolute, 11 prismatic)
- **809 collision geometries** analyzed (631 spheres, 65 cylinders, 63 boxes, 50 meshes)

---

## 📁 Repository Structure

```
drake_patterns_extraction/
├── analysis/
│   ├── 00_repository_structure.json          # Complete repository structure
│   └── repository_statistics.json            # Statistical summary
│
├── patterns/
│   ├── urdf_patterns/
│   │   ├── robot_type_templates.json         # Templates by robot type
│   │   ├── inertia_templates.json            # Inertia calculation formulas
│   │   └── extracted_patterns.json           # Raw extracted URDF patterns
│   │
│   ├── sdf_patterns/
│   │   ├── object_templates.json             # SDF object templates
│   │   └── extracted_patterns.json           # Raw extracted SDF patterns
│   │
│   └── drake_code_patterns/
│       ├── templates/
│       │   ├── 01_minimal_simulation.py      # Basic robot visualization
│       │   ├── 02_robot_with_pid_controller.py  # PD+G control
│       │   └── 03_inverse_kinematics.py      # IK for end-effector positioning
│       └── extracted_patterns.json           # Raw Python pattern analysis
│
├── validation/
│   ├── urdf_validation_rules.json            # URDF validation constraints
│   └── drake_code_validation_rules.json      # Python code best practices
│
├── examples/
│   └── complete/
│       └── 01_pick_and_place/
│           └── example.json                  # Complete example specification
│
└── README.md                                 # This file
```

---

## 🎯 Purpose and Use Cases

### For LLM Training

This dataset enables LLMs to:

1. **Generate URDF files** from descriptions like "Create a 6-DOF robot arm with a gripper"
2. **Create SDF scenes** from prompts like "Add a table with three boxes on it"
3. **Write Drake simulation code** from requirements like "Simulate a robot picking up an object"
4. **Validate generated code** using extracted rules and patterns
5. **Debug common errors** using catalogued error patterns

### For Robotics Engineers

- **Reference templates** for common robot types and objects
- **Validation checklists** to verify URDF/SDF correctness
- **Code patterns** for controllers, IK, and simulation setup
- **Troubleshooting guide** for common Drake issues

---

## 📊 Key Findings

### Robot Type Patterns

#### Serial Manipulators (17 examples)
- **Typical DOF**: 6-7
- **Examples**: Panda, IIWA, Jaco, Kinova Gen3, UR3e
- **Naming**: `{robot}_link{n}`, `{robot}_joint{n}`
- **Joint limits**:
  - Base joints: ±2.9 rad, 87-320 Nm effort
  - Wrist joints: ±3.0 rad, 12-40 Nm effort
- **Collision**: Sphere-based (3-14 spheres/link) or cylinder-based

#### Grippers & Hands (5 examples)
- **Types**: Parallel jaw (2 DOF) or dexterous hands (16 DOF)
- **Joint types**: Prismatic (0-0.08m stroke) or revolute with mimic
- **Typical force**: 20-100 N for parallel jaws

#### Mobile Bases (2 examples)
- **DOF**: 3-4 (x, y, θ for differential drive)
- **Mass**: 20-100 kg
- **Wheel radius**: 0.1-0.2 m

### Inertia Patterns

**Box**: `I_xx = m(h² + d²)/12`
**Cylinder**: `I_xx = I_yy = m(3r² + h²)/12, I_zz = mr²/2`
**Sphere**: `I_xx = I_yy = I_zz = (2/5)mr²`

**Validation Rules**:
- Mass > 0
- I_xx, I_yy, I_zz > 0
- Triangle inequality: `I_xx + I_yy ≥ I_zz` (and cyclic)

### Drake Code Patterns

**Minimal Simulation** (7 steps):
```python
1. meshcat = StartMeshcat()
2. builder = DiagramBuilder()
3. plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
4. parser.AddModelsFromUrl(...)
5. plant.WeldFrames(...)
6. plant.Finalize()  # CRITICAL
7. diagram = builder.Build()
```

**Common Controllers**:
- **PD+G**: `τ = Kp(q_d - q) - Kd·q̇ + g(q)`
  - Typical Kp: 10-300 (higher for base joints)
  - Typical Kd: 2-10 (√Kp rule of thumb)
- **Inverse Kinematics**: Position + orientation constraints with quadratic cost

---

## 🔍 Pattern Templates

### URDF Serial Manipulator Template

```xml
<robot xmlns:drake="http://drake.mit.edu" name="{{ROBOT_NAME}}">
  <!-- Base link -->
  <link name="{{ROBOT_NAME}}_link0">
    <inertial>
      <mass value="{{MASS}}"/>
      <inertia ixx="{{IXX}}" iyy="{{IYY}}" izz="{{IZZ}}"
               ixy="0" ixz="0" iyz="0"/>
    </inertial>
    <visual>
      <geometry><mesh filename="{{MESH_PATH}}"/></geometry>
    </visual>
    <collision>
      <geometry><sphere radius="{{RADIUS}}"/></geometry>
    </collision>
  </link>

  <!-- Joint template -->
  <joint name="{{ROBOT_NAME}}_joint1" type="revolute">
    <parent link="{{ROBOT_NAME}}_link0"/>
    <child link="{{ROBOT_NAME}}_link1"/>
    <origin xyz="{{X}} {{Y}} {{Z}}" rpy="{{R}} {{P}} {{Y}}"/>
    <axis xyz="0 0 1"/>
    <limit lower="{{LOWER}}" upper="{{UPPER}}"
           effort="{{EFFORT}}" velocity="{{VEL}}"/>
    <dynamics damping="0.5"/>
  </joint>

  <!-- Repeat for remaining links and joints -->
</robot>
```

### SDF Object Template

```xml
<sdf version="1.6">
  <model name="{{OBJECT_NAME}}">
    <pose>{{X}} {{Y}} {{Z}} 0 0 0</pose>
    <link name="{{OBJECT_NAME}}_link">
      <inertial>
        <mass>{{MASS}}</mass>
        <inertia>
          <ixx>{{IXX}}</ixx>
          <iyy>{{IYY}}</iyy>
          <izz>{{IZZ}}</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box><size>{{SX}} {{SY}} {{SZ}}</size></box>
        </geometry>
        <material><diffuse>{{R}} {{G}} {{B}} {{A}}</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>{{SX}} {{SY}} {{SZ}}</size></box>
        </geometry>
        <drake:proximity_properties>
          <drake:compliant_hydroelastic/>
          <drake:hydroelastic_modulus>1e7</drake:hydroelastic_modulus>
          <drake:mu_dynamic>0.4</drake:mu_dynamic>
        </drake:proximity_properties>
      </collision>
    </link>
  </model>
</sdf>
```

---

## ✅ Validation Rules

### URDF Validation Checklist

- [ ] All link names unique
- [ ] All joint names unique
- [ ] All parent/child links exist
- [ ] No kinematic loops
- [ ] All masses > 0
- [ ] Inertia matrices satisfy triangle inequality
- [ ] Joint limits: `lower < upper`
- [ ] Effort and velocity > 0
- [ ] Collision geometries defined
- [ ] Mesh file paths valid

### Drake Code Validation Checklist

- [ ] Essential imports present
- [ ] Meshcat started and cleared
- [ ] `plant.Finalize()` called BEFORE visualization
- [ ] All system ports connected
- [ ] Initial states set
- [ ] Time step reasonable (0.0001-0.01)
- [ ] Base link welded if needed
- [ ] File paths absolute for `file://` URLs

---

## 🐛 Common Errors and Fixes

### URDF Errors

**Error**: `Link 'xyz' not found`
- **Cause**: Joint references undefined link
- **Fix**: Ensure all `<parent>` and `<child>` links are defined

**Error**: `Inertia matrix not positive definite`
- **Cause**: Invalid inertia values
- **Fix**: Use formulas from `inertia_templates.json` or CAD export

### Drake Errors

**Error**: `RuntimeError: Plant is already finalized`
- **Cause**: Trying to modify plant after `Finalize()`
- **Fix**: Move all modifications BEFORE `plant.Finalize()`

**Error**: `Robot falls through floor`
- **Cause**: Missing base weld or ground collision
- **Fix**: `plant.WeldFrames(plant.world_frame(), base_link.body_frame())`

**Symptom**: Simulation explodes, NaN values
- **Causes**: Invalid inertia, time step too large, penetrating initial conditions
- **Fixes**: Validate URDF, reduce time_step, check initial poses

---

## 📚 Executable Templates

All Python templates in `patterns/drake_code_patterns/templates/` are fully executable and documented:

1. **01_minimal_simulation.py**: Load and visualize a robot
2. **02_robot_with_pid_controller.py**: PD+G controller for joint control
3. **03_inverse_kinematics.py**: IK solver for end-effector positioning

Each template includes:
- Detailed comments explaining each section
- Customization points clearly marked
- Best practices and common pitfalls
- Complete working code ready to run

---

## 🎓 Learning Path

### Beginner
1. Read `01_minimal_simulation.py` template
2. Study URDF structure in `robot_type_templates.json`
3. Understand validation rules in `urdf_validation_rules.json`

### Intermediate
1. Study `02_robot_with_pid_controller.py`
2. Learn inertia calculations from `inertia_templates.json`
3. Explore SDF object templates

### Advanced
1. Implement inverse kinematics with `03_inverse_kinematics.py`
2. Study complete pick-and-place example
3. Build custom controllers and planners

---

## 🔧 Usage for LLM Training

### Few-Shot Prompting

Use templates as few-shot examples:

```
Example 1: [template from 01_minimal_simulation.py]
Example 2: [template from 02_robot_with_pid_controller.py]

User request: Create a simulation with a UR5 robot...
LLM: [generates code following patterns]
```

### Fine-Tuning Dataset

Format for supervised fine-tuning:

```json
{
  "instruction": "Create a URDF for a 6-DOF robot arm",
  "input": "Robot name: my_robot, link masses: [5, 4, 3, 2, 1, 0.5]",
  "output": "[Generated URDF following templates]",
  "patterns_used": ["serial_manipulator", "sphere_collision"]
}
```

### Validation Layer

Use validation rules to check LLM outputs:

1. Parse generated URDF/SDF
2. Apply rules from `urdf_validation_rules.json`
3. Flag violations for correction
4. Retry generation with error feedback

---

## 📈 Statistics Summary

### File Analysis
- **URDF files**: 40 total
- **SDF files**: 43 total
- **Python scripts**: 6 tutorials

### Robot Types Distribution
- Serial manipulators: 42.5%
- Grippers/hands: 12.5%
- Mobile bases: 5%
- Humanoids: 5%
- Quadrupeds: 2.5%
- Other: 32.5%

### Joint Type Distribution
- Revolute: 95.2% (216 joints)
- Prismatic: 4.8% (11 joints)

### Collision Geometry Distribution
- Spheres: 77.8% (631 instances)
- Cylinders: 8.0% (65 instances)
- Boxes: 7.8% (63 instances)
- Meshes: 6.2% (50 instances)

### Controller Usage
- With controllers: 66.7%
- With IK: 50%
- With trajectory planning: 33.3%
- With logging: 66.7%

---

## 🚀 Next Steps

### Recommended Additions

1. **More complete examples**: Add 5-10 more full examples covering different scenarios
2. **Trajectory planning patterns**: Extract path planning algorithms
3. **Contact dynamics patterns**: Document force/torque control
4. **Multi-robot patterns**: Extract patterns for robot-robot interaction
5. **Sensor integration**: Document camera, lidar, force sensor usage

### For LLM Integration

1. Create prompt templates for each use case
2. Build automated validation pipeline
3. Develop iterative refinement prompts
4. Create debugging assistant prompts

---

## 📖 References

- **Drake Documentation**: https://drake.mit.edu/
- **Drake Tutorials**: https://deepnote.com/workspace/Drake-0b3b2c53-a7ad-441b-80f8-bf8350752305
- **Manipulation Course**: http://manipulation.csail.mit.edu/
- **URDF Specification**: http://wiki.ros.org/urdf/XML
- **SDF Format**: http://sdformat.org/

---

## 🤝 Contributing

To extend this pattern library:

1. Add new templates to `patterns/`
2. Document validation rules in `validation/`
3. Create complete examples in `examples/complete/`
4. Update statistics in `analysis/`
5. Run `extract_patterns.py` to regenerate statistics

---

## 📄 License

This extraction is based on the Robotics-II-fork repository. Please refer to the original repository for licensing information.

---

## ✨ Acknowledgments

Patterns extracted from the Robotics-II-fork repository, which includes:
- Drake models from Toyota Research Institute
- Franka Emika Panda robot description
- KUKA iiwa robot description
- Various open-source robot models

**Generated**: 2025-11-19

**Total Patterns Extracted**: 1,000+

**Ready for LLM Training**: ✅

---

*For questions or issues, please refer to the original repository or Drake documentation.*
