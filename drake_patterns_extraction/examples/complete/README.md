# Complete Drake Simulation Examples

This directory contains **6 complete, real-world Drake simulation scenarios** extracted from actual robotics projects. Each example demonstrates specific patterns and use cases for training LLMs to generate Drake simulations.

---

## Quick Reference

| ID | Name | Complexity | Robot Count | Mobile | Custom Tools | Key Pattern |
|----|------|------------|-------------|--------|--------------|-------------|
| 01 | Pick & Place Sorting | ⭐⭐ | 1 | No | No | Sorting bins |
| 02 | Drawing Robot | ⭐⭐⭐ | 1 | No | **Yes** | Custom end-effector |
| 03 | Mobile Stacking | ⭐⭐⭐⭐ | 1 | **Yes (1-DOF)** | No | Prismatic base |
| 04 | Door Opening | ⭐⭐⭐⭐⭐ | 1 | **Yes (3-DOF)** | No | Articulated environment |
| 05 | TAMP with Obstacles | ⭐⭐⭐ | 1 | No | No | Obstacle avoidance |
| 06 | Object Handover | ⭐⭐⭐ | **2** | No | No | Multi-robot coordination |

---

## Example Descriptions

### 01: Pick and Place with Sorting Bins ⭐⭐

**Scenario**: Single Jaco arm sorts colored objects into color-coded bins

**Use Cases**:
- Warehouse sorting
- Quality control
- Parts organization

**Key Learnings**:
- Setting up sorting bins
- Mixed object types (cubes, cylinders, capsules)
- Color-based task logic

**Patterns Demonstrated**:
```python
patterns_used = [
    "single_robot_fixed_base",
    "sorting_bins_array",
    "mixed_object_types",
    "static_workspace"
]
```

**Files**: `01_pick_and_place_sorting/`
- `example.json` - Complete scenario specification
- `world.sdf` - Full SDF world file

---

### 02: Drawing Robot with Custom End-Effector ⭐⭐⭐

**Scenario**: Panda arm with pen tool drawing on vertical whiteboard

**Use Cases**:
- Writing/drawing tasks
- Surface inspection
- Contact-based manipulation

**Key Learnings**:
- Attaching custom tools to robot
- Vertical workspace setup
- Precision contact control

**Patterns Demonstrated**:
```python
patterns_used = [
    "custom_end_effector",
    "vertical_workspace",
    "tool_attachment",
    "contact_task"
]
```

**Novel Elements**:
```xml
<!-- Custom pen tool attached to robot end-effector -->
<joint name="pen_fixed_joint" type="fixed">
  <parent>panda::panda_link7</parent>
  <child>pen_endeffector::pen_link</child>
</joint>
```

**Files**: `02_drawing_robot/`
- Contains full SDF with custom end-effector definition

---

### 03: Mobile Manipulation - Stacking ⭐⭐⭐⭐

**Scenario**: Panda on 1-DOF prismatic base stacking colored cubes

**Use Cases**:
- Warehouse automation
- Extended reach tasks
- Assembly lines

**Key Learnings**:
- Mobile base configuration (1-DOF prismatic)
- Base-mounted manipulator setup
- Extended workspace coverage

**Patterns Demonstrated**:
```python
patterns_used = [
    "mobile_base_1dof",
    "base_mounted_arm",
    "prismatic_joint",
    "extended_workspace"
]
```

**Mobile Base Pattern**:
```xml
<joint name="base_slide_x" type="prismatic">
  <parent>world</parent>
  <child>base_link</child>
  <axis><xyz>1 0 0</xyz>
    <limit>
      <lower>-1.5</lower>
      <upper>1.5</upper>
      <effort>100.0</effort>
      <velocity>1.0</velocity>
    </limit>
  </axis>
</joint>

<!-- Attach robot to mobile base -->
<joint name="panda_to_mobile_base" type="fixed">
  <parent>mobile_base::base_link</parent>
  <child>panda::panda_link0</child>
</joint>
```

**Files**: `03_mobile_stacking/`

---

### 04: Door Opening with Planar Mobile Base ⭐⭐⭐⭐⭐

**Scenario**: Panda on 3-DOF planar base opening hinged door

**Use Cases**:
- Service robotics
- Home automation
- Indoor navigation

**Key Learnings**:
- Planar mobile base (x, y, yaw)
- Articulated environment (hinged door)
- Contact-rich manipulation
- Chained joint configuration

**Patterns Demonstrated**:
```python
patterns_used = [
    "mobile_base_3dof",
    "articulated_environment",
    "hinged_door",
    "contact_rich_manipulation"
]
```

**Planar Base Pattern** (Chained Joints):
```xml
<!-- Virtual intermediate links for kinematic chain -->
<link name="x_link"/>
<link name="y_link"/>

<joint name="base_slide_x" type="prismatic">
  <parent>world</parent><child>x_link</child>
  <axis><xyz>1 0 0</xyz></axis>
</joint>
<joint name="base_slide_y" type="prismatic">
  <parent>x_link</parent><child>y_link</child>
  <axis><xyz>0 1 0</xyz></axis>
</joint>
<joint name="base_yaw" type="revolute">
  <parent>y_link</parent><child>base_link</child>
  <axis><xyz>0 0 1</xyz></axis>
</joint>
```

**Articulated Door Pattern**:
```xml
<model name="doorway">
  <link name="door_panel">
    <inertial><mass>25.0</mass></inertial>
    <!-- visual/collision for door -->
  </link>
  <link name="wall_left">...</link>
  <link name="wall_right">...</link>

  <!-- Door hinge -->
  <joint name="door_hinge" type="revolute">
    <parent>wall_left</parent>
    <child>door_panel</child>
    <axis><xyz>0 0 1</xyz>
      <limit><lower>0</lower><upper>1.57</upper></limit>
      <dynamics><damping>3.0</damping></dynamics>
    </axis>
  </joint>
</model>
```

**Files**: `04_door_opening/`

---

### 05: Task and Motion Planning with Obstacles ⭐⭐⭐

**Scenario**: Panda stacking cubes with wall obstacle bisecting workspace

**Use Cases**:
- Constrained planning
- TAMP research
- Obstacle-rich environments

**Key Learnings**:
- Obstacle definition and placement
- Constrained motion planning
- Stacking with obstacles

**Patterns Demonstrated**:
```python
patterns_used = [
    "obstacle_avoidance",
    "constrained_workspace",
    "wall_obstacle",
    "stacking_task"
]
```

**Obstacle Pattern**:
```xml
<!-- Wall bisecting workspace -->
<model name="wall">
  <static>true</static>
  <pose>0.55 0 0.2 0 0 0</pose>
  <link name="wall_top_link">
    <visual>...</visual>
    <collision>
      <geometry>
        <box><size>0.35 0.02 0.3</size></box>
      </geometry>
    </collision>
  </link>
</model>
```

**Files**: `05_tamp_obstacle/`

---

### 06: Multi-Robot Object Handover ⭐⭐⭐

**Scenario**: Two Panda robots on elevated pedestals passing objects

**Use Cases**:
- Collaborative assembly
- Human-robot handover
- Multi-agent systems

**Key Learnings**:
- Multi-robot setup with unique naming
- Elevated robot mounting
- Handover zone creation
- Coordination primitives

**Patterns Demonstrated**:
```python
patterns_used = [
    "multi_robot_elevated",
    "handover_zone",
    "coordinated_manipulation",
    "elevated_pedestals"
]
```

**Multi-Robot Pattern**:
```xml
<!-- Robot 1 -->
<include>
  <uri>file://robot.urdf</uri>
  <n>panda_1</n>  <!-- Unique name -->
  <pose>0.0 0.5 0.4 0 0 0</pose>  <!-- Elevated -->
</include>
<joint name="world_to_panda_1" type="fixed">
  <parent>world</parent>
  <child>panda_1::panda_link0</child>
</joint>

<!-- Robot 2 (mirrored) -->
<include>
  <uri>file://robot.urdf</uri>
  <n>panda_2</n>  <!-- Different name -->
  <pose>0.0 -0.5 0.4 0 0 0</pose>  <!-- Mirrored position -->
</include>
<joint name="world_to_panda_2" type="fixed">
  <parent>world</parent>
  <child>panda_2::panda_link0</child>
</joint>

<!-- Pedestals for elevation -->
<model name="panda_base_1">
  <static>true</static>
  <pose>0.0 0.5 0.2 0 0 0</pose>  <!-- Height 0.4m -->
  <link name="panda_base_1_link">...</link>
</model>
```

**Files**: `06_object_handover/`

---

## Pattern Summary

### New Patterns Identified

| Category | Pattern | Complexity | Examples |
|----------|---------|------------|----------|
| **Mobile Bases** | 1-DOF Prismatic | Medium | 03 |
| **Mobile Bases** | 3-DOF Planar | High | 04 |
| **End-Effectors** | Custom Pen Tool | Medium | 02 |
| **Workspaces** | Vertical Whiteboard | Medium | 02 |
| **Environment** | Hinged Door | High | 04 |
| **Environment** | Wall Obstacle | Low | 05 |
| **Multi-Robot** | Elevated Dual | Medium | 06 |
| **Task Logic** | Color Sorting | Low | 01 |

---

## Natural Language → Example Mapping

Use these mappings to route user requests to appropriate examples:

| User Says | Maps To | Example |
|-----------|---------|---------|
| "sort objects by color" | Sorting | 01 |
| "draw a shape" | Custom tool | 02 |
| "robot on wheels" | Mobile base | 03, 04 |
| "open a door" | Articulated env | 04 |
| "avoid obstacles" | Constrained planning | 05 |
| "two robots working together" | Multi-robot | 06 |

---

## Usage for LLM Training

### Few-Shot Prompting

```python
# Example prompt structure
prompt = f"""
Given the following example scenarios:

Example 1: {load_example("01_pick_and_place_sorting")}
Example 2: {load_example("03_mobile_stacking")}

User Request: "Create a mobile robot that sorts packages"

Generate SDF and Python code for this scenario.
"""
```

### Fine-Tuning Dataset

```json
{
  "instruction": "Create an SDF world with a mobile robot",
  "input": "Robot should move in one direction and stack cubes",
  "output": "<sdf from example 03>",
  "patterns": ["mobile_base_1dof", "stacking"],
  "example_reference": "03_mobile_stacking"
}
```

### Pattern Retrieval

```python
def get_relevant_examples(user_intent):
    """RAG-based example retrieval"""
    embeddings = embed(user_intent)
    similar = search(embeddings, example_database)
    return [load_example(ex_id) for ex_id in similar]
```

---

## Complexity Progression

Recommended learning order:

1. **Level 2**: Start with `01_pick_and_place_sorting`
   - Basic manipulation
   - Single robot
   - Static workspace

2. **Level 3**: Move to `02_drawing_robot` or `05_tamp_obstacle`
   - Custom tools OR obstacles
   - Precision OR planning

3. **Level 3**: Try `06_object_handover`
   - Multi-robot basics
   - Coordination

4. **Level 4**: Advance to `03_mobile_stacking`
   - Mobile manipulation
   - Extended DOF

5. **Level 5**: Master `04_door_opening`
   - Combined challenges
   - Contact-rich manipulation

---

## File Structure

Each example directory contains:

```
XX_example_name/
├── example.json          # Complete metadata
├── world.sdf            # Full SDF world file
└── patterns_used.json   # Pattern references (optional)
```

---

## Integration with Main Repository

These examples complement the main repository extraction:

- **Main Repo**: Provides robot URDFs, basic objects, and Python templates
- **These Examples**: Show how to combine components into complete scenarios

Use together for comprehensive pattern coverage!

---

## Contributing

To add new examples:

1. Create directory `examples/complete/XX_name/`
2. Add `example.json` with full metadata
3. Include `world.sdf` file
4. Update `index.json`
5. Add to pattern mappings if introducing new patterns

---

## Statistics

- **Total Examples**: 6
- **Lines of SDF**: ~1,500
- **Unique Patterns**: 15+
- **Coverage**: Sorting, drawing, mobile manipulation, multi-robot, planning
- **Complexity Range**: 2-5 stars

---

## Next Steps

1. **For LLM Training**: Use as few-shot examples or fine-tuning data
2. **For Engineers**: Reference as templates for similar scenarios
3. **For Researchers**: Study patterns for Drake best practices

---

**Last Updated**: 2025-11-19
**Status**: ✅ Complete and validated
