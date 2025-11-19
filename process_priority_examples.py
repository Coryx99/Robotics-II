"""
Process Priority SDF Examples
Extracts patterns from 6 high-priority Drake simulation examples
"""

import json
import os
from pathlib import Path

# Define the priority examples with their SDF content and metadata
PRIORITY_EXAMPLES = {
    "02_drawing_robot": {
        "name": "Drawing Robot with Custom End-Effector",
        "description": "Panda arm with pen tool drawing on vertical whiteboard",
        "complexity": 3,
        "key_patterns": ["custom_end_effector", "vertical_workspace", "tool_attachment"],
        "tags": ["drawing", "custom-tool", "precision", "contact-task"],
        "sdf_file": "project_02_drawing_robot_world.sdf"
    },
    "03_mobile_stacking": {
        "name": "Mobile Manipulation - Stacking",
        "description": "Panda on 1-DOF prismatic mobile base stacking colored cubes",
        "complexity": 4,
        "key_patterns": ["mobile_base_1dof", "base_mounted_arm", "extended_workspace"],
        "tags": ["mobile", "stacking", "prismatic-base", "multi-object"],
        "sdf_file": "project_03_stacking.sdf"
    },
    "04_door_opening": {
        "name": "Door Opening with Planar Mobile Base",
        "description": "Panda on 3-DOF planar base opening hinged door",
        "complexity": 5,
        "key_patterns": ["mobile_base_3dof", "articulated_environment", "contact_rich"],
        "tags": ["mobile", "door", "planar-base", "articulated-object", "contact-rich"],
        "sdf_file": "project_04_door_opening.sdf"
    },
    "05_tamp_obstacle": {
        "name": "Task and Motion Planning with Obstacles",
        "description": "Panda stacking cubes with wall obstacle",
        "complexity": 3,
        "key_patterns": ["obstacle_avoidance", "constrained_workspace", "stacking"],
        "tags": ["planning", "obstacles", "stacking", "constrained-space"],
        "sdf_file": "project_06_TAMP.sdf"
    },
    "06_object_handover": {
        "name": "Multi-Robot Object Handover",
        "description": "Two elevated Pandas passing objects between them",
        "complexity": 3,
        "key_patterns": ["multi_robot_elevated", "handover_zone", "coordinated_manipulation"],
        "tags": ["multi-robot", "handover", "coordination", "elevated-base"],
        "sdf_file": "project_07_object_handover.sdf"
    }
}

def extract_pattern_summary():
    """Generate summary of new patterns found in priority examples"""

    patterns = {
        "mobile_base_patterns": {
            "1dof_prismatic": {
                "description": "Single axis linear motion for extended reach",
                "examples": ["03_mobile_stacking"],
                "joint_type": "prismatic",
                "typical_limits": [-1.5, 1.5],
                "typical_effort": 100.0,
                "typical_velocity": 1.0,
                "use_cases": ["warehouse", "extended-workspace", "assembly-line"]
            },
            "3dof_planar": {
                "description": "Planar mobility (x, y, yaw) using chained joints",
                "examples": ["04_door_opening"],
                "joint_chain": ["prismatic_x", "prismatic_y", "revolute_yaw"],
                "requires_virtual_links": True,
                "use_cases": ["indoor-navigation", "door-opening", "mobile-manipulation"]
            }
        },

        "end_effector_patterns": {
            "pen_tool": {
                "description": "Cylindrical pen for drawing/writing tasks",
                "examples": ["02_drawing_robot"],
                "geometry": {"type": "cylinder", "radius": 0.005, "length": 0.1},
                "mass": 0.01,
                "attachment_frame": "panda_link7",
                "joint_type": "fixed",
                "use_cases": ["drawing", "writing", "tracing", "inspection"]
            }
        },

        "workspace_patterns": {
            "vertical_whiteboard": {
                "description": "Vertical surface for contact tasks",
                "examples": ["02_drawing_robot"],
                "orientation": "90deg rotation (vertical)",
                "typical_size": [1.0, 0.02, 0.7],
                "material": "white with transparency",
                "positioning": "offset from robot base"
            }
        },

        "articulated_environment_patterns": {
            "hinged_door": {
                "description": "Door with revolute joint and surrounding walls",
                "examples": ["04_door_opening"],
                "components": ["door_panel", "wall_left", "wall_right", "wall_top", "handle"],
                "joint_type": "revolute",
                "axis": [0, 0, 1],
                "limits": [0, 1.57],
                "mass": 25.0,
                "damping": 3.0,
                "use_cases": ["service-robots", "home-automation", "access-control"]
            }
        },

        "multi_robot_patterns": {
            "elevated_dual_setup": {
                "description": "Two robots on elevated pedestals for handover",
                "examples": ["06_object_handover"],
                "configuration": "mirrored with handover zone",
                "pedestal_height": 0.4,
                "robot_separation": 1.0,
                "use_cases": ["handover", "collaborative-assembly", "parts-transfer"]
            }
        },

        "obstacle_patterns": {
            "vertical_wall": {
                "description": "Wall obstacle bisecting workspace",
                "examples": ["05_tamp_obstacle"],
                "typical_size": [0.35, 0.02, 0.3],
                "placement": "between pick and place zones",
                "use_cases": ["constrained-planning", "obstacle-avoidance", "TAMP"]
            }
        }
    }

    return patterns

def generate_natural_language_mappings():
    """Generate mappings from user intent to patterns"""

    mappings = [
        {
            "user_phrases": ["robot on wheels", "mobile robot", "robot that moves", "extend reach"],
            "pattern": "mobile_base",
            "examples": ["03_mobile_stacking", "04_door_opening"],
            "disambiguation": "1-DOF for linear motion, 3-DOF for planar motion"
        },
        {
            "user_phrases": ["open a door", "interact with door", "push door", "pull door"],
            "pattern": "articulated_environment",
            "examples": ["04_door_opening"],
            "confidence": "high"
        },
        {
            "user_phrases": ["draw", "write", "trace", "pen", "marker"],
            "pattern": "custom_end_effector",
            "examples": ["02_drawing_robot"],
            "confidence": "high"
        },
        {
            "user_phrases": ["two robots", "multiple robots", "robots working together", "handover"],
            "pattern": "multi_robot",
            "examples": ["01_pick_and_place_sorting", "06_object_handover"],
            "disambiguation": "shared workspace vs handover zone"
        },
        {
            "user_phrases": ["avoid obstacles", "wall in way", "constrained space", "blocked path"],
            "pattern": "obstacle_avoidance",
            "examples": ["05_tamp_obstacle"],
            "confidence": "high"
        },
        {
            "user_phrases": ["stack blocks", "pile cubes", "vertical arrangement"],
            "pattern": "stacking_task",
            "examples": ["03_mobile_stacking", "05_tamp_obstacle"],
            "confidence": "high"
        },
        {
            "user_phrases": ["sort by color", "colored bins", "categorize objects"],
            "pattern": "sorting_system",
            "examples": ["01_pick_and_place_sorting"],
            "confidence": "high"
        }
    ]

    return mappings

# Generate the pattern summary
patterns = extract_pattern_summary()
mappings = generate_natural_language_mappings()

# Create output directory
output_dir = Path("/home/user/Robotics-II-fork/drake_patterns_extraction/patterns/sdf_patterns")
output_dir.mkdir(parents=True, exist_ok=True)

# Save patterns
with open(output_dir / "advanced_scene_patterns.json", "w") as f:
    json.dump(patterns, f, indent=2)

# Save mappings
with open(output_dir / "user_intent_mappings.json", "w") as f:
    json.dump({"mappings": mappings}, f, indent=2)

print("✓ Advanced scene patterns extracted")
print("✓ User intent mappings created")
print(f"\nNew patterns identified:")
for category, items in patterns.items():
    print(f"  - {category}: {len(items)} patterns")
