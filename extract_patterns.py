"""
Drake Pattern Extraction Script
Systematically analyzes URDF, SDF, and Python files to extract patterns for LLM training
"""

import os
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import defaultdict
import re

class DrakePatternExtractor:
    def __init__(self, repo_root):
        self.repo_root = Path(repo_root)
        self.output_dir = self.repo_root / "drake_patterns_extraction"

    def extract_urdf_patterns(self):
        """Extract patterns from all URDF files"""
        urdf_files = list(self.repo_root.glob("**/*.urdf"))

        patterns = {
            "total_analyzed": len(urdf_files),
            "robot_patterns": {},
            "joint_patterns": defaultdict(list),
            "link_patterns": defaultdict(list),
            "inertia_patterns": [],
            "collision_patterns": defaultdict(int),
            "material_patterns": defaultdict(int)
        }

        for urdf_path in urdf_files:
            try:
                tree = ET.parse(urdf_path)
                root = tree.getroot()

                robot_name = root.get('name', 'unknown')

                # Analyze structure
                links = root.findall('.//link')
                joints = root.findall('.//joint')

                robot_data = {
                    "file": str(urdf_path.relative_to(self.repo_root)),
                    "num_links": len(links),
                    "num_joints": len(joints),
                    "joint_types": {},
                    "collision_geometries": {}
                }

                # Analyze joints
                for joint in joints:
                    jtype = joint.get('type')
                    if jtype not in robot_data["joint_types"]:
                        robot_data["joint_types"][jtype] = 0
                    robot_data["joint_types"][jtype] += 1

                    # Extract joint limits for revolute/prismatic
                    if jtype in ['revolute', 'prismatic']:
                        limit = joint.find('limit')
                        if limit is not None:
                            joint_info = {
                                "name": joint.get('name'),
                                "type": jtype,
                                "lower": float(limit.get('lower', 0)),
                                "upper": float(limit.get('upper', 0)),
                                "effort": float(limit.get('effort', 0)),
                                "velocity": float(limit.get('velocity', 0))
                            }

                            # Extract axis
                            axis = joint.find('axis')
                            if axis is not None:
                                joint_info["axis"] = axis.get('xyz')

                            patterns["joint_patterns"][jtype].append(joint_info)

                # Analyze links
                for link in links:
                    link_name = link.get('name')

                    # Inertial properties
                    inertial = link.find('inertial')
                    if inertial is not None:
                        mass_elem = inertial.find('mass')
                        inertia_elem = inertial.find('inertia')

                        if mass_elem is not None and inertia_elem is not None:
                            inertia_data = {
                                "link_name": link_name,
                                "mass": float(mass_elem.get('value', 0)),
                                "ixx": float(inertia_elem.get('ixx', 0)),
                                "iyy": float(inertia_elem.get('iyy', 0)),
                                "izz": float(inertia_elem.get('izz', 0)),
                                "ixy": float(inertia_elem.get('ixy', 0)),
                                "ixz": float(inertia_elem.get('ixz', 0)),
                                "iyz": float(inertia_elem.get('iyz', 0))
                            }
                            patterns["inertia_patterns"].append(inertia_data)

                    # Collision geometries
                    collisions = link.findall('collision')
                    for collision in collisions:
                        geom = collision.find('.//geometry/*')
                        if geom is not None:
                            geom_type = geom.tag
                            patterns["collision_patterns"][geom_type] += 1
                            if geom_type not in robot_data["collision_geometries"]:
                                robot_data["collision_geometries"][geom_type] = 0
                            robot_data["collision_geometries"][geom_type] += 1

                # Categorize robot type
                robot_type = self._categorize_robot(robot_name, robot_data)
                if robot_type not in patterns["robot_patterns"]:
                    patterns["robot_patterns"][robot_type] = []
                patterns["robot_patterns"][robot_type].append(robot_data)

            except Exception as e:
                print(f"Error processing {urdf_path}: {e}")

        return patterns

    def _categorize_robot(self, name, data):
        """Categorize robot by type based on name and structure"""
        name_lower = name.lower()
        num_joints = data["num_joints"]

        if any(word in name_lower for word in ['panda', 'iiwa', 'jaco', 'kinova', 'ur', 'arm']):
            return "serial_manipulator"
        elif any(word in name_lower for word in ['hand', 'finger', 'gripper']):
            return "gripper_hand"
        elif any(word in name_lower for word in ['husky', 'ridgeback', 'mobile', 'cart']):
            return "mobile_base"
        elif any(word in name_lower for word in ['atlas', 'pr2', 'humanoid']):
            return "humanoid"
        elif any(word in name_lower for word in ['cheetah', 'quadruped']):
            return "quadruped"
        else:
            return "other"

    def extract_sdf_patterns(self):
        """Extract patterns from SDF files"""
        sdf_files = list(self.repo_root.glob("**/*.sdf"))

        patterns = {
            "total_analyzed": len(sdf_files),
            "object_patterns": [],
            "world_patterns": [],
            "material_patterns": defaultdict(list)
        }

        for sdf_path in sdf_files:
            try:
                tree = ET.parse(sdf_path)
                root = tree.getroot()

                models = root.findall('.//model')
                for model in models:
                    model_name = model.get('name', 'unknown')

                    # Check if it's a world or object
                    is_static = model.find('static')
                    static_val = is_static.text if is_static is not None else "false"

                    links = model.findall('.//link')
                    model_data = {
                        "name": model_name,
                        "file": str(sdf_path.relative_to(self.repo_root)),
                        "static": static_val.lower() == "true",
                        "num_links": len(links)
                    }

                    # Extract geometry types
                    geometries = []
                    for link in links:
                        for geom_type in ['visual', 'collision']:
                            geom_elements = link.findall(f'.//{geom_type}/geometry/*')
                            for geom in geom_elements:
                                geometries.append(geom.tag)

                    model_data["geometries"] = list(set(geometries))

                    # Extract materials
                    materials = model.findall('.//material')
                    for mat in materials:
                        diffuse = mat.find('diffuse')
                        if diffuse is not None:
                            patterns["material_patterns"][model_name].append(diffuse.text)

                    if 'world' in str(sdf_path) or 'scene' in str(sdf_path):
                        patterns["world_patterns"].append(model_data)
                    else:
                        patterns["object_patterns"].append(model_data)

            except Exception as e:
                print(f"Error processing {sdf_path}: {e}")

        return patterns

    def extract_python_patterns(self):
        """Extract patterns from Python tutorial scripts"""
        python_files = list((self.repo_root / "tutorial_scripts").glob("*.py"))

        patterns = {
            "total_analyzed": len(python_files),
            "scripts": []
        }

        for py_path in python_files:
            try:
                with open(py_path, 'r') as f:
                    content = f.read()

                script_data = {
                    "name": py_path.name,
                    "imports": [],
                    "drake_components": [],
                    "has_controller": False,
                    "has_ik": False,
                    "has_trajectory": False,
                    "has_logging": False
                }

                # Extract imports
                import_lines = re.findall(r'^(?:from|import)\s+[\w\.]+', content, re.MULTILINE)
                script_data["imports"] = import_lines[:10]  # First 10 imports

                # Detect patterns
                if 'InverseKinematics' in content:
                    script_data["has_ik"] = True
                if 'LeafSystem' in content:
                    script_data["has_controller"] = True
                if 'trajectory' in content.lower():
                    script_data["has_trajectory"] = True
                if 'LogVectorOutput' in content:
                    script_data["has_logging"] = True

                # Detect Drake components
                components = []
                if 'DiagramBuilder' in content:
                    components.append("DiagramBuilder")
                if 'MultibodyPlant' in content:
                    components.append("MultibodyPlant")
                if 'SceneGraph' in content:
                    components.append("SceneGraph")
                if 'StartMeshcat' in content:
                    components.append("Meshcat")
                if 'Simulator' in content:
                    components.append("Simulator")

                script_data["drake_components"] = components
                patterns["scripts"].append(script_data)

            except Exception as e:
                print(f"Error processing {py_path}: {e}")

        return patterns

    def generate_statistics(self, urdf_patterns, sdf_patterns, python_patterns):
        """Generate summary statistics"""
        stats = {
            "urdf_analysis": {
                "total_files": urdf_patterns["total_analyzed"],
                "robot_types": {k: len(v) for k, v in urdf_patterns["robot_patterns"].items()},
                "total_joints_analyzed": sum(len(v) for v in urdf_patterns["joint_patterns"].values()),
                "joint_type_distribution": {k: len(v) for k, v in urdf_patterns["joint_patterns"].items()},
                "collision_geometry_distribution": dict(urdf_patterns["collision_patterns"])
            },
            "sdf_analysis": {
                "total_files": sdf_patterns["total_analyzed"],
                "objects": len(sdf_patterns["object_patterns"]),
                "worlds": len(sdf_patterns["world_patterns"])
            },
            "python_analysis": {
                "total_scripts": python_patterns["total_analyzed"],
                "with_ik": sum(1 for s in python_patterns["scripts"] if s["has_ik"]),
                "with_controller": sum(1 for s in python_patterns["scripts"] if s["has_controller"]),
                "with_trajectory": sum(1 for s in python_patterns["scripts"] if s["has_trajectory"]),
                "with_logging": sum(1 for s in python_patterns["scripts"] if s["has_logging"])
            }
        }

        return stats

    def save_json(self, data, filename):
        """Save data to JSON file"""
        filepath = self.output_dir / filename
        filepath.parent.mkdir(parents=True, exist_ok=True)
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=4, default=str)
        print(f"Saved: {filepath}")

    def run_extraction(self):
        """Run all extraction tasks"""
        print("Starting Drake pattern extraction...")

        # Extract URDF patterns
        print("\n1. Extracting URDF patterns...")
        urdf_patterns = self.extract_urdf_patterns()
        self.save_json(urdf_patterns, "patterns/urdf_patterns/extracted_patterns.json")

        # Extract SDF patterns
        print("\n2. Extracting SDF patterns...")
        sdf_patterns = self.extract_sdf_patterns()
        self.save_json(sdf_patterns, "patterns/sdf_patterns/extracted_patterns.json")

        # Extract Python patterns
        print("\n3. Extracting Python patterns...")
        python_patterns = self.extract_python_patterns()
        self.save_json(python_patterns, "patterns/drake_code_patterns/extracted_patterns.json")

        # Generate statistics
        print("\n4. Generating statistics...")
        stats = self.generate_statistics(urdf_patterns, sdf_patterns, python_patterns)
        self.save_json(stats, "analysis/repository_statistics.json")

        print("\n✓ Pattern extraction complete!")
        return urdf_patterns, sdf_patterns, python_patterns, stats


if __name__ == "__main__":
    repo_root = "/home/user/Robotics-II-fork"
    extractor = DrakePatternExtractor(repo_root)
    extractor.run_extraction()
