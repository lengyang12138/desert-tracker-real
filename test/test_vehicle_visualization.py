import os
import struct
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class VehicleVisualizationTests(unittest.TestCase):
    def path(self, *parts):
        return os.path.join(ROOT, *parts)

    def test_urdf_is_visual_only_single_base_link(self):
        urdf_path = self.path(
            "src", "vehicle_description", "urdf", "base_link_visual.urdf")
        robot = ET.parse(urdf_path).getroot()
        links = robot.findall("link")
        self.assertEqual([link.get("name") for link in links], ["base_link"])
        self.assertEqual(robot.findall("joint"), [])
        self.assertIsNotNone(links[0].find("visual"))
        self.assertIsNone(links[0].find("collision"))
        self.assertIsNone(links[0].find("inertial"))
        mesh = links[0].find("visual/geometry/mesh")
        self.assertEqual(
            mesh.get("filename"),
            "package://vehicle_description/meshes/base_link_visual.stl")

    def test_mesh_is_github_and_rviz_sized(self):
        mesh_path = self.path(
            "src", "vehicle_description", "meshes", "base_link_visual.stl")
        size = os.path.getsize(mesh_path)
        self.assertLess(size, 20 * 1024 * 1024)
        with open(mesh_path, "rb") as stream:
            stream.seek(80)
            triangles = struct.unpack("<I", stream.read(4))[0]
        self.assertGreater(triangles, 10000)
        self.assertEqual(size, 84 + triangles * 50)

    def test_launch_loads_description_without_publishing_tf(self):
        launch_path = self.path(
            "src", "vehicle_description", "launch",
            "vehicle_visualization.launch")
        root = ET.parse(launch_path).getroot()
        text = ET.tostring(root, encoding="unicode")
        self.assertIn('name="robot_description"', text)
        self.assertIn("base_link_visual.urdf", text)
        self.assertNotIn("robot_state_publisher", text)
        self.assertNotIn("static_transform_publisher", text)

    def test_full_system_can_load_model_and_optionally_start_rviz(self):
        launch_path = self.path(
            "src", "vehicle_bringup", "launch", "full_system.launch")
        root = ET.parse(launch_path).getroot()
        args = {node.get("name"): node.get("default")
                for node in root.findall("arg")}
        self.assertEqual(args["load_vehicle_model"], "true")
        self.assertEqual(args["start_rviz"], "true")
        text = ET.tostring(root, encoding="unicode")
        self.assertIn("vehicle_visualization.launch", text)
        self.assertIn('name="start_rviz" value="$(arg start_rviz)"', text)

    def test_rviz_shows_vehicle_axes_and_goal_pose(self):
        rviz_path = self.path(
            "src", "vehicle_description", "rviz", "real_navigation.rviz")
        with open(rviz_path, encoding="utf-8") as stream:
            config = stream.read()
        self.assertIn("Fixed Frame: map", config)
        self.assertIn("Class: rviz/RobotModel", config)
        self.assertIn("Reference Frame: base_link", config)
        self.assertIn("Class: rviz/Pose", config)
        self.assertIn("Topic: /move_base_simple/goal", config)


if __name__ == "__main__":
    unittest.main()
