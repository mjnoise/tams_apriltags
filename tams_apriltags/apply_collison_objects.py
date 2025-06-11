import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


class CubePublisher(Node):
    def __init__(self):
        super().__init__('cube_publisher')
        self.publisher = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.publish_cube("cube1", [0.5, 0.0, 0.0]) 
        self.publish_cube("cube2", [-0.5, 0.3, 0.0])

    def publish_cube(self, cube_id, pos):
        obj = CollisionObject()
        obj.id = cube_id
        obj.header = Header()
        obj.header.frame_id = "world"
        obj.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.077, 0.077, 0.077]  # Size of cube

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)

        self.publisher.publish(obj)
        self.get_logger().info(f"Published {cube_id} to planning scene")


def main(args=None):
    rclpy.init(args=args)
    node = CubePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
