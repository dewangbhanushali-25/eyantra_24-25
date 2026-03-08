import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2

class IKSolverTest(Node):
    def __init__(self):
        super().__init__('ik_solver_test')

        # Initialize MoveIt2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
            ],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="ur_manipulator"
        )

        # Test the IK function
        self.test_ik()

    def test_ik(self):
        position = [0.3, 0.1, 0.5]  # Example reachable position
        orientation = [0.0, 0.0, 0.0, 1.0]  # Example quaternion

        try:
            joint_angles = self.moveit2.compute_ik(position=position, quat_xyzw=orientation)
            if joint_angles:
                self.get_logger().info(f"Computed IK joint angles: {joint_angles}")
            else:
                self.get_logger().warn("No IK solution found.")
        except Exception as e:
            self.get_logger().error(f"Error in compute_ik: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IKSolverTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
