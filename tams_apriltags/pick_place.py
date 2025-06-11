import rclpy
import time
from rclpy.logging import get_logger
from geometry_msgs.msg import Pose

from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


def plan_and_execute(
    robot,
    planning_component,
    logger,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)




def main(args=None):
    rclpy.init()

    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings={})  # if any xacro args needed
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    kinova = MoveItPy(
        node_name="moveit_py_node",
        config_dict=moveit_config.to_dict()  # ✅ passes everything: description, controllers, pipeline
    )
 
    kinova_arm = kinova.get_planning_component("gen3")
    planning_scene_monitor = kinova.get_planning_scene_monitor()

    kinova_arm.set_start_state(configuration_name="vertical")
    kinova_arm.set_goal_state(configuration_name="home")

    logger = get_logger("move_group")
    logger.info("MoveItPy instance created")


    plan_and_execute(kinova, kinova_arm, logger, sleep_time=3.0)

    goal_pose = Pose()
    goal_pose.position.x = 0.5
    goal_pose.position.y = 0.0
    goal_pose.position.z = 0.80  # slightly above cube
    goal_pose.orientation.w = 1.0  # simple upright pose

