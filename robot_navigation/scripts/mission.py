#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist  # 确保导入 Twist 消息类型
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from math import pi
from std_srvs.srv import Empty
from robot_navigation.srv import detect  # 导入detect服务类型


class MissionNavigator:
    def __init__(self):
        rospy.init_node('mission_node', anonymous=False)
        rospy.on_shutdown(self._shutdown)

        # 目标点的方向角度（四元数转换）
        quaternions = []
        euler_angles = (0, 0, pi / 2, pi / 2, -pi / 2, pi / 2, pi / 2, pi, pi, -pi / 2, pi)

        for angle in euler_angles:
            quaternion = quaternion_from_euler(0, 0, angle, axes='sxyz')
            quaternions.append(Quaternion(*quaternion))

        # 定义每个目标点的位置与朝向
        self.waypoints = [
            Pose(Point(0.4, 0.4, 0), quaternions[0]),
            Pose(Point(3.831, 0.489, 0), quaternions[1]),
            Pose(Point(3.670, 1.572, 0), quaternions[2]),
            Pose(Point(2.990, 1.330, 0), quaternions[3]),
            Pose(Point(2.990, 1.997, 0), quaternions[4]),
            Pose(Point(2.185, 1.881, 0), quaternions[5]),
            Pose(Point(2.210, 3.839, 0), quaternions[6]),
            Pose(Point(1.068, 3.839, 0), quaternions[7]),
            Pose(Point(1.190, 3.088, 0), quaternions[8]),
            Pose(Point(0.865, 0.388, 0), quaternions[9]),
            Pose(Point(0.516, 0.468, 0), quaternions[10]),
        ]

        # 控制相关变量
        self.current_goal_index = 1
        self.stop_flag = False
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 订阅 MoveBaseAction 服务
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")

        self.move_base_client.wait_for_server(rospy.Duration(60))
        rospy.loginfo("MoveBase action server available!")

        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # 服务代理：识别任务
        self.recognize_person_client = rospy.ServiceProxy("recognize_person", detect)
        self.recognize_plate_client = rospy.ServiceProxy("recognize_plate", detect)

        rospy.loginfo("Mission started!")

        # 执行任务
        self._execute_mission()

    def _move_to_goal(self, goal, max_retries=3):
        retries = 0
        while retries < max_retries:

            self.move_base_client.send_goal(goal)
            finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(60))

            if finished_within_time and self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Successfully reached goal at position: %s", str(goal.target_pose.pose.position))
                return True
            else:
                rospy.logwarn("Failed to reach goal. Attempt %d of %d.", retries + 1, max_retries)
                retries += 1
        rospy.logerr("Max retries reached for goal at position: %s. Moving to next goal.", str(goal.target_pose.pose.position))
        return False

    def _execute_mission(self):
        while not rospy.is_shutdown():
            while not self.stop_flag:
                rospy.loginfo("Navigating to goal %d at position: %s", self.current_goal_index, str(self.waypoints[self.current_goal_index].position))

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = self.waypoints[self.current_goal_index]

                if self._move_to_goal(goal):
                    try:
                        self.clear_costmaps_service()
                        rospy.loginfo("Cleared costmaps after reaching goal %d.", self.current_goal_index)
                    except rospy.ServiceException as e:
                        rospy.logerr("Failed to clear costmaps after reaching goal %d: %s", self.current_goal_index, str(e))

                    # Handle post-goal actions
                    self.current_goal_index += 1
                    if self.current_goal_index == 11:
                        self.stop_flag = True
                        rospy.loginfo("Navigation task completed.")  # 输出任务完成日志
                    elif self.current_goal_index == 4:
                        self._recognize_person_at(1)
                    elif self.current_goal_index == 5:
                        self._recognize_person_at(2)
                    elif self.current_goal_index == 9:
                        self._recognize_plate_at(3)

    def _recognize_person_at(self, point):
        rospy.loginfo("Starting person recognition at point %d.", point)
        try:
            result = self.recognize_person_client.call(point)
            rospy.loginfo("Person recognized at point %d: %s", point, result)
        except rospy.ServiceException as e:
            rospy.logerr("Person recognition failed at point %d: %s", point, str(e))

    def _recognize_plate_at(self, point):
        rospy.loginfo("Starting plate recognition at point %d.", point)
        try:
            result = self.recognize_plate_client.call(point)
            rospy.loginfo("Plate recognized at point %d: %s", point, result)
        except rospy.ServiceException as e:
            rospy.logerr("Plate recognition failed at point %d: %s", point, str(e))

    def _shutdown(self):
        # 确保 move_base_client 存在
        if hasattr(self, 'move_base_client'):
            rospy.loginfo("Shutting down. Stopping the robot.")
            self.move_base_client.cancel_goal()
            rospy.sleep(2)
            self.cmd_vel_pub.publish(Twist())  # 发布空的 Twist 消息来停止机器人
            rospy.sleep(1)
        else:
            rospy.logwarn("move_base_client is not initialized.")
        rospy.loginfo("Robot stopped and shutdown complete.")

if __name__ == '__main__':
    try:
        MissionNavigator()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
