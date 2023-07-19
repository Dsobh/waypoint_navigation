from typing import List
import math
import random
import rclpy

from rclpy.logging import LoggingSeverity
from waypoint_navigation_msgs.action import NavigateToWp
from waypoint_navigation_msgs.msg import Wp
from waypoint_navigation_msgs.srv import (
    GetWps
)

from nav_msgs.msg import Path
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import LaserScan
from simple_node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

class WayPointNavigationHospitalNode(Node):
    DISTANCE_THRESHOLD = 1.2
    OBSTACLE_DISTANCE_THRESHOLD = 1.4

    def __init__(self):
        super().__init__("waypoint_navigation_hospital_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        #actions
        #we create an action in the client to navigate
        
        self.__action_client = self.create_action_client(
            NavigateToWp, "/waypoint_navigation/navigate_to_wp")
            
        self.__gpws_client = self.create_client(GetWps, '/waypoint_navigation/get_wps')
        
        self.__gpws_client.wait_for_service()

        self.previous_distance = float('inf')
        self.previous_goal_id = None
        self.finished_goals = []
        self.new_plan = 0
        self.navigation_status = None

        self.scan_qos = QoSProfile(
          durability=QoSDurabilityPolicy.VOLATILE,
          reliability=QoSReliabilityPolicy.BEST_EFFORT,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=5)

        #Topics subscription
        self.subscription_plan = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, self.scan_qos)
        self.subscription_nav_status = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status_callback, 10)

        
    def calculate_distance(self, p1, p2):
        distance = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2)
        return distance
    
    def plan_callback(self, msg):
        total_distance = 0

        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            distance = self.calculate_distance(p1, p2)
            total_distance += distance
            
        if total_distance > self.previous_distance * WayPointNavigationHospitalNode.DISTANCE_THRESHOLD:
            self.get_logger().info('Previous distance: {:.2f} meters'.format(self.previous_distance))
            self.get_logger().info('Total distance: {:.2f} meters'.format(total_distance))
            
            if min(self.last_scan.ranges) < WayPointNavigationHospitalNode.OBSTACLE_DISTANCE_THRESHOLD:
                self.get_logger().info('Obstacle detected')
            else:
                self.get_logger().info('No obstacle detected')
            self.new_plan = 1
        elif self.new_plan == 1:
            self.get_logger().info('Replaning')
            self.new_plan = 0

        self.previous_distance = total_distance

    def scan_callback(self, msg):
        self.last_scan = msg

    def status_callback(self, msg):
        current_status = None
        for status in msg.status_list:
            current_goal_id = status.goal_info.goal_id
            if current_goal_id != self.previous_goal_id and current_goal_id not in self.finished_goals:
                self.previous_goal_id = current_goal_id
                current_status = "Navigation to a new goal has started"
            if status.status in [2, 4, 5, 6] and current_goal_id not in self.finished_goals:
                status_dict = {
                    2: "is in progress",
                    4: "has succeeded",
                    5: "was cancelled",
                    6: "has aborted"
                }
                current_status = "Navigation to the goal {}.".format(status_dict[status.status])
                if status.status in [4, 5, 6]:
                    self.finished_goals.append(current_goal_id)
        if not current_status:
            current_status = "No navigation is running."

        # Update log when status changed
        if self.navigation_status != current_status:
            self.navigation_status = current_status
            self.get_logger().info(current_status)
    
    def run(self):
        self.__gpws_client.wait_for_service()

        wps_list = self.__gpws_client.call(GetWps.Request())
        final_wps=random.choices(wps_list.wps, k=3)
        
        self.get_logger().info('3 Points recevived')
        for i in final_wps:
             goal = NavigateToWp.Goal()
             goal.wp_id = i.id

             self.get_logger().info('A point have been received')
             self.__action_client.wait_for_server()
             
             self.__action_client.send_goal(goal)
             self.get_logger().info('Navigating to the point...')
             self.__action_client.wait_for_result()
             self.get_logger().info('Point reached')

        self.get_logger().info('All point reached')

def main(args=None):
    rclpy.init()

    node = WayPointNavigationHospitalNode()

    node.run()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

       
       

        
              
        

   
                
                
