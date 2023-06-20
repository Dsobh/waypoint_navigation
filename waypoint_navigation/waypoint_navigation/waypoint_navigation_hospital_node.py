from typing import List
import random
import rclpy

from rclpy.logging import LoggingSeverity
from waypoint_navigation_msgs.action import NavigateToWp
from waypoint_navigation_msgs.msg import Wp
from waypoint_navigation_msgs.srv import (
    GetWps
)

from simple_node import Node

class WayPointNavigationHospitalNode(Node):
       
    def __init__(self):
        super().__init__("waypoint_navigation_hospital_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        #actions
        #we create an action in the client to navigate
        
        self.__action_client = self.create_action_client(
            NavigateToWp, "/waypoint_navigation/navigate_to_wp")
            
        self.__gpws_client = self.create_client(GetWps, '/waypoint_navigation/get_wps')
        
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

    node.join_spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

       
       

        
              
        

   
                
                
