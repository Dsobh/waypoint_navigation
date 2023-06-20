from typing import List
import random
import rclpy

from geometry_msgs.msg import Pose
from nav2_msgs.action import NavigateToPose
from waypoint_navigation_msgs.action import NavigateToWp
from waypoint_navigation_msgs.msg import Wp
from waypoint_navigation_msgs.srv import (
    AddWp,
    GetWp,
    GetWps
)

from simple_node import Node

class WayPointNavigationHospitalNode(Node):
       
    def __init__(self):
        super().__init__("waypoint_navigation_hospital_node")

       
        #actions
        #we create an action in the client to navigate
        
        self.__action_client = self.create_action_client(
            NavigateToWp, "/waypoint_navigation/navigate_to_wp")
            
        self.__gpws_client = self.create_client(GetWps, '/waypoint_navigation/get_wps')
        
        self.__gpws_client.wait_for_service()

        wps_list = self.__gpws_client.call(GetWps.Request())
        final_wps=random.choices(wps_list.wps, k=3)
        
        for i in final_wps:
             goal = NavigateToWp.Goal()
             goal.wp_id = i.id

             
             self.__action_client.wait_for_server()
             self.__action_client.send_goal(goal)

             self.__action_client.wait_for_result()

        
    
def main(args=None):
    rclpy.init()

    node = WayPointNavigationHospitalNode()

    node.join_spin()

    rclpy.shutdown()

if __name__ == '__main__':
    main()

       
       

        
              
        

   
                
                
