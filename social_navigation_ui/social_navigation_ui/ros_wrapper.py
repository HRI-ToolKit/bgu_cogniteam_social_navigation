#ros
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup,ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


# action for nav2
from rclpy.action import ActionClient
from rclpy.task import Future
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped,Pose
from nav2_msgs.action import NavigateToPose,ComputePathToPose
from nav2_msgs.srv import ManageLifecycleNodes,ClearEntireCostmap,GetCostmap,LoadMap

import cv2
import numpy as np
import math
import time


class RosWrapper(Node):
    
    def __init__(self, executor):

        super().__init__('social_navigation_ui_node')

        self.executor:MultiThreadedExecutor = executor
       
        self.actions_cb_group = MutuallyExclusiveCallbackGroup()


        self._action_client_goal = ActionClient(self, NavigateToPose, '/navigate_to_pose',callback_group=self.actions_cb_group)
        self._action_client_goal_path = ActionClient(self, ComputePathToPose, '/compute_path_to_pose',callback_group=self.actions_cb_group)

        #elf.get_logger().info('Waiting for action server...')
        path_action_server = self._action_client_goal_path.wait_for_server(5)
        if not path_action_server:
            #self.get_logger().error('ComputePathToPose action server is not available')
            exit()

        goal_action_server = self._action_client_goal.wait_for_server(5)
        if not goal_action_server:
            #self.get_logger().error('NavigateToPose action server is not available')
            exit()
        self.get_logger().info('NavigateToPose and ComputePathToPose action server is up!')

        # Set up service clients
        self.cli = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes',callback_group=self.actions_cb_group)
        manage_nodes = self.cli.wait_for_service(5)
        if not manage_nodes:
            #self.get_logger().error('lifecycle_manager_navigation service is not available')
            exit()
        self.manage_lifecycle_nodes_req = ManageLifecycleNodes.Request()
        #self.get_logger().info('lifecycle_manager_navigation serivce is up!') 

        #clear-costmap-service
        self.clear_costmap_global_srv = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        while not self.clear_costmap_global_srv.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Clear global costmaps service not available, waiting...')

        
        self.persons_publisher = self.create_publisher(String, '/persons', 10)


    def publishPersons(self, str_persons):

        # Create and publish the string message
        msg = String()
        msg.data = str_persons
        self.persons_publisher.publish(msg)      
    
    def calculatePath(self, goal):

        if goal == None:
            return None

        goalOnMap = PoseStamped()
        goalOnMap.header.frame_id = 'map'
        goalOnMap.pose.position.x = goal[0]
        goalOnMap.pose.position.y = goal[1]
        goalOnMap.pose.position.z = 0.0
        goalOnMap.pose.orientation.w = 1.0   

        path_msg = ComputePathToPose.Goal()
        path_msg.pose = goalOnMap
        path_msg.planner_id = 'GridBased'

        self._send_goal_path_future = self._action_client_goal_path.send_goal_async(
        path_msg)

        self.executor.spin_until_future_complete(self._send_goal_path_future)

        
        self.goal_handle = self._send_goal_path_future.result()



        if not self.goal_handle.accepted:
            self.get_logger().error(f'Goal to ' + str(goal.pose.position.x) + ' ' +
                       str(goal.pose.position.y) + ' was rejected!')
            return None
        
        result:Future = self._send_goal_path_future.result().get_result_async()
        self.executor.spin_until_future_complete(result)
        path = result.result().result.path.poses
        self.goal_handle = None
        # print(' num_of_pose '+ str(path))

        if len(path) == 0:

            return None

        return path 


    def clearCostMap(self):
        
        # Create a request object
        request = ClearEntireCostmap.Request()

        # Call the clear_entirely service
        future = self.clear_costmap_global_srv.call_async(request)

        # Wait for the service call to complete
        self.executor.spin_until_future_complete( future)

        if future.result() is not None:
            # Service call was successful
            self.get_logger().info('Global costmap cleared entirely.')
        else:
            # Service call failed
            self.get_logger().error('Failed to clear global costmap.')

            #self.clearCostMap()


    def destroy_node(self):
        self._action_client_goal.destroy()
        self._action_client_goal_path.destroy()
        # self.resetNav2()
        self.cli.destroy()
        super().destroy_node()