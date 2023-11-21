import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import json
import cv2
import numpy as np
import yaml
import random
import math

#ros
import rclpy
from rclpy.node import Node
import ament_index_python
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
from nav2_msgs.srv import ManageLifecycleNodes

package_name = 'social_navigation_ui'
package_share_directory = ament_index_python.get_package_share_directory(package_name)


class Person:
    
    def __init__(self, map_resolution):
        
        self.map_resolution = map_resolution

        self.position_m = (0.0,0.0)

        #https://en.wikipedia.org/wiki/Ellipse
        self.axes_length_a_m = 0.5
        self.axes_length_b_m = 0.2
        self.yaw_deg_angle = 0.0

class PersonsGenerator:

    def __init__(self, num_of_persons, cv_map, 
        map_resolution, map_origin_position_x, 
        map_origin_position_y):

        self.num_of_persons = num_of_persons
        self.cv_map = cv_map
        self.map_resolution = map_resolution
        self.map_origin_position_x = map_origin_position_x
        self.map_origin_position_y = map_origin_position_y

    def generatePersons(self):
        height, width = self.cv_map.shape[:2]
        persons = []
        valid_positions = []

        # Iterate over the image pixels
        for y in range(height):
            for x in range(width):
                # Check if the pixel value is 254
                if self.cv_map[y, x] == 254:
                    valid_positions.append(self.convert_meters_pix_to_pose((x, y)))

        count = 0
        while True:

            if count >= self.num_of_persons:
                break

            person = Person(self.map_resolution)
            person.position_m = self.generate_position(valid_positions)
            person.yaw_deg_angle = self.generate_random_yaw()

            if self.isPersonsOnMap(person):
                persons.append(person)

            count+= 1    

        return persons


    def generate_position(self, valid_positions):

        print('lne of valid_positions ' + str(len(valid_positions)))
        index = random.randint(0, len(valid_positions))
        print('index ' + str(index))
        x_pose, y_pose = valid_positions[index]
        return (x_pose, y_pose)

    def generate_random_yaw(self):
        yaw_value = np.random.uniform(0, 360)
        return yaw_value

    def isPersonsOnMap(self, person):
        height, width = self.cv_map.shape[:2]

        person_x_pix, person_y_pix  = self.convert_pose_to_pix(person.position_m)

        if person_x_pix < 0 or person_x_pix > width or person_y_pix < 0 or person_y_pix > height:
            print('Flase')
            return False
        
        pixel_value = self.cv_map[person_y_pix, person_x_pix]
        # print('pixel_value' + str(pixel_value))

        if pixel_value != 254:
            return False


        return True

    def convert_meters_pix_to_pose(self, x_y):

        x_pose =  (x_y[0] * self.map_resolution) + self.map_origin_position_x
        y_pose =  (x_y[1] * self.map_resolution) + self.map_origin_position_y

        return (x_pose, y_pose)

    def convert_pix_to_meters(self, pix_l):

        meter =     pix_l *  self.map_resolution  
        return   meter  
    
    def convert_meters_to_pix(self, meter):

        pix_l =     meter / self.map_resolution  
        return   pix_l   

    def convert_pose_to_pix(self, pose):

        x = float(pose[0] - self.map_origin_position_x) / float(self.map_resolution)
        y = float(pose[1] - self.map_origin_position_y) / float(self.map_resolution)

        return (int(x),int(y))    

class SocialNavigationUI(Node):
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

        
        self.persons_publisher = self.create_publisher(String, '/persons', 10)


        self.root = tk.Tk()
        self.root.title("BGU social navigation")
        self.root.geometry("1000x1000")
        self.root.configure(bg="black")
        # Variables
        self.json_data = None
        self.image_path = None
        self.cv_map = None 
        self.rgb_image = None 

        self.personsGenerator = None 
        self.num_of_persons = 50

        #map vlaues
        self.map_resolution = 0.0
        self.map_origin_position_x = 0.0
        self.map_origin_position_y = 0.0       

        self.load_map_button = tk.Button(self.root, text="set path Map", command=self.set_path)
        self.load_map_button.pack(pady=10)

        self.update_map_button = tk.Button(self.root, text="Generate map with persons", command=self.update_map)
        self.update_map_button.pack(pady=10)


        self.image_viewer = tk.Label(self.root)
        self.image_viewer.pack(pady=10)

        self.load_map()
    


    def set_path(self):
        
        goalOnMap = PoseStamped()
        goalOnMap.header.frame_id = 'map'
        goalOnMap.pose.position.x = -7.2
        goalOnMap.pose.position.y = -9.2
        goalOnMap.pose.position.z = 0.0
        goalOnMap.pose.orientation.w = 1.0   

        path_msg = ComputePathToPose.Goal()
        path_msg.pose = goalOnMap
        # path_msg.planner_id = 'GridBased'

        print('11111111111111')
        self._send_goal_path_future = self._action_client_goal_path.send_goal_async(
        path_msg)
        print('2222222222222222')

        self.executor.spin_until_future_complete(self._send_goal_path_future)

        print('3333333333333333')
        
        self.goal_handle = self._send_goal_path_future.result()

        print('4444444444444444')


        if not self.goal_handle.accepted:
            self.get_logger().error(f'Goal to ' + str(goal.pose.position.x) + ' ' +
                       str(goal.pose.position.y) + ' was rejected!')
            return False
        
        result:Future = self._send_goal_path_future.result().get_result_async()
        self.executor.spin_until_future_complete(result)
        path = result.result().result.path.poses
        self.goal_handle = None
        # print(' num_of_pose '+ str(path))

        for i in range(len(path) - 1):
            pix_1 = self.convert_pose_to_pix((path[i].pose.position.x,path[i].pose.position.y))
            pix_2 = self.convert_pose_to_pix((path[i+1].pose.position.x,path[i+1].pose.position.y))
            cv2.line(self.rgb_image, pix_1,pix_2 , (0, 255, 200), 2)

        self.setImageViewer()


    def load_map(self):
        #file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.gif")])
        #if file_path:
        self.image_path = package_share_directory+'/map.pgm'
        self.cv_map = cv2.imread(self.image_path, 0)
        self.cv_map = cv2.flip(self.cv_map, 0)

        map_yaml_path = package_share_directory+'/map.yaml'
        # Open the YAML file and load its content
        with open(map_yaml_path, 'r') as file:
            yaml_data = yaml.safe_load(file)

        self.map_resolution = float(yaml_data['resolution'])  
        self.map_origin_position_x = float(yaml_data['origin'][0])
        self.map_origin_position_y = float(yaml_data['origin'][1]) 

        self.personsGenerator = PersonsGenerator(self.num_of_persons, self.cv_map, 
            self.map_resolution, self.map_origin_position_x, 
            self.map_origin_position_y)

       
        self.rgb_image = cv2.cvtColor(self.cv_map, cv2.COLOR_GRAY2RGB)

        #robot position
        
        cv2.circle(self.rgb_image, self.convert_pose_to_pix((-2.5,0.3)), 5, (0,100,200), -1) 

        self.setImageViewer()
        
    def setImageViewer(self):
        
        tk_image = ImageTk.PhotoImage(Image.fromarray(self.rgb_image))
        self.image_viewer.configure(image=tk_image)
        self.image_viewer.image = tk_image  # Keep a reference to avoid garbage collection issues


    def update_map(self):
        if np.all(self.cv_map != None):
            print('bla')

            self.load_map()

            persons = self.personsGenerator.generatePersons()

            array_of_persons = []           
        

            for person in persons:
                print('draw----------')
                self.drawPerson(person)
                array_of_persons.append([person.position_m[0],
                    person.position_m[1],person.axes_length_a_m,person.axes_length_b_m, person.yaw_deg_angle])

            array_string = ';'.join(','.join(map(str, obj)) for obj in array_of_persons)

            # Create and publish the string message
            msg = String()
            msg.data = array_string
            self.persons_publisher.publish(msg)

            self.setImageViewer()
        else:
            print('222222222222')    

    def convert_meters_to_pix(self, value):

        new_val =     value / self.map_resolution  
        print(' new_val ' + str(new_val)) 
        return   new_val   

    def convert_pose_to_pix(self, pose):

        x = float(pose[0] - self.map_origin_position_x) / float(self.map_resolution)
        y = float(pose[1] - self.map_origin_position_y) / float(self.map_resolution)

        print('the x is  ' + str(x) + ' the y is ' + str(y))
        return (int(x),int(y))

    def drawPerson(self, person):
        center = self.convert_pose_to_pix((person.position_m[0],person.position_m[1]))
        axes = (int(self.convert_meters_to_pix(person.axes_length_a_m)),int(self.convert_meters_to_pix(person.axes_length_b_m)))
        angle = person.yaw_deg_angle
        startAngle = 0
        endAngle = 360
        color = (128, 0, 128)  # Green color

        cv2.ellipse(self.rgb_image, center, axes, angle, startAngle, endAngle, color, thickness=-1)
        
        # Length of the arrow
        arrow_length = self.convert_meters_to_pix(1.0)

        # Calculate the endpoint of the arrow
        end_point = (
            int(center[0] + arrow_length * math.cos(math.radians(person.yaw_deg_angle))),
            int(center[1] + arrow_length * math.sin(math.radians(person.yaw_deg_angle)))
        )
        cv2.arrowedLine(self.rgb_image, center, end_point, (255, 0, 0), thickness=2, tipLength=0.2)

    def destroyNode(self):
        self.destroy_node()
    
    def destroy_node(self):
        self._action_client_goal.destroy()
        self._action_client_goal_path.destroy()
        # self.resetNav2()
        self.cli.destroy()
        super().destroy_node()

# if __name__ == "__main__":
#     root = tk.Tk()
#     app = SocialNavigationUI(root)
#     root.mainloop()




# def main(args=None):
#     rclpy.init(args=args)
#     root = tk.Tk()
#     app = SocialNavigationUI(root)
#     root.mainloop()
    
  
       

# if __name__ == "__main__":
#     main()    

def main(args=None):
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor(4)
        app = SocialNavigationUI(executor)
        executor.add_node(app)
        flag = False
        try:
            #executor.spin()
            app.root.mainloop()
        except KeyboardInterrupt as e:
            executor.shutdown()
            app.destroyNode()
            flag = True
        finally:
            if not flag:
                executor.shutdown()
                app.destroyNode()
            
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()