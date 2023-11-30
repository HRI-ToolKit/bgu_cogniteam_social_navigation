import tkinter as tk
from tkinter import filedialog
from PIL import Image, ImageTk
import json
import cv2
import numpy as np
import yaml
import random
import math
import time

import ament_index_python
package_name = 'social_navigation_ui'
package_share_directory = ament_index_python.get_package_share_directory(package_name)
import rclpy
from rclpy.executors import MultiThreadedExecutor


from social_navigation_ui.ros_wrapper import RosWrapper
from social_navigation_ui.PersonsGenerator import PersonsGenerator
from social_navigation_ui.Person import Person
from social_navigation_ui.utils import* 


class SocialNavigationUI():
    
    def __init__(self, executor):  

        self.ros_wrapper = RosWrapper(executor)      

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
        self.persons = None
        
        self.robot_position = (-2.5,0.3) # take from the launch
        self.goal = None


        #map vlaues
        self.map_resolution = 0.0
        self.map_origin_position_x = 0.0
        self.map_origin_position_y = 0.0       


        self.update_map_button = tk.Button(self.root, text="Generate map with persons", command=self.setPersonsCmd)
        self.update_map_button.pack(pady=10)

        self.clear_cost_map_button = tk.Button(self.root, text="clear costmap", command=self.ros_wrapper.clearCostMap)
        self.clear_cost_map_button.pack(pady=10)

       
        self.image_viewer = tk.Label(self.root)
        self.image_viewer.pack(pady=10)      
         
        # Bind the left mouse button click event to the callback function
        self.image_viewer.bind("<Button-1>", self.on_mouse_click)

        self.load_map()   
    

    def set_path(self):
        
        path = self.ros_wrapper.calculatePath(self.goal)

        self.rgb_image = cv2.cvtColor(self.cv_map, cv2.COLOR_GRAY2RGB)
        
        if path != None:
            for i in range(len(path) - 1):
                pix_1 = convert_pose_to_pix((path[i].pose.position.x,path[i].pose.position.y),
                     self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
                pix_2 = convert_pose_to_pix((path[i+1].pose.position.x,path[i+1].pose.position.y),
                     self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
                cv2.line(self.rgb_image, pix_1,pix_2 , (0, 255, 200), 2)

        self.setImageViewer()


    def load_map(self):
        
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
        robot_pix = convert_pose_to_pix((self.robot_position[0], self.robot_position[1]), 
            self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
        
        print('robot_pix ' + str(robot_pix))
        cv2.circle(self.rgb_image,robot_pix , 5, (0,100,200), -1) 

        self.setImageViewer()

    def on_mouse_click(self, event):
        # Get the x and y coordinates of the mouse click
        x = event.x
        y = event.y

        # Get the RGB color of the clicked pixel from the image
        pixel_color = self.mouse_image.getpixel((x, y))
        print(f"Mouse clicked at (x={x}, y={y}), RGB color: {pixel_color}")

        self.goal = convert_pix_to_pose((x,y),  self.map_resolution, 
            self.map_origin_position_x, self.map_origin_position_y)

        self.set_path()

    def setImageViewer(self):

        self.mouse_image = Image.fromarray(self.rgb_image)
        tk_image = ImageTk.PhotoImage(self.mouse_image)
        self.image_viewer.configure(image=tk_image)
        self.image_viewer.image = tk_image  # Keep a reference to avoid garbage collection issues


    def setPersonsCmd(self):
        
        if np.all(self.cv_map != None):

            self.load_map()
            
            self.generatePersons()
            
            self.drawAndPublishPersons()

            self.setImageViewer()

    def generatePersons(self):        

        self.persons = self.personsGenerator.generatePersons()

    def drawAndPublishPersons(self):

        array_of_persons = []        

        for person in self.persons:
            self.drawPerson(person)
            array_of_persons.append([person.position_m[0],
                person.position_m[1],person.axes_length_a_m,person.axes_length_b_m, person.yaw_deg_angle])

        array_string = ';'.join(','.join(map(str, obj)) for obj in array_of_persons)

        self.ros_wrapper.publishPersons(array_string)

       

    def drawPerson(self, person):
        center = convert_pose_to_pix((person.position_m[0],person.position_m[1]), 
             self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
        
        axes_a = int(convert_meters_to_pix(person.axes_length_a_m, self.map_resolution))
        axes_b = int(convert_meters_to_pix(person.axes_length_b_m, self.map_resolution)) 
        
        angle = person.yaw_deg_angle
        startAngle = 0
        endAngle = 360
        color = (128, 0, 128)  # Green color

        cv2.ellipse(self.rgb_image, center, (axes_a, axes_b), angle,
             startAngle, endAngle, color, thickness=-1)
        
        # Length of the arrow
        arrow_length = convert_meters_to_pix(1.0, self.map_resolution)

        # Calculate the endpoint of the arrow
        end_point = (
            int(center[0] + arrow_length * math.cos(math.radians(person.yaw_deg_angle))),
            int(center[1] + arrow_length * math.sin(math.radians(person.yaw_deg_angle)))
        )
        cv2.arrowedLine(self.rgb_image, center, end_point, (255, 0, 0), thickness=2, tipLength=0.2)

    def destroyNode(self):
        self.ros_wrapper.destroy_node()
    
   
def main(args=None):
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor(4)
        app = SocialNavigationUI(executor)
        executor.add_node(app.ros_wrapper)
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