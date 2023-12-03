import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox 

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

        self.robot_radius_m = 0.3
        self.personsGenerator = None 
        self.num_of_persons = 1
        self.persons = None
        
        self.robot_position = (-5.0,7.0) # take from the launch
        self.goal = None

        #map vlaues
        self.map_resolution = 0.0
        self.map_origin_position_x = 0.0
        self.map_origin_position_y = 0.0      


        self.update_map_button = tk.Button(self.root, text="Generate map with persons", command=self.setPersonsCmd)
        self.update_map_button.pack(pady=10)       
       
        self.image_viewer = tk.Label(self.root)
        self.image_viewer.pack(pady=10)      
         
        # Bind the left mouse button click event to the callback function
        self.image_viewer.bind("<Button-1>", self.on_mouse_click)

        self.load_map()   

        self.drawRobotPosition()

        self.setImageViewer()
    

    def set_path(self):
        
        path = self.ros_wrapper.calculatePath(self.goal)

        self.rgb_image = cv2.cvtColor(self.cv_map, cv2.COLOR_GRAY2RGB)

        if self.robot_position != None:
            self.drawRobotPosition()

        if self.persons != None:
            self.drawPersons()
        
        if path != None:
            for i in range(len(path) - 1):
                pix_1 = convert_pose_to_pix((path[i].pose.position.x,path[i].pose.position.y),
                     self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
                pix_2 = convert_pose_to_pix((path[i+1].pose.position.x,path[i+1].pose.position.y),
                     self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
                # cv2.circle(self.rgb_image, pix_1 , int(convert_meters_to_pix(self.robot_radius_m, 
                #     self.map_resolution)), (0,100,0), 1) 
                cv2.line(self.rgb_image, pix_1,pix_2 , (0, 255, 200), int(convert_meters_to_pix(self.robot_radius_m, 
                    self.map_resolution)))


        else:
            messagebox.showerror("error", "path is invalid") 

        self.setImageViewer()

    def drawRobotPosition(self):

        #robot position        
        robot_pix = convert_pose_to_pix((self.robot_position[0], self.robot_position[1]), 
            self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
        
        print('robot_pix ' + str(robot_pix))
        cv2.circle(self.rgb_image,robot_pix , int(convert_meters_to_pix(self.robot_radius_m,
             self.map_resolution)), (0,100,200), -1) 

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

            self.drawRobotPosition()
            
            self.generatePersons()
            
            self.drawPersons()

            #self.publishPersons()

            self.ros_wrapper.clearCostMap()

            self.setImageViewer()

    def generatePersons(self):        

        self.persons = self.personsGenerator.generatePersons()

    def drawPersons(self):

        array_of_persons = []        

        for person in self.persons:
            self.drawPerson(person)      

    def publishPersons(self):

        array_of_persons = []        

        for person in self.persons:
            array_of_persons.append([person.position_m[0],
                person.position_m[1],person.axes_length_a_m,person.axes_length_b_m, person.yaw_deg_angle])

        array_string = ';'.join(','.join(map(str, obj)) for obj in array_of_persons)

        self.ros_wrapper.publishPersons(array_string)


    def drawPerson(self, person):
        center = convert_pose_to_pix((person.position_m[0],person.position_m[1]), 
             self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
        
        points_m = person.getPoints()
        pixel_points = []
        for pointM in points_m:
            if math.isnan(pointM[0]) or math.isnan(pointM[1]):
                continue

            pix = convert_pose_to_pix((pointM[0],pointM[1]), 
             self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
            pixel_points.append(((pix[0]),(pix[1])))

        # Convert the pixel list to NumPy array
        pts = np.array(pixel_points, np.int32)

        # Reshape the array into a 2D array
        pts = pts.reshape((-1, 1, 2))

        # Draw the polygon on the image
        cv2.fillPoly(self.rgb_image, [pts], color=(147,112,219))
        
        # Length of the arrow
        arrow_length = convert_meters_to_pix(1.0, self.map_resolution)

        # Calculate the endpoint of the arrow
        end_point = (
            int(center[0] + arrow_length * math.cos(math.radians(person.yaw_deg_angle))),
            int(center[1] + arrow_length * math.sin(math.radians(person.yaw_deg_angle)))
        )
        cv2.arrowedLine(self.rgb_image, center, end_point, (255, 0, 0), thickness=2, tipLength=0.2)

        cv2.circle(self.rgb_image,center , int(convert_meters_to_pix(self.robot_radius_m,
             self.map_resolution)), (139,0,139), -1) 

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

# def main(args=None):
#     rclpy.init(args=args)
#     # Load the grayscale image
#     img = cv2.imread('/home/yakir/bgu_ws/src/bgu_cogniteam_social_navigation/social_navigation_ui/map.pgm', cv2.IMREAD_GRAYSCALE)

#     # Check if the image is loaded successfully
#     if img is None:
#         print("Error: Unable to load the image.")
        

#     # Change all pixels with intensity 255 to 254
#     img[(img != 254) & (img != 0)] = 254

#     # Save the modified image as a PGM file
#     cv2.imwrite('/home/yakir/bgu_ws/src/bgu_cogniteam_social_navigation/social_navigation_ui/map.pgm', img)



# if __name__ == "__main__":
#     main()


    
