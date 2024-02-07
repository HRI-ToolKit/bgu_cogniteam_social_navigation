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
        
        # self.ros_wrapper.declare_parameter('initial_pose_x', 0.0)
        # initial_pose_x = self.ros_wrapper.get_parameter('initial_pose_x').get_parameter_value().double_value 
        # self.ros_wrapper.declare_parameter('initial_pose_y', 0.0)
        # initial_pose_y = self.ros_wrapper.get_parameter('initial_pose_y').get_parameter_value().double_value 

        self.robot_position = self.ros_wrapper.get_initial_x_y()#(initial_pose_x,initial_pose_y, 0) # take from the launch

        self.ros_wrapper.declare_parameter('map', '')
        self.map_path = self.ros_wrapper.get_parameter('map').get_parameter_value().string_value 


        self.goal = None

        #map vlaues
        self.map_resolution = 0.0
        self.map_origin_position_x = 0.0
        self.map_origin_position_y = 0.0      


        # self.update_map_button = tk.Button(self.root, text="Generate map with persons", command=self.setPersonsCmd)
        # self.update_map_button.pack(pady=10) 

        self.update_map_button = tk.Button(self.root, text="upload persons", command=self.uploadPersonsCmd)
        self.update_map_button.pack(pady=10)

        self.update_robot_position_button = tk.Button(self.root, text="save robot position", command=self.saveRobotPositionCmd)
        self.update_robot_position_button.pack(pady=10)

        # Create a text field (Entry widget)
        self.robot_location_text_field = tk.Entry(self.root, width=10)
        self.robot_location_text_field.pack()       
       
        self.image_viewer = tk.Label(self.root)
        self.image_viewer.pack(pady=10)      
         
        # Bind the left mouse button click event to the callback function
        self.image_viewer.bind("<Button-1>", self.on_mouse_click)

        self.load_map()   

        self.drawRobotPosition()

        self.setImageViewer()
    

    def set_path(self):
        
        self.ros_wrapper.set_robot_x_y(self.robot_position[0], self.robot_position[1])

        path = self.ros_wrapper.calculatePath(self.goal)

        self.rgb_image = cv2.cvtColor(self.cv_map, cv2.COLOR_GRAY2RGB)

        if self.robot_position != None:
            self.drawRobotPosition()

        if self.persons != None:
            self.drawPersons()
        
        if path != None and self.goal_pix != None:
            for i in range(len(path) - 1):
                pix_1 = convert_pose_to_pix((path[i].pose.position.x,path[i].pose.position.y),
                     self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
                pix_2 = convert_pose_to_pix((path[i+1].pose.position.x,path[i+1].pose.position.y),
                     self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
                # cv2.circle(self.rgb_image, pix_1 , int(convert_meters_to_pix(self.robot_radius_m, 
                #     self.map_resolution)), (0,100,0), 1) 
                cv2.line(self.rgb_image, pix_1,pix_2 , (0, 255, 200), int(convert_meters_to_pix(self.robot_radius_m, 
                    self.map_resolution)))

            goal_x_str = "{:.2f}".format(round(self.goal[0]), 2) 
            goal_y_str = "{:.2f}".format(round(self.goal[1]), 2) 

            self.ros_wrapper.get_logger().info(f"goal_x_str: {goal_x_str},goal_y_str: {goal_y_str}")

            
            cv2.putText(self.rgb_image, '('+goal_x_str+','+goal_y_str+')' , tuple(self.goal_pix), 1, 1, (0,0,0), 2)
    


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
        
        self.image_path = self.map_path+'/map.pgm'
        self.cv_map = cv2.imread(self.image_path, 0)
        self.cv_map = cv2.flip(self.cv_map, 0)

        map_yaml_path = self.map_path+'/map.yaml'
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
        self.ros_wrapper.set_robot_x_y(self.robot_position[0], self.robot_position[1])

        # Get the x and y coordinates of the mouse click
        x = event.x
        y = event.y

        # Get the RGB color of the clicked pixel from the image
        pixel_color = self.mouse_image.getpixel((x, y))
        print(f"Mouse clicked at (x={x}, y={y}), RGB color: {pixel_color}")

        self.goal_pix = [x,y]
        self.goal = convert_pix_to_pose((x,y),  self.map_resolution, 
            self.map_origin_position_x, self.map_origin_position_y)

        self.set_path()

    def setImageViewer(self):

        self.mouse_image = Image.fromarray(self.rgb_image)
        tk_image = ImageTk.PhotoImage(self.mouse_image)
        self.image_viewer.configure(image=tk_image)
        self.image_viewer.image = tk_image  # Keep a reference to avoid garbage collection issues


    def saveRobotPositionCmd(self):
        try:
            entered_text = self.robot_location_text_field.get()
            
            x_y_str = entered_text.split(',')
            self.robot_position = (float(x_y_str[0]), float(x_y_str[1]))
            
            self.ros_wrapper.get_logger().info(f"{self.robot_position}")
            
            self.ros_wrapper.set_robot_x_y(self.robot_position[0], self.robot_position[1])

            self.load_map()

            self.drawRobotPosition()

            if self.persons != None:
                self.drawPersons()

            self.setImageViewer()

            
        except ValueError as e:
            messagebox.showerror("error", "invalid robot's x,y values") 

    def uploadPersonsCmd(self):

        if np.all(self.cv_map != None):            
           

            # Open a file dialog to select a JSON file
            file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])

            # Check if a file was selected
            if file_path:
                try:

                    try:
                        # Open the selected JSON file and parse its content
                        with open(file_path, 'r') as file:
                            json_data = json.load(file)
                            # Access the 'persons' list in the parsed JSON data
                        persons_list = json_data.get("persons", [])

                        # Iterate through each person in the list
                        self.persons = []
                        for person in persons_list:

                            position_x = float(person.get("position_x", 0))
                            position_y = float(person.get("position_y", 0))
                            yaw_deg_angle = float(person.get("yaw_deg_angle", 0))
                            right_x = float(person.get("left_x", 0)) # becaus of flip issue left is right and right is left
                            left_x = float(person.get("right_x", 0))
                            forward_y = float(person.get("forward_y", 0))
                            backward_y = float(person.get("backward_y", 0))

                            person = Person((position_x, position_y), yaw_deg_angle, 
                                self.map_resolution, right_x, left_x, forward_y, backward_y )

                            self.persons.append(person)
                            # Do something with the extracted data
                            self.ros_wrapper.get_logger().info(f"Person: {position_x}, {position_y}, {yaw_deg_angle}, {right_x}, {left_x}, {forward_y}, {backward_y}")
                    except json.JSONDecodeError as e:
                        # Handle JSON decoding errors
                        messagebox.showerror("error", "error with Json file data") 
                        return

                    self.load_map()

                    self.drawRobotPosition()

                    self.drawPersons()

                    self.publishPersons()

                    self.ros_wrapper.clearCostMap()

                    self.setImageViewer()
                
                
                except Exception as e:
                    # Handle any exceptions that may occur during file reading or JSON parsing
                    print(f"Error: {e}")
                    self.ros_wrapper.get_logger().info(e)
            else:
                self.ros_wrapper.get_logger().info('44444444444444444444')


          

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
            points = person.getPoints()
            for p in points:
                p_str = (str(p[0])+'_'+str(p[1])).strip("'")
                array_of_persons.append(p_str)


        self.ros_wrapper.publishPersons(str(array_of_persons))


    def drawPerson(self, person):
        center = convert_pose_to_pix((person.position[0],person.position[1]), 
             self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)
        
        points_m = person.getPoints()
        pixel_points = []
        for pointM in points_m:
            # self.ros_wrapper.get_logger().info(str(pointM))

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
        # cv2.fillPoly(self.rgb_image, [pts], color=(147,112,219))

        height, width, channels = self.rgb_image.shape
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.fillPoly(mask, [pts], color=255)
        filled_points = np.column_stack(np.where(mask != 0))
        
        filled_points_m = []
        for p in filled_points:
            y, x = p
            # Draw a green circle (you can customize the radius and thickness)            
            filled_points_m.append(convert_pix_to_pose((x,y),  self.map_resolution, 
                self.map_origin_position_x, self.map_origin_position_y))
            self.rgb_image[y, x] = (147,112,219)

        person.setPoints(filled_points_m)      


        # Length of the arrow
        arrow_length = convert_meters_to_pix(1.0, self.map_resolution)

        # Calculate the endpoint of the arrow
        end_point = (
            int(center[0] + arrow_length * math.cos(math.radians(person.yaw_deg_angle + 90))),
            int(center[1] + arrow_length * math.sin(math.radians(person.yaw_deg_angle + 90)))
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

    
