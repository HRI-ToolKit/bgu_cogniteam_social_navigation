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

package_name = 'social_navigation_ui'
package_share_directory = ament_index_python.get_package_share_directory(package_name)

# ros2 run nav2_map_server map_server --ros-args --params-file /home/yakir/bgu_cogniteam_social_navigation_ws/src/social_navigation_ui/map_server_params.yaml 
# ros2 lifecycle set /map_server configure 
# ros2 lifecycle set /map_server activate



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

class JsonImageLoaderApp(Node):
    def __init__(self, root):
        self.root = root
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

        # Create GUI components
        # self.load_json_button = tk.Button(root, text="Load JSON", command=self.load_json)
        # self.load_json_button.pack(pady=10)

        self.load_map_button = tk.Button(root, text="Initialize Map", command=self.load_map)
        self.load_map_button.pack(pady=10)

        self.update_map_button = tk.Button(root, text="Generate map with persons", command=self.update_map)
        self.update_map_button.pack(pady=10)

        self.image_viewer = tk.Label(root)
        self.image_viewer.pack(pady=10)

    def load_json(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON files", "*.json")])
        if file_path:
            with open(file_path, 'r') as json_file:
                self.json_data = json.load(json_file)
                # You can use self.json_data as needed

    def load_map(self):
        #file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.png;*.jpg;*.jpeg;*.gif")])
        #if file_path:
        self.image_path = package_share_directory+'/map.pgm'
        self.cv_map = cv2.imread(self.image_path, 0)

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

        self.setImageViewer()
        
    def setImageViewer(self):

        tk_image = ImageTk.PhotoImage(Image.fromarray(self.rgb_image))
        self.image_viewer.configure(image=tk_image)
        self.image_viewer.image = tk_image  # Keep a reference to avoid garbage collection issues


    def update_map(self):
        if np.all(self.cv_map != None):
            print('bla')

            persons = self.personsGenerator.generatePersons()

            for person in persons:
                print('draw----------')
                self.drawPerson(person)

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


if __name__ == "__main__":
    root = tk.Tk()
    app = JsonImageLoaderApp(root)
    root.mainloop()




def main(args=None):
    rclpy.init(args=args)
    root = tk.Tk()
    app = JsonImageLoaderApp(root)
    root.mainloop()
    
  
       

if __name__ == "__main__":
    main()    
