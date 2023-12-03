import cv2
import numpy as np
import yaml
import random
import math
import time

from social_navigation_ui.utils import* 
from social_navigation_ui.Person import Person


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
                    valid_positions.append(convert_meters_pix_to_pose((x, y), 
                        self.map_resolution, self.map_origin_position_x, self.map_origin_position_y))

        count = 0
        while True:

            if count >= self.num_of_persons:
                break
            
            # r_x  = 0.25
            # l_x = -0.7
            # f_y = 1.0
            # b_y = -0.5

            # r_x  = 0.5
            # l_x = -0.5
            # f_y = 1.0
            # b_y = -1.0

            r_x  = 0.7
            l_x = -0.3
            f_y = 1.0
            b_y = 0.0

            yaw = self.generate_random_yaw()
            position = self.generate_position(valid_positions)

            person = Person(position,
                yaw, 
                self.map_resolution,
                r_x, l_x,f_y, b_y )
            

            if self.isPersonsOnMap(person):
                persons.append(person)
           
            
            count+= 1

        return persons


    def generate_position(self, valid_positions):

        index = random.randint(0, len(valid_positions))
        x_pose, y_pose = valid_positions[index]
        return (x_pose, y_pose)

    def generate_random_yaw(self):
        yaw_value = np.random.uniform(0, 360)
        return yaw_value

    def isPersonsOnMap(self, person):
        height, width = self.cv_map.shape[:2]

        person_x_pix, person_y_pix  = convert_pose_to_pix(person.position_m, 
             self.map_resolution, self.map_origin_position_x, self.map_origin_position_y)

        if person_x_pix < 0 or person_x_pix > width or person_y_pix < 0 or person_y_pix > height:
            return False
        
        pixel_value = self.cv_map[person_y_pix, person_x_pix]

        if pixel_value != 254:
            return False

        return True

