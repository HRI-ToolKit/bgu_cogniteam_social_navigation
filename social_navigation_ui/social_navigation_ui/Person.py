from social_navigation_ui.Oval import Oval


class Person:
    
    def __init__(self, position, yaw_deg_angle,  map_resolution):
        
        self.map_resolution = map_resolution

        self.position = position

        self.yaw_deg_angle = yaw_deg_angle

        self.position_m = (0.0,0.0)

        self.oval = Oval(self.position_m, )

      

