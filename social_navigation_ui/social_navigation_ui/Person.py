

class Person:
    
    def __init__(self, map_resolution):
        
        self.map_resolution = map_resolution

        self.position_m = (0.0,0.0)

        #https://en.wikipedia.org/wiki/Ellipse
        self.axes_length_a_m = 0.5
        self.axes_length_b_m = 0.2
        self.yaw_deg_angle = 0.0

        