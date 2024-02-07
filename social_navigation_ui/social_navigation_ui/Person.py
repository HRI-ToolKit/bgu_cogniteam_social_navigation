from social_navigation_ui.Oval import Oval


class Person:
    
    def __init__(self, position, yaw_deg_angle, map_resolution,  r_x , l_x, f_y, b_y):
        
        self.position = position

        self.yaw_deg_angle = yaw_deg_angle

        self.map_resolution = map_resolution

        self.oval = Oval(self.position, self.yaw_deg_angle, r_x, l_x, f_y, b_y, self.map_resolution )

        self.is_oval_good = False

    def getPoints(self):
        
        ovalPoints = self.oval.getOvalPoints()  
        print('the ovalPoints lllll is '+ str(len(ovalPoints)))

        return ovalPoints
        
    def setFilledPoints(self, points):

        self.oval.setOvalPoints(points)
        self.is_oval_good = True
        

