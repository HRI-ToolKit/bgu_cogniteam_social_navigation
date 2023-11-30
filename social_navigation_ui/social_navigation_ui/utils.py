def convert_meters_to_pix(value, map_resolution):

    new_val =     value / map_resolution  
    return   new_val   


def convert_pix_to_pose(pix, map_resolution, map_origin_position_x, map_origin_position_y):

    x_pose = (pix[0]*float(map_resolution)) + map_origin_position_x
    y_pose = (pix[1]*float(map_resolution)) + map_origin_position_y

    print('x_pose ' + str(x_pose) + ' y_pose ' + str(y_pose))
    return (float(x_pose),float(y_pose))   

def convert_meters_pix_to_pose( x_y, map_resolution, map_origin_position_x, map_origin_position_y):

    x_pose =  (x_y[0] * map_resolution) + map_origin_position_x
    y_pose =  (x_y[1] * map_resolution) + map_origin_position_y

    return (x_pose, y_pose)

def convert_pix_to_meters( pix_l, map_resolution):

    meter =     pix_l * map_resolution  
    return   meter  


def convert_pose_to_pix( pose,map_resolution, map_origin_position_x, map_origin_position_y):

    x = float(pose[0] - map_origin_position_x) / float(map_resolution)
    y = float(pose[1] - map_origin_position_y) / float(map_resolution)

    return (int(x),int(y))    

  


