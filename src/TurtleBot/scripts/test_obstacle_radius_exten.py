import cv2
import rrt
import numpy as np

def extend_obstacles(my_map, radius):
    # Convert the map to grayscale
    # my_map = cv2.cvtColor(my_map, cv2.COLOR_BGR2GRAY)
    my_map = cv2.cvtColor(rrt.map_img(my_map), cv2.COLOR_GRAY2BGR)[::-1]
    # Threshold the grayscale image to get the binary map
    _, binary_map = cv2.threshold(my_map, 127, 255, cv2.THRESH_BINARY)

    # Calculate the structuring element size based on the radius
    structuring_element_size = int(2 * radius + 1)

    # Create a circular structuring element
    structuring_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (structuring_element_size, structuring_element_size))

    # Dilate the binary map using the circular structuring element
    dilated_map = cv2.dilate(binary_map, structuring_element)

    # Invert the dilated map to get the extended obstacles
    extended_obstacles = cv2.bitwise_not(dilated_map)

    # Convert the extended obstacles to color format
    extended_obstacles_color = cv2.cvtColor(extended_obstacles, cv2.COLOR_GRAY2BGR)

    return my_map, binary_map, dilated_map, extended_obstacles_color





if __name__ == "__main__":
    
    map_img = cv2.imread('my_map.pgm')

    robot_radius = 0.5
    my_map, binary_map, dilated_map, extended_map = extend_obstacles(map_img, robot_radius)

    # cv2.imshow('My Map', map_img)

    # cv2.imshow('Extended Map', extended_map)
    
    combined_image = np.hstack((my_map,binary_map,dilated_map,extended_map))
    cv2.imshow('Combined Image', combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()