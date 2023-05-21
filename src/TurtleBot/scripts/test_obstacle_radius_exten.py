import cv2
# from rrt import map_img
import numpy as np

def extend_obstacles(my_map, radius):
    # Threshold the grayscale image to get the binary map

    _, binary_map = cv2.threshold(my_map, 127, 255, cv2.THRESH_BINARY_INV)

    # Calculate the structuring element size based on the radius
    structuring_element_size = int(2 * radius + 1)

    # Create a circular structuring element
    structuring_element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (structuring_element_size, structuring_element_size))

    # Dilate the binary map using the circular structuring element
    dilated_map = cv2.dilate(binary_map, structuring_element)

    # Invert the dilated map to get the extended obstacles
    extended_obstacles = cv2.bitwise_not(dilated_map)

    return my_map, binary_map, dilated_map, extended_obstacles


def map_img(arr):
    disp_map = np.ones((384,384))*255
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1 ]):
            if arr[i][j]==-1:
                disp_map[i][j] = 100
            if arr[i][j] == 100:
                disp_map[i][j] = 0
    im = np.array(disp_map, dtype = np.uint8)
    return im[::-1]


if __name__ == "__main__":
    
    map_in = cv2.imread('my_map.pgm')
    print(map_in)
    # map_in = cv2.cvtColor(map_img(map_in), cv2.COLOR_GRAY2BGR)[::-1]
    print(len(map_in))
    robot_radius = 0.4
    my_map, binary_map, dilated_map, extended_map = extend_obstacles(map_in, robot_radius)
    # my_map, binary_map, dilated_map = extend_obstacles(map_in, robot_radius)

    # cv2.imshow('My Map', map_img)

    # cv2.imshow('Extended Map', extended_map)
    
    combined_image = np.hstack((my_map,binary_map,dilated_map,extended_map))
    # combined_image = np.hstack((my_map,binary_map,dilated_map))
    cv2.imshow('Combined Image', combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

