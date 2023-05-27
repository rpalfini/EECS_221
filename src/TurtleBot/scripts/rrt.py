#!/usr/bin/env python2

import rospy
import numpy as np
from matplotlib import cm
import matplotlib.pyplot as plt
from scipy.misc import imread
import random, sys, math, os.path
import cv2
import PID_Controller as pid
import Motion_Planner as mp
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import pickle
import time
import os

dbg = True
MIN_NUM_VERT = 20 # Minimum number of vertex in the graph
MAX_NUM_VERT = 70000 # Maximum number of vertex in the graph
STEP_DISTANCE = 20 # Maximum distance between two vertex
SEED = None # For random numbers

class SubscriberNode_Map(pid.SubscriberNode):
  def __init__(self, topic, msg, msg_object):
    super(SubscriberNode_Map,self).__init__(topic, msg, msg_object)
    self.current_map = None
    self.prev_map = None

  def callback(self, data):
    super(SubscriberNode_Map,self).callback(data)
    self.prev_map = self.current_map
    self.current_map = self.convert_map_2D(data)

  def convert_map_2D(self,data):
    '''This function converts the 1D map data into 2D array'''
    self.height = data.info.height
    self.width = data.info.width
    map_out = np.zeros((self.height,self.width))
    current_row = 0
    current_col = 0
    for cell in data.data:
      if current_col == self.width:
        current_col = 0
        current_row += 1
      map_out[current_row,current_col] = cell
      current_col += 1  
    return map_out 

class node_obj(object):
    def __init__(self,node,parent):
        self.state = node #this should be (x,y) value
        self.parent = parent #this should refer to the parent node_obj

def is_present(target, lst):
    return target in lst

def BFS(graph,start):
    rospy.loginfo('starting BFS')
    frontier = []
    visited = []
    # find initial edge
    for node_edge in graph:
        if start == node_edge[0]:
            node2expand = node_edge
            break
    # add first level deep to initialize frontier
    start_node = node_obj(node2expand[0],None)
    visited.append(node2expand[0])

    for child in node2expand[1]:
        frontier.append(node_obj(child,start_node))
        
    while not len(frontier) == 0:
        node2explore = frontier.pop(0)
        visited.append(node2explore.state)
        if node2explore.state == graph[-1][0]: #check if node_edge is goal
            break
        for node_edge in graph:
            if node2explore.state == node_edge[0]:
                node2expand = node_edge
                break
        for child in node2expand[1]:
            if not is_present(child,visited):
                frontier.append(node_obj(child,node2explore))

    path = [node2explore.state]
    parent_node = node2explore.parent
    # while not parent_node.state == start:
    while not parent_node is None:
        node2explore = parent_node
        parent_node = node2explore.parent
        path.insert(0,node2explore.state)
    return path  

def rapidlyExploringRandomTree(img, start, goal, seed=None):
  rospy.loginfo('entered rapidlyExploringRandomTree')
  hundreds = 1000
  random.seed(seed)
  points = []
  graph = []
  points.append(start)
  graph.append((start, []))
  # print 'Generating and conecting random points'
  occupied = True
  phaseTwo = False
  # Phase two values (points 5 step distances around the goal point)
  minX = max(goal[0] - 5 * STEP_DISTANCE, 0)
  maxX = min(goal[0] + 5 * STEP_DISTANCE, len(img[0]) - 1)
  minY = max(goal[1] - 5 * STEP_DISTANCE, 0)
  maxY = min(goal[1] + 5 * STEP_DISTANCE, len(img) - 1)

  i = 0
  while (goal not in points) and (len(points) < MAX_NUM_VERT) and not rospy.is_shutdown():
    # if dbg:
    #   rospy.loginfo('entered goal not found loop')
    # if (i % 100) == 0:
    #   print i, 'points randomly generated'

    if (len(points) % hundreds) == 0:
      print(len(points), 'vertex generated')
      # hundreds = hundreds + 100
    if len(points) == 1:
       print('point length is 1, if message repeats, that means invalid start_goal location sent to rrt')
    while(occupied):
      if phaseTwo and (random.random() > 0.8):
        point = [ random.randint(minX, maxX), random.randint(minY, maxY) ]
      else:
        point = [ random.randint(0, len(img[0]) - 1), random.randint(0, len(img) - 1) ]

      if(img[point[1]][point[0]][0] > 250):
        occupied = False

    occupied = True

    nearest = findNearestPoint(points, point)
    newPoints = connectPoints(point, nearest, img)
    addToGraph(graph, newPoints, point)
    newPoints.pop(0) # The first element is already in the points list
    points.extend(newPoints)

    i = i + 1
    # if dbg:
    #   # pid.debug_info('length of points',p=len(points))
    #   rospy.loginfo('length of points %d' % (len(points)))
    
    if len(points) >= MIN_NUM_VERT:
      
      if not phaseTwo:
        print('Phase Two')
      phaseTwo = True

    if phaseTwo:
      nearest = findNearestPoint(points, goal)
      newPoints = connectPoints(goal, nearest, img)
      addToGraph(graph, newPoints, goal)
      newPoints.pop(0)
      points.extend(newPoints)

  if goal in points:
    print('Goal found, total vertex in graph:', len(points), 'total random points generated:', i)
    path = BFS(graph,start)
    if path is None:
       output_dbg_info('Is_None_search',graph=graph,goal=goal,path=path,points=points)
    print('Showing resulting map')
    print('Final path:', path)
    print('The final path is made from:', len(path),'connected points')

  else:
    path = None
    print('Reached maximum number of vertex and goal was not found')
    print('Total vertex in graph:', len(points), 'total random points generated:', i)
    print('Showing resulting map')
    # plot_traj_found()

  return path,graph

# def searchPath(graph, point, path):
#   for i in graph:
#     # pid.debug_info('for i in graph',i=i,graph=graph,graph_type=type(graph)) if dbg else None
#     if point == i[0]:
#       # pid.debug_info('point == i[0]',point=point,i_0=i[0],i_type = type(i)) if dbg else None
#       p = i

#   if p[0] == graph[-1][0]:
    
#       # pid.debug_info('searchPath',len_point=len(point),len_graph=len(graph))
#       # pid.debug_info('search_path_path',len_path=len(path),path=path)
#       # pid.debug_info('path type',ptype = type(path))
#     # rospy.loginfo('entered_early_return')
#     # output_dbg_info('searchPath_bad',graph=graph,point=point,path=path)
#     return path

#   for link in p[1]:
#     path.append(link)
#     finalPath = searchPath(graph, link, path)

#     if finalPath != None:
#     # if not finalPath is None:
#       rospy.loginfo('final path returned')
#       return finalPath
#     else:
#       path.pop()

def addToGraph(graph, newPoints, point):
  if len(newPoints) > 1: # If there is anything to add to the graph
    for p in range(len(newPoints) - 1):
      nearest = [ nearest for nearest in graph if (nearest[0] == [ newPoints[p][0], newPoints[p][1] ]) ]
      nearest[0][1].append(newPoints[p + 1])
      graph.append((newPoints[p + 1], []))

def connectPoints(a, b, img):
  newPoints = []
  newPoints.append([ b[0], b[1] ])
  step = [ (a[0] - b[0]) / float(STEP_DISTANCE), (a[1] - b[1]) / float(STEP_DISTANCE) ]

  # Set small steps to check for walls
  pointsNeeded = int(math.floor(max(math.fabs(step[0]), math.fabs(step[1]))))

  if math.fabs(step[0]) > math.fabs(step[1]):
    if step[0] >= 0:
      step = [ 1, step[1] / math.fabs(step[0]) ]
    else:
      step = [ -1, step[1] / math.fabs(step[0]) ]

  else:
    if step[1] >= 0:
      step = [ step[0] / math.fabs(step[1]), 1 ]
    else:
      step = [ step[0]/math.fabs(step[1]), -1 ]

  blocked = False
  for i in range(pointsNeeded+1): # Creates points between graph and solitary point
    for j in range(STEP_DISTANCE): # Check if there are walls between points
      coordX = round(newPoints[i][0] + step[0] * j)
      coordY = round(newPoints[i][1] + step[1] * j)

      if coordX == a[0] and coordY == a[1]:
        break
      if coordY >= len(img) or coordX >= len(img[0]):
        break
      if img[int(coordY)][int(coordX)][0] < 240:
        blocked = True
      if blocked:
        break

    if blocked:
      break
    if not (coordX == a[0] and coordY == a[1]):
      newPoints.append([ newPoints[i][0]+(step[0]*STEP_DISTANCE), newPoints[i][1]+(step[1]*STEP_DISTANCE) ])

  if not blocked:
    newPoints.append([ a[0], a[1] ])
  return newPoints

def findNearestPoint(points, point):
  best = (sys.maxint, sys.maxint, sys.maxint)
  for p in points:
    if p == point:
      continue
    dist = math.sqrt((p[0] - point[0]) ** 2 + (p[1] - point[1]) ** 2)
    if dist < best[2]:
      best = (p[0], p[1], dist)
  return (best[0], best[1])

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


def find_path_RRT(start,goal,my_map):
  global robot_radius
  global use_dilated_map
  my_map = cv2.cvtColor(map_img(my_map), cv2.COLOR_GRAY2BGR)[::-1]
  if use_dilated_map:
    _,_,_,my_map = extend_obstacles(my_map,robot_radius)
  path,graph = rapidlyExploringRandomTree(my_map, start, goal, seed=None)
  return path,graph

def map_img(arr):
    disp_map = np.ones((384,384))*255
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            if arr[i][j]==-1:
                disp_map[i][j] = 100
            if arr[i][j] == 100:
                disp_map[i][j] = 0
    im = np.array(disp_map, dtype = np.uint8)
    return im[::-1]

def get_index_from_coordinates(start_goal_data,map_node):
  
  # start_goal_data should be Float64MultiArray of length 4
  if not len(start_goal_data) == 4:
    raise Exception('expecting length of data to be 4 but got %d' % (len(start_goal_data)))
  start_real = start_goal_data[0:2]
  goal_real = start_goal_data[2:4]
  x_origin = map_node.data.info.origin.position.x
  y_origin = map_node.data.info.origin.position.y
  resolution = map_node.data.info.resolution

  x_start_index = convert_real_to_index(start_real[0],x_origin,resolution)
  y_start_index = convert_real_to_index(start_real[1],y_origin,resolution)
  x_goal_index = convert_real_to_index(goal_real[0],x_origin,resolution)
  y_goal_index = convert_real_to_index(goal_real[1],y_origin,resolution)

  start_goal_index = (x_start_index,y_start_index,x_goal_index,y_goal_index)
  return start_goal_index

def start_real_from_index(point_index,resolution,origin):
  return point_index*resolution + origin

def check_if_valid_input():
   pass

def arg_parse():
    args = {}
    args['plot_traj'] = rospy.get_param('~plot_traj',False)
    # print('my boolean param %s' % (args['plot_traj']))
    args['use_dilated_map'] = rospy.get_param('~use_dilated_map',True)
    default_map = os.path.dirname(os.path.abspath(__file__)) + "/../../../my_map.pgm"
    print('loading map from: %s' % (default_map))
    args['image_path'] = rospy.get_param('~image_path',default_map)
    args['robot_radius'] = rospy.get_param('~robot_radius',3.7)
    
    return args

def main():
  global robot_radius
  global use_dilated_map

  rospy.init_node('RRT_node')

  # load args
  args = arg_parse()
  image_path = args['image_path']
  robot_radius = args['robot_radius']

  # set up subscriptions
  map_node = SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
  start_goal_node = pid.SubscriberNode('/start_goal',Float64MultiArray,Float64MultiArray())

  # set up publishers
  traj_pub = rospy.Publisher('/trajectory',Float64MultiArray,queue_size=5)

  # option
  plot_traj = args['plot_traj'] 
  use_dilated_map = args['use_dilated_map']

  # flags
  is_first = True
  is_traj_computed = False
  is_first_point_rec = False
  last_rec_point = []
  is_map_loaded = False
  # flag to make sure RRT isn't activated until a valid start goal is received
  while not is_first_point_rec and not rospy.is_shutdown():
    is_first = mp.status_msg('Waiting for first start_goal',is_first)
    if not start_goal_node.data.data == []:
      if check_if_valid_input(start_goal_node.data.data):
        is_first_point_rec = True
        cur_start_goal = start_goal_node.data.data
      else:
        if start_goal_node.data.data != last_rec_point:
          rospy.loginfo('invalid start_goal received (%.2f,%.2f)_(%.2f,%.2f)' % (start_goal_node.data.data[0],start_goal_node.data.data[1],start_goal_node.data.data[2],start_goal_node.data.data[3]))
          last_rec_point = start_goal_node.data.data
          
  rospy.loginfo('first start_goal received (%.2f,%.2f)_(%.2f,%.2f)' % (cur_start_goal[0],cur_start_goal[1],cur_start_goal[2],cur_start_goal[3]))
  # make sure map is loaded
  is_first = True
  while not is_map_loaded and not rospy.is_shutdown():
    is_first = mp.status_msg('Waiting for map load',is_first)
    if not map_node.current_map is None:
      is_map_loaded = True

  is_first = True
  while not rospy.is_shutdown():
    if not is_traj_computed:
      rospy.loginfo('searching for trajectory')
      start_goal_index = get_index_from_coordinates(cur_start_goal,map_node)
      rospy.loginfo(start_goal_index)
      # path,graph = find_path_RRT(cv2.cvtColor(map_img(map_node.current_map),cv2.COLOR_GRAY2BGR)[::-1],start_goal_map[0:2],start_goal_map[2:4])
      start = [start_goal_index[0],start_goal_index[1]]
      goal = [start_goal_index[2],start_goal_index[3]]
      path,graph = find_path_RRT(start,goal,map_node.current_map)
      pid.debug_info('rrt',path_type=type(path),graph_type=type(graph),len_path=len(path)) if dbg else None
      print('path ' + str(len(path)))
      rospy.loginfo('showing path')
      for item in path:
        print(item)
  
      path_msg = create_path_msg(path,map_node.data.info)
      traj_pub.publish(path_msg)
      is_traj_computed = True
      rospy.loginfo('trajectory found and outputted')
      if plot_traj:
        rospy.loginfo('Plotting trajectory')
        # plot_traj_found_extended_map(path)
        plot_traj_found(path,image_path,map_node.data.info)

    if not cur_start_goal == start_goal_node.data.data:
      rospy.loginfo('new start_goal received (%.2f,%.2f,%.2f,%.2f), old_goal is (%.2f,%.2f,%.2f,%.2f)' % (start_goal_node.data.data[0],start_goal_node.data.data[1],start_goal_node.data.data[2],start_goal_node.data.data[3],cur_start_goal[0],cur_start_goal[1],cur_start_goal[2],cur_start_goal[3]))
      cur_start_goal = start_goal_node.data.data
      is_traj_computed = False

def create_path_msg(path,map_info,convert_to_real=True):
  path_msg = Float64MultiArray()
  path_msg.data = [coord for point in path for coord in point]
  if convert_to_real:
    path_msg.data = convert_traj_to_real(path_msg.data,map_info)
  return path_msg

def convert_traj_to_real(traj_list,map_info):
    real_traj = []
    is_x_coord = True
    resolution = map_info.resolution
    for point in traj_list:
        if is_x_coord:
           origin = map_info.origin.position.x
           is_x_coord = False
        else:
           origin = map_info.origin.position.y
           is_x_coord = True
        real_traj.append(convert_index_to_real(point,origin,resolution))
    return real_traj

def convert_index_to_real(index,origin,resolution):
    return index*resolution + origin

def convert_real_to_index(real,origin,resolution):
    # return origin + int(round(real/resolution)) #This looks incorrect...
    point = abs(real-origin)
    return int(round(point/resolution))

def plot_traj_found(path,image_path,map_info):
  height = map_info.height
  plot_both = True
  if plot_both:
    # plt.clf()
    # plt.pause(0.5)
    fig = plt.figure()
    # plt.ion()
    ax1 = fig.add_subplot(1,2,1)
    img = imread(image_path)
    # img = imread('/my_map.pgm')
    _,_,_,extended_map = extend_obstacles(img, robot_radius)
    ax1.imshow(extended_map,cmap=cm.gray)
    ax1.axis('off')
    x_coords = [point[0] for point in path]
    y_coords = [height-point[1] for point in path]
    ax1.plot(x_coords,y_coords,color='red',linewidth=2)
    ax1.set_title('Configuration Space')

    ax2 = fig.add_subplot(1,2,2)
    img = imread(image_path)
    ax2.imshow(img,cmap=cm.gray)
    ax2.axis('off')
    x_coords = [point[0] for point in path]
    y_coords = [height-point[1] for point in path]
    ax2.plot(x_coords,y_coords,color='red',linewidth=2)
    ax2.set_title('Map Space')
    plt.show()

  else:
    img = imread(image_path)
    plt.imshow(img,cmap=cm.gray)
    plt.axis('off')
    x_coords = [point[0] for point in path]
    y_coords = [384-point[1] for point in path]
    plt.plot(x_coords,y_coords,color='red',linewidth=2)
    plt.show()

def plot_traj_found_extended_map(path,image_path):
  global robot_radius
  plt.figure()
  img = imread(image_path)
  _,_,_,extended_map = extend_obstacles(img, robot_radius)
  plt.imshow(extended_map,cmap=cm.gray)
  plt.axis('off')
  x_coords = [point[0] for point in path]
  y_coords = [384-point[1] for point in path]
  plt.plot(x_coords,y_coords,color='red',linewidth=2)
  plt.show()
   

def output_dbg_info(fheader,**kwargs):
  current_time = time.strftime("%Y%m%d%H%M%S")
  file_name = fheader + '_' + current_time + '.pkl'
  with open(file_name, 'wb') as file:
    # Write the data to the file using pickle.dump()
    pickle.dump(kwargs, file)

if __name__ == "__main__":
  main()
