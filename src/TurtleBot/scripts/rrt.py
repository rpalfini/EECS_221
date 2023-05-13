#!/usr/bin/env python2

import rospy
import numpy as np
from matplotlib import cm
from scipy.misc import imread
import random, sys, math, os.path
import cv2
import PID_Controller as pid
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray

MIN_NUM_VERT = 20 # Minimum number of vertex in the graph
MAX_NUM_VERT = 1500 # Maximum number of vertex in the graph
STEP_DISTANCE = 20 # Maximum distance between two vertex
SEED = None # For random numbers

class SubscriberNode_Map(pid.SubscriberNode):
  def __init__(self, topic, msg, msg_object):
    super(SubscriberNode_Map,self).__init__(topic, msg, msg_object)
    self.current_map = None

  def callback(self, data):
    super(SubscriberNode_Map,self).callback(data)
    rospy.loginfo('callback occurs')
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

def rapidlyExploringRandomTree(img, start, goal, seed=None):
  hundreds = 100
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
  while (goal not in points) and (len(points) < MAX_NUM_VERT):
    # if (i % 100) == 0:
    #   print i, 'points randomly generated'

    if (len(points) % hundreds) == 0:
      # print len(points), 'vertex generated'
      hundreds = hundreds + 100

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

    if len(points) >= MIN_NUM_VERT:
      if not phaseTwo:
        print 'Phase Two'
      phaseTwo = True

    if phaseTwo:
      nearest = findNearestPoint(points, goal)
      newPoints = connectPoints(goal, nearest, img)
      addToGraph(graph, newPoints, goal)
      newPoints.pop(0)
      points.extend(newPoints)


  if goal in points:
    # print 'Goal found, total vertex in graph:', len(points), 'total random points generated:', i
    path = searchPath(graph, start, [start])
    # print 'Showing resulting map'
    # print 'Final path:', path
    # print 'The final path is made from:', len(path),'connected points'
  else:
    path = None
    print 'Reached maximum number of vertex and goal was not found'
    print 'Total vertex in graph:', len(points), 'total random points generated:', i
    print 'Showing resulting map'


  return path,graph

def searchPath(graph, point, path):
  for i in graph:
    if point == i[0]:
      p = i

  if p[0] == graph[-1][0]:
    return path

  for link in p[1]:
    path.append(link)
    finalPath = searchPath(graph, link, path)

    if finalPath != None:
      return finalPath
    else:
      path.pop()

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

def find_path_RRT(start,goal,my_map):
  my_map = cv2.cvtColor(map_img(my_map), cv2.COLOR_GRAY2BGR)[::-1]
  path = rapidlyExploringRandomTree(my_map, start, goal, seed=None)
  return path

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
  def convert_to_map_coord(real,origin,resolution):
    return origin + int(round(real/resolution))
  
  # start_goal_data should be Float64MultiArray of length 4
  if not len(start_goal_data) == 4:
    raise Exception('expecting length of data to be 4 but got %d' % (len(start_goal_data)))
  start_real = start_goal_data[0:2]
  goal_real = start_goal_data[2:4]
  x_origin = map_node.data.info.origin.position.x
  y_origin = map_node.data.info.origin.position.y
  resolution = map_node.data.info.resolution

  x_start_index = convert_to_map_coord(start_real[0],x_origin,resolution)
  y_start_index = convert_to_map_coord(start_real[1],y_origin,resolution)
  x_goal_index = convert_to_map_coord(goal_real[0],x_origin,resolution)
  y_goal_index = convert_to_map_coord(goal_real[1],y_origin,resolution)

  start_goal_index = (x_start_index,y_start_index,x_goal_index,y_goal_index)
  return start_goal_index

def main():
  rospy.init_node('RRT_node')
  # set up subscriptions
  map_node = SubscriberNode_Map('/map',OccupancyGrid,OccupancyGrid())
  start_goal_node = pid.SubscriberNode('/start_goal',Float64MultiArray,Float64MultiArray())

  # set up publishers
  traj_pub = rospy.Publisher('/trajectory',Float64MultiArray,queue_size=5)

  # flags
  is_traj_computed = False
  is_first_point_rec = False
  # flag to make sure RRT isn't activated until a start goal is received
  while not is_first_point_rec and not rospy.is_shutdown():
    if not start_goal_node.data == []:
      is_first_point_rec = True
      cur_start_goal = start_goal_node.data

  while not rospy.is_shutdown():
    if not is_traj_computed:
      start_goal_map = get_index_from_coordinates(cur_start_goal,map_node)
      path,graph = find_path_RRT(cv2.cvtColor(map_img(map_node.current_map),cv2.COLOR_GRAY2BGR)[::-1],start_goal_map[0:2],start_goal_map[2:4])
      traj_pub.publish(path)
      is_traj_computed = True
    else:
      if not cur_start_goal == start_goal_node.data:
        is_traj_computed = False
      

if __name__ == "__main__":
  main()
