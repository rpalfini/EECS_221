#!/usr/bin/env python2

class node_obj(object):
    def __init__(self,node,parent):
        self.state = node #this should be (x,y) value
        self.parent = parent #this should refer to the parent node_obj

def is_present(target, lst):
    return target in lst

def myBFS(graph,start):
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