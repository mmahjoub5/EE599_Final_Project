import numpy as np

from controller import Supervisor

sizes = {
    "road_1": (21.5, 69),
    "road_3": (21.5, 69),
    "road_5": (21.5, 30),
    "road_7": (21.5, 30),
    "road_8": (21.5, 30),
    "road_10": (21.5, 30),
    "road_12": (21.5, 69),
    "road_14": (21.5, 69),
}


class GetNodes:
    def __init__(self, node, name) -> None:
        self.node = node
        self.name = name
        pass

class road:
    def __init__(self, name, width, length, angle, x, y) -> None:
        self.name = name
        self.width = width
        self.length = length
        self.angle = angle
        self.x = x
        self.y = y
        pass

def getNodes():
    # Set up the Webots supervisor node
    supervisor = Supervisor()
    time_step = int(supervisor.getBasicTimeStep())

    # nodes we need
    '''
        CurvedRoadSegment
        StraightRoadSegment
        RoadIntersection
    '''

    roadTypes = ["CurvedRoadSegment", "StraightRoadSegment", "RoadIntersection"]



    # Get the root node of the scene
    root_node = supervisor.getRoot()

    print(root_node)
    # # Get a list of all child nodes of the root node
    all_nodes = root_node.getField("children")
    
    road_nodes = []
    # Loop through all nodes and print their names
    for i in range(all_nodes.getCount()):
        node = all_nodes.getMFNode(i)
        # print(str(node.getTypeName()))

        if (str(node.getTypeName())  in roadTypes ):
            
            #print(node.getField("name").getSFString())
            road_nodes.append(GetNodes(node, node.getField("name").getSFString()))
    print(len(road_nodes))
    for rnodes in road_nodes:
        if "road" in rnodes.name:
            try:
                print(rnodes.node.getField("length").getSFFloat())
            except: 
                print("does not have length")
            print(rnodes.name, rnodes.node.getField("width").getSFFloat())
            
   