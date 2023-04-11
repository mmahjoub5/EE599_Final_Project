import numpy as np

from controller import Supervisor

def getNodes():
#     # Set up the Webots supervisor node
#     supervisor = Supervisor()
#     time_step = int(supervisor.getBasicTimeStep())

#     # Get the root node of the scene
#     root_node = supervisor.getRoot()

#     # Get a list of all child nodes of the root node
#     all_nodes = root_node.getField("children")

#     # Loop through all nodes and print their names
#     for i, node in enumerate(root_node.getChildren()):
#         print(node.getName())
#         # node = root_node.getChild(i)
#         # node_name = node.getName()
#         # print("Node %d: name='%s'" % (i, node_name))
#         # node = all_nodes.getMFNode(i).get
        
#         # node_name = node.getField("name")
#         # print(node.getName())
#         # break
#         # print("Node %d: name='%s'" % (i, node_name))
#         # for i in range(node.getNumberOfFields()):
#         #     field = node.getFieldByIndex(i)
#         #     field_name = field.getName()
#         #     field_type = field.getTypeName()
#         #     node_name = field.getNode().getName()
#         #     print(node_name)
#         #     # print("Field %d: name='%s', type='%s'" % (i, field_name, field_type))



    # Set up the Webots supervisor node
    supervisor = Supervisor()

    # Get the root node of the scene
    root_node = supervisor.getRoot()

    # Loop through all nodes in the scene and print their names
    for i, node in enumerate(root_node.getChildren()):
        node_name = node.getName()
        print("Node %d: name='%s'" % (i, node_name))
