# Array of nodes to be traversed for fulfilling the order

string node_id                                      # Unique node identification
uint32 sequence_id                                  # sequenceId to discern multiple nodes with same nodeId.
string node_description                             # Additional information on the node
vda5050_msgs/NodePosition node_position                  # Node position (see Topic: Order)
bool released                                       # true indicates that the node is part of the base.
                                                    # false indicates that the node is part of the horizon.
