# Directional connection between two nodes

string edge_id                      # Unique edge identification

uint32 sequence_id                  # Id to track the sequence of nodes and edges in an order and to simplify order
                                    # updates. The variable sequence_id runs across all nodes and edges of the same order
                                    # and is reset when a new order_id is issued.

string edge_description             # Additional information on the edge

bool released                       # True indicates that the edge is part of the base. False indicates that the edge is
                                    # part of the horizon.

string start_node_id                # nodeID of startNode
string end_node_id                  # nodeID of endNode
float64 max_speed                   # [m/s] Permitted maximum speed on the edge. Speed is defined by the fastest point of the
                                    # vehicle.

float64 max_height                  # [m] Permitted maximum height of the vehicle, including the load, on edge
float64 min_height                  # [m] Permitted minimal height of the edge measured at the bottom of the load
float64 orientation                 # [rad] Orientation of the AGV on the edge relative to the global project specific
                                    # map coordinate origin (for holonomic vehicles with more than one driving
                                    # direction).
                                    # Example: orientation Pi/2 rad will lead to a rotation of 90 degrees.
                                    # If AGVstarts in different orientation, rotate the vehicle on the edge to the
                                    # desired orientation if rotationAllowed is set to “true”. If rotationAllowed
                                    # is “false", rotate before entering the edge. If that is not possible, reject
                                    # the order.
                                    # If a trajectory with orientation is defined, follow the trajectories orientation.
                                    # If a trajectory without orientation and the orientation field here is defined,
                                    # apply the orientation to the tangent of the trajectory.

string direction                    # Sets direction at junctions for line-guided vehicles, to be defined initially 
                                    # (vehicle individual) Example: left, right, straight, 433MHz

bool rotation_allowed               # “true”: rotation is allowed on the edge. “false”: rotation is not allowed on the edge.
                                    # Optional: Default to “false”. If this value is set, rotation is allowed on the edge.
float64 max_rotation_speed          # [rad/s] Maximum rotation speed Optional: No limit if not set
vda5050_msgs/Trajectory trajectory  # Trajectory JSON-object for this edge as a NURBS. Defines the curve on which the 
                                    # AGV should move between start_node and end_node. Optional: Can be omitted if AGV 
                                    # cannot process trajectories or if AGV plans its own trajectory.
float64 length                      # [m] Length of the path from startNode to endNode. Optional: This value is used
                                    # by lineguided AGVs to decrease their speed before reaching a stop position.

vda5050_msgs/Action[] actions       # Array of action_ids to be executed on the edge. An action triggered by an edge will
                                    # only be active for the time that the AGV is traversing the edge which triggered
                                    # the action. When the AGV leaves the edge, the action will stop and the state
                                    # before entering the edge will be restored.
