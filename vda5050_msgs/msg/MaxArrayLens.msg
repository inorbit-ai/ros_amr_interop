uint32 order_nodes                  # Maximum number of nodes per order processable by the AGV

uint32 order_edges                  # Maximum number of edges per order processable by the AGV

uint32 node_actions                 # Maximum number of action per node processable by the AGV

uint32 edge_actions                 # Maximum number of action per edge processable by the AGV

uint32 actions_parameters           # Maximum number of parameters per action processable by the AGV

uint32 instant_actions              # Maximum number of instant actions per message processable by the AGV

uint32 trajectory_knot_vector       # Maximum number of knots per trajectory processable by the AGV

uint32 trajectory_control_points    # Maximum number of control points per trajectory processable by the AGV

uint32 state_node_states            # Maximum number of nodeStates sent by the AGV, maximum number of nodes in base of AGV

uint32 state_edge_states            # Maximum number of edgeStates sent by the AGV, maximum number of edges in base of AGV

uint32 state_loads                  # Maximum number of load-objects sent by the AGV

uint32 state_action_states          # Maximum number of actionStates sent by the AGV

uint32 state_errors                 # Maximum number of errors sent by the AGV in one state-message

uint32 state_information            # Maximum number of information objects sent by the AGV in one state-message

uint32 error_references             # Maximum number of error references sent by the AGV for each error

uint32 info_references              # Maximum number of info references sent by the AGV for each information