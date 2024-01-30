# HEADER
uint32 header_id                                      # header ID of the message. The header_id is defined per topic and incremented by 1 with each sent
                                                     # (but not necessarily received) message.

string timestamp                                     # Timestamp after ISO8601 in the format YYYY-MM-DDTHH:mm:ss.ssZ (e.g.“2017-04-15T11:40:03.12Z”)

string version                                       # Version of the protocol [Major].[Minor].[Patch] (e.g. 1.3.2)

string manufacturer                                  # Manufacturer of the AGV

string serial_number                                 # Serial Number of the AGV

# CONTENTS
string order_id                                      # Unique order identification “none” if vehicle is idle and has no order

uint32 order_update_id                               # order_update identification. Is unique per order_id. If an order update is rejected, this field is to be passed in the rejection message.

string zone_set_id                                   # Unique ID of the zone set that the AGV currently uses for path planning. Must be the same as the one used in the order,
                                                     # otherwise the AGV hasto reject the order. Optional: If the AGV does not use zones, this field can be omitted.

string last_node_id                                  # nodeId of last reached node or, if AGV is currently on a node, current node (e.g. „node7”).
                                                     # Empty string ("") if no lastNodeId is available.

uint32 last_node_sequence_id                         # sequence_id of the last reached node or, if the AGV is currently on a node, sequence_id of current node.
                                                     # “0” if no last_node_sequence_id is available.

vda5050_msgs/NodeState[] node_states                 # Array of node_state_objects (empty list if idle)

vda5050_msgs/EdgeState[] edge_states                 # Array of edge_state_objects (empty list if idle)

vda5050_msgs/AGVPosition agv_position                # Current position of the AGV. Optional: Can only be omitted for
                                                     # AGVs without the capability to localize themselves, e.g. line
                                                     # guided AGVs.

vda5050_msgs/Velocity velocity                       # AGV's velocity in vehicle coordinates

vda5050_msgs/Load[] loads                            # Loads that are currently handled by the AGV.
                                                     # Optional: If AGV cannot determine load state, leave the array out of the state.
                                                     # If the  AGV  can determine the  load  state,  but  the  array  is  empty,  the  AGV  is considered unloaded.

bool driving                                         # True: indicates that the AGV is driving and/or rotating. Other
                                                     # movements of the AGV (e.g. lift movements) are not included here.
                                                     # False: indicates that the AGV is neither driving nor rotating

bool paused                                          # True: AGV is currently in a paused state, either because of the push of a physical button on the AGV or
                                                     # because of an instantAction. The AGV can resume the order.
                                                     # False: The AGV is currently not in a paused state

bool new_base_request                                # True: AGV is almost at the end of the base and will reduce speed if no new base is transmitted. Trigger for MC to send ne base
                                                     # False: no base update required

float64 distance_since_last_node                     # Used by line guided vehicles to indicate the distance it has been driving past the last_node_id. Distance is in meters

vda5050_msgs/CurrentAction[] action_states           # Contains a list of the current actions and the actions which are
                                                     # yet to be finished. This may include actions from previous nodes
                                                     # that are still in progress. When an action is completed, an
                                                     # updated state message is published with action_status set to
                                                     # finished and if applicable with the corresponding
                                                     # result_description. Completed actions are omitted from the array

vda5050_msgs/BatteryState battery_state              # Contains all batteryrelated information.

string operating_mode                                # Enum {AUTOMATIC, SEMIAUTOMATIC, MANUAL, SERVICE, TEACHIN}
                                                     # For additional information see chapter 6.2

vda5050_msgs/Error[] errors                          # Array of errorobjects. Empty array if there are no errors.

vda5050_msgs/Info[] information                      # Array of info-objects. An empty array indicates that the AGV has no information.
                                                     # This should only be used for visualization or debugging – it must not be used for logic in master control

vda5050_msgs/SafetyState safety_state                # Contains all safetyrelated information.



                                                     # Enums for operatingMode
string AUTOMATIC="AUTOMATIC"

                                                     # AGV is under full control of the supervisor. AGV drives and executes actions based on orders from the supervisor
string SEMIAUTOMATIC="SEMIAUTOMATIC"

                                                     # AGV is under control of the supervisor. AGV drives and executes actions based on orders from the supervisor. The driving speeds is controlled by the HMI. (speed can’t exceed the speed of automatic mode) The steering is under automatic control. (non-safe HMI possible)
string MANUAL="MANUAL"

                                                     # Supervisor is not in control of the AGV. Supervisor doesn’t send driving order or actions to the AGV. HMI can be used the control the steering and velocity and handling device of the AGV. Location of the AGV is send to the supervisor. When AGV enters or leaves this mode, it immediately clears all the orders. (safe HMI required)
                                                     # Supervisor is not in control of the AGV. Supervisor doesn’t send driving order or actions to the AGV. Authorized personal can reconfigure the AGV.
string SERVICE="SERVICE"

                                                     # Supervisor is not in control of the AGV. Supervisor doesn’t send driving order or actions to the AGV. The AGV is being taught, e.g. mapping is done by a supervisor
string TEACHIN="TEACHIN"
