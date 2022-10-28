string set_name

string load_type

string[] load_positions                     # List of load positions / load handling devices

BoundingBoxReference bounding_box_reference # Bounding box reference as defined in parameter loads[] in state-message

LoadDimensions load_dimensions

float64 max_weight                          # Maximum weight of load type [kg]

float64 min_loadhandling_height             # Minimum allowed height for handling of this load-type and weight [m]

float64 max_loadhandling_height             # Maximum allowed height for handling of this load-type and weight [m]

float64 min_loadhandling_depth              # Minimum allowed depth for this load-type and weight [m]

float64 max_loadhandling_depth              # Maximum allowed depth for this load-type and weight [m]

float64 min_loadhandling_tilt               # Minimum allowed tilt for this load-type and weight [rad]

float64 max_loadhandling_tilt               # Maximum allowed tilt for this load-type and weight [rad]

float64 agv_speed_limit                     # Maximum allowed speed for this load-type and weight [m/s^2]

float64 agv_acceleration_limit              # Maximum allowed acceleration for this load-type and weight [m/s^2]

float64 agv_deceleration_limit              # Maximum allowed deceleration for this load-type and weight [m/s^2]

float64 pick_time                           # Approx. time for picking up the load [s]

float64 drop_time                           # Approx. time for dropping the load [s]

string description                          # Free description of the load handling set
