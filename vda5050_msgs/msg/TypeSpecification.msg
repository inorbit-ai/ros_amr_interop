string series_name                  # Generalized series name as specified by manufacturer

string series_description           # Human readable description of the AGV type series

string agv_kinematic                # Simplified description of AGV kinematics-type

string agv_class                    # Simplified description of AGV class

float64 max_load_mass               # Maximum loadable mass [Kg]

string[] localization_types         # Simplified description of localization type

string[] navigation_types           # Path planning types supported by the AGV, sorted by priority

# Enums for agv_kinematic
string DIFF = "DIFF"
string OMNI = "OMNI"
string THREEWHEEL = "THREEWHEEL"

# Enums for agv_class
string FORKLIFT = "FORKLIFT"
string CONVEYOR = "CONVEYOR"
string TUGGER = "TUGGER"
string CARRIER = "CARRIER"

# Enums for localization_types
string NATURAL = "NATURAL"
string REFLECTOR = "REFLECTOR"
string RFID = "RFID"
string DMC = "DMC"
string SPOT = "SPOT"
string GRID = "GRID"

# Enums for navigation_types
string PHYSICAL_LINE_GUIDED = "PHYSICAL_LINE_GUIDED"
string VIRTUAL_LINE_GUIDED = "VIRTUAL_LINE_GUIDED"
string AUTONOMOUS = "AUTONOMOUS"