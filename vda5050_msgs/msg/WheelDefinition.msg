string type                 # Wheel type
bool is_active_driven
bool is_active_steered
Position position
float64 diameter            # Nominal diameter of the wheel [m]
float64 width               # Nominal width of the wheel [m]
float64 center_displacement # Nominal displacement of the wheel’s center to the rotation point [m]
string constraints

# Enums for wheel type
string DRIVE = "DRIVE"
string CASTER = "CASTER"
string FIXED = "FIXED"
string MECANUM = "MECANUM"