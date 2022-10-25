string error_type                              # Type / name of error
vda5050_msgs/ErrorReference[] error_references   # Array of references to identify the source of the error (e. g. header_id,
                                              # order_id, action_id, …). For additional information see best practice
                                              # chapter 6.3
string error_description                       # Error description 
string error_level                             # Enum {warning, fatal} warning: AGV is ready to start (e.g. maintenance
                                              # cycle expiration warning) fatal: AGV is not in running condition, user
                                              # intervention required (e.g. laser scanner is contaminated)

# Enums for error_level
string WARNING="WARNING"
string FATAL="FATAL"
