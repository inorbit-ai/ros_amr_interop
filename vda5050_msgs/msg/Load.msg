string load_id                                           # Unique identification number of the load (e. g. barcode or
                                                        # RFID)

string load_type                                         # Type of load

string load_position                                     # Indicates which load handling/carrying unit of the AGV is
                                                        # used, e. g. in case the AGV has multiple spots/positions to
                                                        # carry loads. For example: “front”, “back”, “positionC1”, etc.

vda5050_msgs/BoundingBoxReference bounding_box_reference  # Point of reference for the location of the bounding box. The
                                                        # point of reference is always the center of the bounding box’s
                                                        # bottom surface (at height = 0) and is described in coordinates
                                                        # of the AGV’s coordinate system.

vda5050_msgs/LoadDimensions load_dimensions              # Dimensions of the load’s bounding box in meters.

float64 weight                                           # Absolute weight of the load measured in kg.
