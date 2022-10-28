# Defines the position on a map in world coordinates. Each floor has its own map.

float64 x                                         # [m] X-position on the map in reference to the world coordinate system
float64 y                                         # [m] Y-position on the map in reference to the world coordinate system

float64 theta                                     # [rad] The angular dimension


float32 allowed_deviation_x_y                     # Indicates how exact an AGV has to drive over a node in order for it
                                                  # to count as traversed.
                                                  #
                                                  # If = 0: no deviation is allowed (no deviation means within the
                                                  # normal tolerance of the AGV manufacturer).
                                                  #
                                                  # If > 0: allowed deviationradius in meters. If the AGV passes a node
                                                  # within the deviation-radius, the node is considered to have been
                                                  # traversed.

float32 allowed_deviation_theta                   # Range: [0 ... Pi]
                                                  # Indicates how big the deviation of theta angle can be.
                                                  # The lowest acceptable angle  is theta -allowedDevaitionTheta and
                                                  # the  highest acceptable angle is theta + allowedDeviationTheta

string map_id                                     # Unique identification of the map in which the position is referenced. Each map has the same
                                                  # origin of coordinates. When an AGV uses an elevator, e. g. leading from a departure floor to a
                                                  # target floor, it will disappear off the map of the departure floor and spawn in the related
                                                  # lift node on the map of the target floor.

string map_description                            # Additional information on the map
