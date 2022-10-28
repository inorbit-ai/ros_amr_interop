float64 x           # X coordinate described in the world coordinate system.

float64 y           # Y coordinate described in the world coordinate system.

float64 orientation # [rad] Range [-pi...pi] Orientation of the AGV on this position of the curve.
                    # The orientation is in world coordinates.
                    # When not defined the orientation of the AGV will be tangential to the curve.

float64 weight      # Range [0..infinity) The weight with which this control point pulls on the curve.
                    # When not defined, the default will be 1.0
