int64 id                        # object unique id number
string class_name               # e.g. cone, bicycle, ...
geometry_msgs/Point position    # object position
float64[] covariance            # object covariance (should include width/height if use_wh is true)
float64 width                   # object width
float64 height                  # object height

float64 ell     # number of updates since object was last observed
