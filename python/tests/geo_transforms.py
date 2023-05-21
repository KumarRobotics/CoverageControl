import numpy as np
import pyCoverageControl
from pyCoverageControl import GeoLocalTransform

# Provide lat, lon, altitude as the origin
geo_transform = GeoLocalTransform(40.74050005471615, -74.1759877275644, 0)

# Reverse() takes in x, y, z relative to the origin and gives out [lat, lon, altitude]
lla = geo_transform.Reverse(100, 100, 0);
print(lla)

# Forward() takes in lat, lon, altitude and gives out [x, y, z] relative to origin
xyz = geo_transform.Forward(lla[0], lla[1], lla[2]);
print(xyz) # Should be [100 100 0]
