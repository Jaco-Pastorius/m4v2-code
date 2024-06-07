import numpy as np
from interpolate import create_interpolators, interpolate_values

# Create interpolators
x_interpolators, u_interpolators = create_interpolators()

# Interpolate values for a new time value, for example 5.5
t_new = 5.5
x_new, u_new = interpolate_values(x_interpolators, u_interpolators, t_new)

print("Interpolated x values at time", t_new, ":", x_new)
print("Interpolated u values at time", t_new, ":", u_new)
