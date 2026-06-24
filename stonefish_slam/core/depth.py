"""Hydrostatic pressure-to-depth conversion.

Pure numpy, no ROS imports, so it loads under the test fixture without
rclpy/gtsam contamination (mirrors the kalman_filter.py extraction pattern).
"""

# Hydrostatic constants (seawater).
P_ATM_PA = 101325.0      # standard atmospheric pressure, Pascals
RHO_SEAWATER = 1025.0    # seawater density, kg/m^3
G = 9.80665              # standard gravity, m/s^2


def pressure_to_depth(fluid_pressure: float) -> float:
    """Convert an absolute fluid pressure to depth.

    ``sensor_msgs/FluidPressure.fluid_pressure`` is absolute pressure in
    Pascals. Hydrostatics give ``P = P_atm + rho*g*h``, so
    ``h = (P - P_atm) / (rho*g)``.

    Frame convention: NED (z-down), so a deeper measurement returns a larger
    POSITIVE depth; an above-atmospheric reading near the surface returns a
    small negative value (not clamped — honest sensor data).

    Args:
        fluid_pressure (float): absolute pressure in Pascals.

    Returns:
        float: depth in metres, positive downward (NED).
    """
    return (fluid_pressure - P_ATM_PA) / (RHO_SEAWATER * G)
