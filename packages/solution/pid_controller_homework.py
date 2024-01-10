from typing import Tuple

import numpy as np
import yaml
import os

def PIDController(
    v_0: float, y_ref: float, y_hat: float, prev_e_y: float, prev_int_y: float, delta_t: float
) -> Tuple[float, float, float, float]:
    """
    PID performing lateral control.

    Args:
        v_0:        linear Duckiebot speed (constant).
        y_ref:      target y coordinate.
        y_hat:      the current estimated y.
        prev_e_y:   tracking error at previous iteration.
        prev_int_y: previous integral error term.
        delta_t:    time interval since last call.

    Returns:
        v_0:        linear velocity of the Duckiebot
        omega:      angular velocity of the Duckiebot
        e:          current tracking error (automatically becomes prev_e_y at next iteration).
        e_int:      current integral error (automatically becomes prev_int_y at next iteration).
    """


# Read PID gains from file
    script_dir = os.path.dirname(__file__)
    file_path = os.path.join(script_dir, "GAINS.yaml")
    with open(file_path) as f:
        gains = yaml.full_load(f)
        
    kp = gains['kp']
    kd = gains['kd']
    ki = gains['ki']

# Calculate the lateral tracking error
    e_y = y_ref - y_hat
# Calculate the integral error term 
    e_int_y = prev_int_y + e_y * delta_t
# Implement anti-windup for the integral term
    e_int_y = max(min(e_int_y, 2.0), -2.0)
    e_der = (e_y - prev_e_y)/delta_t

    omega = kp*e_y + ki*e_int_y+kd*e_der
    return v_0, omega, e_y, e_int_y






