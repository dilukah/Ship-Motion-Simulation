# model.py

import numpy as np

class Ship:
    """ Simplified dynamic model of a surface vessel in 2D """
    def __init__(self, mass=500.0, inertia=200.0, damping=(50, 50, 20), dt=0.1):
        """
        mass = ...          # kg, ship's mass (total displacement mass)
        inertia = ...       # kg·m², rotational inertia around yaw axis
        damping = (         # N·s/m or N·s·m, hydrodynamic damping coefficients
        ...,            # surge damping (X-direction)
        ...,            # sway damping (Y-direction)
        ...             # yaw damping (rotation about Z-axis)
        )
        dt = ...            # s, time step for numerical integration
        """
        self.mass = mass
        self.inertia = inertia
        self.damping = damping
        self.dt = dt
        # State: x, y, psi (heading), u, v, r
        #   x   Global x-position of the ship
        #   y   Global y-position of the ship
        #   psi Heading angle (yaw)
        #   u   Surge velocity (forward speed in body frame)
        #   v   Sway velocity (sideways speed in body frame)
        #   r   Yaw rate (angular velocity around z-axis)
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def reset(self, state=None):
        self.state = state if state is not None else np.zeros(6)

    def step(self, yaw_control, wind=np.zeros(3)):
        """ 
        yaw_control = Control input (torque/moment to steer)
        wind = External disturbance forces  
        """
        x, y, psi, u, v, r = self.state
        X = 100  # Constant forward thrust
        Y = wind[1]
        N = yaw_control + wind[2]

        u_dot = (X - self.damping[0]*u) / self.mass
        v_dot = (Y - self.damping[1]*v) / self.mass
        r_dot = (N - self.damping[2]*r) / self.inertia

        u += u_dot * self.dt
        v += v_dot * self.dt
        r += r_dot * self.dt

        x += (u * np.cos(psi) - v * np.sin(psi)) * self.dt
        y += (u * np.sin(psi) + v * np.cos(psi)) * self.dt
        psi += r * self.dt

        self.state = np.array([x, y, psi, u, v, r])
        return self.state

    def get_position(self):
        return self.state[:2]

    def get_heading(self):
        return self.state[2]