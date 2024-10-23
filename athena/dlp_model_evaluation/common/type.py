class Point2D:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

class PlanningPoint:
    def __init__(self, position=None, theta=0.0, velocity=0.0, acceleration=0.0, 
                 omega=0.0, curva=0.0, jerk=0.0, omega_dot=0.0):
        self.position = position if position is not None else Point2D()
        self.theta = theta
        self.velocity = velocity
        self.acceleration = acceleration
        self.omega = omega
        self.curva = curva
        self.jerk = jerk
        self.omega_dot = omega_dot

    def __repr__(self):
        return (f"PlanningPoint(position=({self.position.x}, {self.position.y}), theta={self.theta}, "
                f"velocity={self.velocity}, acceleration={self.acceleration}, "
                f"omega={self.omega}, curva={self.curva}, jerk={self.jerk}, omega_dot={self.omega_dot})")