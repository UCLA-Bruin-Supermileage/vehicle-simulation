class VehicleState:
    """
    Mutable simulation state.

    The old straight-line models use x/v/a. Those are kept as aliases for
    distance along track, longitudinal speed, and longitudinal acceleration.
    """

    def __init__(self):
        self.s_m = 0.0
        self.v_mps = 0.0
        self.a_mps2 = 0.0
        self.t = 0.0
        self.lap = 0
        self.distance_total_m = 0.0
        self.energy_used_J = 0.0

    @property
    def x(self):
        return self.s_m

    @x.setter
    def x(self, value):
        self.s_m = float(value)

    @property
    def v(self):
        return self.v_mps

    @v.setter
    def v(self, value):
        self.v_mps = float(value)

    @property
    def a(self):
        return self.a_mps2

    @a.setter
    def a(self, value):
        self.a_mps2 = float(value)
