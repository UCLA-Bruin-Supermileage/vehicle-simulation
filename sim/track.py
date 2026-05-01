from dataclasses import dataclass
import math


FT_TO_M = 0.3048
M_TO_FT = 1.0 / FT_TO_M


@dataclass(frozen=True)
class TrackSegment:
    name: str
    kind: str
    start_m: float
    end_m: float
    curvature_1pm: float

    @property
    def length_m(self):
        return self.end_m - self.start_m


class OvalTrack:
    """
    Centerline model for a rounded-rectangle/oval style track.

    The path starts at the left end of the bottom straight and travels
    counterclockwise: bottom straight, bottom-right corner, right straight, etc.
    """

    def __init__(
        self,
        long_straight_m,
        short_straight_m,
        corner_arc_m,
        name="oval",
    ):
        self.name = name
        self.long_straight_m = float(long_straight_m)
        self.short_straight_m = float(short_straight_m)
        self.corner_arc_m = float(corner_arc_m)
        self.corner_radius_m = self.corner_arc_m / (math.pi / 2.0)

        self.x_left_m = -self.long_straight_m / 2.0
        self.x_right_m = self.long_straight_m / 2.0
        self.y_bottom_m = -(self.short_straight_m / 2.0 + self.corner_radius_m)
        self.y_top_m = +(self.short_straight_m / 2.0 + self.corner_radius_m)

        c = 1.0 / self.corner_radius_m
        lengths = [
            ("bottom_straight", "straight", self.long_straight_m, 0.0),
            ("bottom_right_corner", "corner", self.corner_arc_m, c),
            ("right_straight", "straight", self.short_straight_m, 0.0),
            ("top_right_corner", "corner", self.corner_arc_m, c),
            ("top_straight", "straight", self.long_straight_m, 0.0),
            ("top_left_corner", "corner", self.corner_arc_m, c),
            ("left_straight", "straight", self.short_straight_m, 0.0),
            ("bottom_left_corner", "corner", self.corner_arc_m, c),
        ]

        self.segments = []
        start = 0.0
        for name, kind, length, curvature in lengths:
            end = start + length
            self.segments.append(TrackSegment(name, kind, start, end, curvature))
            start = end
        self.length_m = start

    @classmethod
    def from_feet(
        cls,
        long_straight_ft=3300.0,
        short_straight_ft=660.0,
        corner_arc_ft=1320.0,
        name="oval",
    ):
        return cls(
            long_straight_m=long_straight_ft * FT_TO_M,
            short_straight_m=short_straight_ft * FT_TO_M,
            corner_arc_m=corner_arc_ft * FT_TO_M,
            name=name,
        )

    def wrap_s(self, s_m):
        return float(s_m) % self.length_m

    def segment_at(self, s_m):
        s = self.wrap_s(s_m)
        for segment in self.segments:
            if segment.start_m <= s < segment.end_m:
                return segment
        return self.segments[-1]

    def curvature_at(self, s_m):
        return self.segment_at(s_m).curvature_1pm

    def speed_limit_at(self, s_m, mu=0.9, g=9.81):
        curvature = self.curvature_at(s_m)
        if curvature <= 0.0:
            return math.inf
        return math.sqrt(float(mu) * float(g) / curvature)

    def xy_at(self, s_m):
        s = self.wrap_s(s_m)
        segment = self.segment_at(s)
        u = s - segment.start_m
        r = self.corner_radius_m
        lx = self.long_straight_m
        ly = self.short_straight_m
        xl = self.x_left_m
        xr = self.x_right_m
        yb = self.y_bottom_m
        yt = self.y_top_m

        if segment.name == "bottom_straight":
            return xl + u, yb

        if segment.name == "bottom_right_corner":
            theta = -math.pi / 2.0 + u / r
            return xr + r * math.cos(theta), -ly / 2.0 + r * math.sin(theta)

        if segment.name == "right_straight":
            return xr + r, -ly / 2.0 + u

        if segment.name == "top_right_corner":
            theta = 0.0 + u / r
            return xr + r * math.cos(theta), ly / 2.0 + r * math.sin(theta)

        if segment.name == "top_straight":
            return xr - u, yt

        if segment.name == "top_left_corner":
            theta = math.pi / 2.0 + u / r
            return xl + r * math.cos(theta), ly / 2.0 + r * math.sin(theta)

        if segment.name == "left_straight":
            return xl - r, ly / 2.0 - u

        theta = math.pi + u / r
        return xl + r * math.cos(theta), -ly / 2.0 + r * math.sin(theta)

    def sample_xy(self, points_per_segment=200):
        xs = []
        ys = []
        for segment in self.segments:
            count = max(2, int(points_per_segment))
            for i in range(count):
                s = segment.start_m + segment.length_m * i / (count - 1)
                x, y = self.xy_at(s)
                xs.append(x)
                ys.append(y)
        return xs, ys
