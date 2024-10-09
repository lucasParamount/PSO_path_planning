import numpy as np
class Env:
    def __init__(self):
        self.x_range = (0, 1500)
        self.y_range = (0, 1200)
        self.start = np.array((2, 2))
        self.goal = np.array((845, 805))
        self.obs_boundary = self.obs_boundary()
        self.obs_circle = self.obs_circle()
        self.obs_rectangle = self.obs_rectangle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 1200],
            [0, 1200, 1500, 1],
            [1, 0, 1500, 1],
            [1500, 1, 1, 1200]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():  # [x_downleft_point,y_downleft_point,horizontal_length,vertical_length]
        obs_rectangle = [
            [639, 726, 86, 70],
            [750, 726, 94, 68],
            [672, 826, 18, 74],
            [750, 820, 80, 80],
            [850, 790, 128, 110],
            [1006, 724, 95, 176],
            [1113, 784, 87, 116],
            [600, 985, 230, 185],
            [639, 726, 86, 70],
            [385, 895, 70, 100],
            [385, 1013, 81, 60],
            [725, 521, 134, 183],
            [893, 497, 126, 207]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [0, 0, 0],
        ]

        return obs_cir

    def check_collision(self, p1, p2):
        for obs in self.obs_rectangle:
            left, bottom = obs[0], obs[1]
            right = left + obs[2]
            top = bottom + obs[3]

            # Check if the line segment (p1, p2) intersects the rectangle (obs)
            if self.line_intersects_rect(p1, p2, left, right, bottom, top):
                return True
        return False

    # Check if a line segment intersects a rectangle
    def line_intersects_rect(self, p1, p2, left, right, bottom, top):
        def on_segment(p, q, r):
            return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                    q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

        def orientation(p, q, r):
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0  # collinear
            return 1 if val > 0 else 2  # clockwise or counterclockwise

        def do_intersect(p1, q1, p2, q2):
            o1 = orientation(p1, q1, p2)
            o2 = orientation(p1, q1, q2)
            o3 = orientation(p2, q2, p1)
            o4 = orientation(p2, q2, q1)

            if o1 != o2 and o3 != o4:
                return True
            if o1 == 0 and on_segment(p1, p2, q1):
                return True
            if o2 == 0 and on_segment(p1, q2, q1):
                return True
            if o3 == 0 and on_segment(p2, p1, q2):
                return True
            if o4 == 0 and on_segment(p2, q1, q2):
                return True
            return False

        # Check the 4 edges of the rectangle
        rect_points = [(left, bottom), (right, bottom), (right, top), (left, top)]
        for i in range(4):
            if do_intersect(p1, p2, rect_points[i], rect_points[(i + 1) % 4]):
                return True
        return False