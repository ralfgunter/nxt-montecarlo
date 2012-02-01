# ------------------------------------------------------------------------
# coding=utf-8
# ------------------------------------------------------------------------
#
#  Created by Martin J. Laubach on 2011-11-15
#
# ------------------------------------------------------------------------

import math
import turtle
import random

# Heading 0 is facing east.
turtle.mode("standard")

turtle.tracer(50000, delay=0)
turtle.register_shape("dot", ((-3,-3), (-3,3), (3,3), (3,-3)))
turtle.register_shape("tri", ((-3, -2), (0, 3), (3, -2), (0, 0)))
turtle.speed(0)
turtle.title("Poor robbie is lost")

UPDATE_EVERY = 0
DRAW_EVERY = 2

def sqr_distance(x1, y1, x2, y2):
    return (x1 - x2) ** 2 + (y1 - y2) ** 2

class Maze(object):
    def __init__(self, dim_x, dim_y, maze):
        # 'maze' is a list of pairs of points, which represent the endpoints
        # of each line segment.
        self.maze = maze

        self.dim_x = dim_x
        self.dim_y = dim_y

        turtle.setworldcoordinates(0, 0, self.dim_x, self.dim_y)
        self.update_cnt = 0

    def draw(self):
        for (ax, ay), (bx, by) in self.maze:
            turtle.up()
            turtle.setposition(ax, ay)
            turtle.down()
            turtle.setposition(bx, by)
            turtle.up()

        turtle.color("#00ffff")
        turtle.update()

    def weight_to_color(self, weight):
        return "#%02x00%02x" % (int(weight * 255), int((1 - weight) * 255))

    def show_mean(self, x, y, confident=False):
        if confident:
            turtle.color("#00AA00")
        else:
            turtle.color("#cccccc")
        turtle.setposition(x, y)
        turtle.shape("turtle")
        turtle.stamp()

    def show_particles(self, particles):
        self.update_cnt += 1
        if UPDATE_EVERY > 0 and self.update_cnt % UPDATE_EVERY != 1:
            return

        turtle.clearstamps()
        turtle.shape('tri')

        draw_cnt = 0
        for p in particles:
            draw_cnt += 1
            if DRAW_EVERY == 0 or draw_cnt % DRAW_EVERY == 1:
                turtle.setposition(*p.xy)
                turtle.setheading(p.h)
                turtle.color(self.weight_to_color(p.w))
                turtle.stamp()

    def show_robot(self, robot):
        turtle.color("red")
        turtle.shape('turtle')
        turtle.setposition(*robot.xy)
        turtle.setheading(robot.h)
        turtle.stamp()
        turtle.update()

    def random_pose(self):
        x = random.uniform(0, self.dim_x)
        y = random.uniform(0, self.dim_y)
        d = random.uniform(0, 360)
        return x, y, d

    def is_in(self, x, y, d):
        count = len(self.find_crossings(x, y, d))
        return x >= 0 and y >= 0 and x <= self.dim_x and y <= self.dim_y and count % 2 == 1

    def random_free_place(self):
        while True:
            p = self.random_pose()
            if self.is_in(*p):
                return p

    def intersection(self, xp, yp, dp, seg):
        x = seg[0][0] + (seg[1][0]-seg[0][0])*((seg[0][1]-yp) - (seg[0][0]-xp)*math.tan(dp))/((seg[1][0]-seg[0][0])*math.tan(dp) - (seg[1][1]-seg[0][1]))
        y = seg[0][1] + (seg[1][1]-seg[0][1])*((seg[0][1]-yp) - (seg[0][0]-xp)*math.tan(dp))/((seg[1][0]-seg[0][0])*math.tan(dp) - (seg[1][1]-seg[0][1]))
        return x, y

    def find_crossings(self, xp, yp, dp):
        r = lambda x,y: y -     math.tan(dp)*x +     math.tan(dp)*xp - yp
        t = lambda x,y: y + (1/math.tan(dp))*x - (1/math.tan(dp))*xp - yp
        xref, yref = xp + math.cos(dp), yp + math.sin(dp)
        refsign = t(xref, yref) > 0
        hits = []

        for segment in self.maze:
            if (r(*segment[0]) > 0) != (r(*segment[1]) > 0):
                p = self.intersection(xp, yp, dp, segment)
                if (t(*p) > 0) == refsign:
                    hits.insert(0, p)

        return hits

    def sonar_measure(self, xp, yp, dp):
        drad = math.radians(dp)
        hits = self.find_crossings(xp, yp, dp)

        min_dist = float('inf')
        for point in hits:
            min_dist = min(min_dist, sqr_distance(xp, yp, *point))

        return math.sqrt(min_dist), len(hits)
