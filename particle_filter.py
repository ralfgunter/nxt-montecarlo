# ------------------------------------------------------------------------
# coding=utf-8
# ------------------------------------------------------------------------
#
#  Created by Martin J. Laubach on 2011-11-15
#
# ------------------------------------------------------------------------

from __future__ import absolute_import

import random
import bisect
import math
import time
import draw

import nxt
from nxt.motcont import MotCont

from draw import Maze

# Total number of particles
PARTICLE_COUNT = 3000

# Does the robot know where north is? If so, it
# makes orientation a lot easier since it knows which direction it is facing.
# If not -- and that is really fascinating -- the particle filter can work
# out its heading too, it just takes more particles and more time. Try this
# with 3000+ particles, it obviously needs lots more hypotheses as a particle
# now has to correctly match not only the position but also the heading.
ROBOT_HAS_COMPASS = False

ROBOT_LENGTH = 13

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(3.0, *coords)

def add_some_noise(*coords):
    return add_noise(3.0, *coords)

# This is just a gaussian kernel I pulled out of my hat, to transform
# values near to robbie's measurement => 1, further away => 0
sigma2 = 3.0 ** 2
def w_gauss(a, b):
    error = a - b
    g = math.e ** -(error ** 2 / (2 * sigma2))
    return g

# ------------------------------------------------------------------------
def compute_mean_point(particles):
    """
    Compute the mean for all particles that have a reasonably good weight.
    This is not part of the particle filter algorithm but rather an
    addition to show the "best belief" for current position.
    """

    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        m_count += p.w
        m_x += p.x * p.w
        m_y += p.y * p.w

    if m_count == 0:
        return -1, -1, False

    m_x /= m_count
    m_y /= m_count

    # Now compute how good that mean is -- check how many particles
    # actually are in the immediate vicinity
    m_count = 0
    for p in particles:
        if draw.sqr_distance(p.x, p.y, m_x, m_y) < 10:
            m_count += 1

    return m_x, m_y, m_count > PARTICLE_COUNT * 0.95

# ------------------------------------------------------------------------
class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = [p for p in state if p.w > 0]
        self.distribution = []
        for x in self.state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        try:
            return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]
        except IndexError:
            # Happens when all particles are improbable w=0
            return None

# ------------------------------------------------------------------------
class Particle(object):
    def __init__(self, x, y, heading=None, w=1, noisy=False):
        if heading is None:
            heading = random.uniform(0, 360)
        if noisy:
            x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place()) for _ in range(0, count)]

    def read_sensor(self, maze):
        return maze.sonar_measure(*self.xyh)[0]

    def advance_by(self, speed, checker=None, noisy=False):
        h = self.h
        if noisy:
            speed, h = add_little_noise(speed, h)
            h += random.uniform(-5, 10) # needs more noise to disperse better
        r = math.radians(h)
        dx = math.cos(r) * speed
        dy = math.sin(r) * speed
        if checker is None or checker(self, dx, dy):
            self.move_by(dx, dy)
            return True
        return False

    def move_by(self, x, y):
        self.x += x
        self.y += y

# ------------------------------------------------------------------------
class Robot(Particle):
    def __init__(self, maze, bot):
        super(Robot, self).__init__(180, 35, 90)
        #super(Robot, self).__init__(*maze.random_free_place())
        self.step_count = 0

        # NXT specific
        self.motcont = MotCont(bot)
        self.motcont.start()
        self.sonar = nxt.Ultrasonic(bot, nxt.PORT_3)

        self.step = 360
        self.march_power = 80
        self.turn_power  = 50
        self.speed = 10

        #self.choose_random_direction()

    def choose_random_direction(self):
        heading = random.uniform(0, 360)
        self.turn_inplace(((heading - self.h) % 360) * 6)
        print("heading :" + repr(heading))
        self.h = heading

    def read_sensor(self, maze):
        # Sonar is not exactly at the robot's baricenter.
        s = self.sonar.get_sample() + ROBOT_LENGTH
        print("sonar: " + repr(s - ROBOT_LENGTH))
        return s

    def turn_inplace(self, degrees):
        self.motcont.cmd(nxt.motor.PORT_A,  self.turn_power, degrees, True, True)
        self.motcont.cmd(nxt.motor.PORT_C, -self.turn_power, degrees, True, True)
        while not (self.motcont.is_ready(nxt.motor.PORT_A) and self.motcont.is_ready(nxt.motor.PORT_C)):
            time.sleep(0.01)

    def move_ahead(self, tacholimit):
        self.motcont.cmd(nxt.motor.PORT_A, self.march_power, tacholimit, True, True)
        self.motcont.cmd(nxt.motor.PORT_C, self.march_power, tacholimit, True, True)
        while not (self.motcont.is_ready(nxt.motor.PORT_A) and self.motcont.is_ready(nxt.motor.PORT_C)):
            time.sleep(0.01)

    def advance_by(self, speed, checker=None):
        h = self.h
        r = math.radians(h)
        dx = math.cos(r) * self.speed
        dy = math.sin(r) * self.speed
        if checker is None or checker():
            self.move_ahead(self.step)
            self.move_by(dx, dy)
            return True
        return False

    def move(self, maze):
        while True:
            self.step_count += 1
            if self.advance_by(self.speed,
                checker=lambda: self.sonar.get_sample() > 20):
                break
            # Bumped into something or too long in same direction,
            # choose random new direction
            self.choose_random_direction()

# ------------------------------------------------------------------------

# The maze
"""
tmap = [
         ((0, 80), (0, 320)),
         ((0, 320), (79, 320)),
         ((79, 320), (79, 240)),
         ((79, 240), (81, 240)),
         ((81, 240), (81, 320)),
         ((81, 320), (160, 320)),

         ((160, 320), (160, 161)),
         ((160, 159), (70, 159)),
         ((70, 159), (70, 161)),
         ((70, 161), (160, 161)),

         ((160, 159), (160, 0)),
         ((160, 0), (80, 0)),
         ((80, 0), (80, 80)),
         ((80, 80), (0, 80)),
       ]
"""
tmap = [
        ((40, 0), (40, 160)),
        ((40, 160), (0, 160)),
        ((0, 160), (0, 240)),
        ((0, 240), (120, 240)),
        ((120, 240), (120, 200)),
        ((120, 200), (240, 200)),
        ((240, 200), (240, 40)),
        ((240, 40), (200, 40)),
        ((200, 40), (200, 0)),
        ((200, 0), (40, 0)),

        ((88, 48), (88, 80)),
        ((88, 80), (120, 80)),
        ((120, 80), (120, 48)),
        ((120, 48), (88, 48)),

        ((160, 120), (189, 120)),
        ((189, 120), (189, 91)),
        ((189, 91), (160, 91)),
        ((160, 91), (160, 120)),

        ((120, 200), (120, 160)),
        ((120, 160), (96, 160)),
        ((96, 160), (120, 200))
       ]

world = Maze(240, 240, tmap)
world.draw()

# connect to brick
bot = nxt.find_one_brick()

# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT, world)
robbie = Robot(world, bot)

while True:
    # Read robbie's sensor
    r_d = robbie.read_sensor(world)

    # Update particle weight according to how good every particle matches
    # robbie's sensor reading
    for p in particles:
        if world.is_in(*p.xyh):
            p_d = p.read_sensor(world)
            p.w = w_gauss(r_d, p_d)
        else:
            p.w = 0

    # ---------- Try to find current best estimate for display ----------
    m_x, m_y, m_confident = compute_mean_point(particles)

    # ---------- Show current state ----------
    world.show_particles(particles)
    world.show_mean(m_x, m_y, m_confident)
    world.show_robot(robbie)

    # ---------- Shuffle particles ----------
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        if p is None:  # No pick b/c all totally improbable
            new_particle = Particle.create_random(1, world)[0]
        else:
            new_particle = Particle(p.x, p.y,
                    heading=robbie.h if ROBOT_HAS_COMPASS else p.h,
                    noisy=True)
        new_particles.append(new_particle)

    particles = new_particles

    # ---------- Move things ----------
    old_heading = robbie.h
    robbie.move(world)
    d_h = robbie.h - old_heading

    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in particles:
        p.h += d_h # in case robot changed heading, swirl particle heading too
        p.advance_by(robbie.speed)
