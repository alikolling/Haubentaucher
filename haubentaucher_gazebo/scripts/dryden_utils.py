#! /usr/bin/env python3
"""
Utility scripts for the Dryden wind model
The Dryden Gust model is implemented using this package: 
https://github.com/goromal/wind-dynamics

"""

import numpy as np
from rotors_comm.msg import *
import rospy
import random

class GustModelBase:
    """ """

    def __init__(self, V, L, sigma, dt=0.05):
        """
        Inputs:
            V, average velocity through medium, m/s
            L, characteristic length scale, m
            sigma, turbulence intensity
        """

        self.dt = dt
        b = 2 * np.sqrt(3) * L / V
        c = 2 * L / V

        self.alpha = sigma * np.sqrt(2 * L / np.pi / V)
        self.beta = self.alpha * b
        self.delta = 2 * c
        self.gamma = c * c

        self.u_km1 = 0
        self.u_km2 = 0
        self.y_km1 = 0
        self.y_km2 = 0

        self.initialized = True
        return

    def run(self, dt):
        """ """

        C1 = 1.0 + 2 * self.delta / dt + 4 * self.gamma / dt / dt
        C2 = 2.0 - 8 * self.gamma / dt / dt
        C3 = 1.0 - 2 * self.delta / dt + 4 * self.gamma / dt / dt
        C4 = self.alpha + 2 * self.beta / dt
        C5 = 2 * self.alpha
        C6 = self.alpha - 2 * self.beta / dt

        u_k = np.random.uniform(-1, 1)
        y_k = (
            C4 * u_k
            + C5 * self.u_km1
            + C6 * self.u_km2
            - C2 * self.y_km1
            - C3 * self.y_km2
        ) / C1

        self.u_km2 = self.u_km1
        self.u_km1 = u_k
        self.y_km2 = self.y_km1
        self.y_km1 = y_k

        return y_k

    def integrate(self, dt):
        """ """

        if dt > self.dt:
            t = 0
            y = 0
            while t < dt:
                t_inc = min(self.dt, dt - t)
                y = self.run(t_inc)
                t += t_inc
            return y
        else:
            return self.run(dt)


class DrydenWind:
    """ """

    def __init__(
        self,
        wx_nominal,
        wy_nominal,
        wz_nominal,
        wx_sigma,
        wy_sigma,
        wz_sigma,
        altitude=2.0,
    ):

        self.wx_nominal, self.wy_nominal, self.wz_nominal = (
            wx_nominal,
            wy_nominal,
            wz_nominal,
        )
        self.altitude = altitude

        Lz_ft = 3.281 * altitude
        Lx_ft = Lz_ft / ((0.177 + 0.000823 * Lz_ft) ** (1.2))
        Ly_ft = Lx_ft

        self.wx_gust = GustModelBase(1.0, Lx_ft / 3.281, wx_sigma)
        self.wy_gust = GustModelBase(1.0, Ly_ft / 3.281, wy_sigma)
        self.wz_gust = GustModelBase(1.0, Lz_ft / 3.281, wz_sigma)

        self.initialized = True

        return

    def getWind(self, dt):
        """ """

        if self.initialized:
            wind_vector = np.array(
                [self.wx_nominal, self.wy_nominal, self.wz_nominal]
            ) + np.array(
                [
                    self.wx_gust.integrate(dt),
                    self.wy_gust.integrate(dt),
                    self.wz_gust.integrate(dt),
                ]
            )
        else:
            wind_vector = np.array([0, 0, 0])

        return wind_vector


if __name__ == "__main__":

    dt = 0.05
    x_mean = 0.0
    y_mean = 0.0
    z_mean = 0.0
    x_sigma = 50.0
    y_sigma = 50.0
    z_sigma = 10.0

    rospy.init_node("wind_nodea", anonymous=False)
    pub = rospy.Publisher("/haubentaucher/wind_speed", WindSpeed, queue_size=0)

    speed = WindSpeed()

    # pub = rospy.Publisher("/haubentaucher/wind_speed", WindSpeed, queue_size=10)
    r = rospy.Rate(100)  # 10hz

    while not rospy.is_shutdown():
        t = random.randint(5, 100)
        h = random.uniform(1.5, 12.0)
        on_off = random.choice((0, 1))
        # using the dryden model
        wind = DrydenWind(x_mean, y_mean, z_mean, x_sigma, y_sigma, z_sigma, altitude=h)

        n = int(np.floor(t / dt)) + 1

        for i in range(n):
            w = wind.getWind(dt)
            speed.velocity.x = w[0] * on_off
            speed.velocity.y = w[1] * on_off
            speed.velocity.z = w[2] * on_off
            pub.publish(speed)
            rospy.sleep(dt)
        r.sleep()
