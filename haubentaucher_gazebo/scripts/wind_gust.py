# Dryden Gust Model
# Mid size modelling
# Date: 10th Nov 2023
# ----------------------------------------------------------#
# Module import
#! /usr/bin/env python3
import csv
import math
import random

import numpy as np
import rospy
import scipy.io as sio
from matplotlib import pyplot as plt
from matplotlib.colors import Normalize
from rotors_comm.msg import *
from scipy import signal
from scipy.signal import butter, firwin, freqz, lfilter


# Low altitude model
# Transfer function for along-wind
def u_transfer_function(height, airspeed):
    # turbulence level defines value of wind speed in knots at 20 feet
    # turbulence_level = 15 * 0.514444
    # convert speed from knots to meters per second
    turbulence_level = 15
    length_u = height / ((0.177 + 0.000823 * height) ** (0.2))
    # length_u = 1750
    sigma_w = 0.1 * turbulence_level
    sigma_u = sigma_w / ((0.177 + 0.000823 * height) ** (0.4))
    num_u = [sigma_u * (math.sqrt((2 * length_u) / (math.pi * airspeed))) * airspeed]
    den_u = [length_u, airspeed]
    phi_u = signal.TransferFunction(num_u, den_u)
    return phi_u


# Transfer function for vertical-wind
def v_transfer_function(height, airspeed):
    # turbulence level defines value of wind speed in knots at 20 feet
    # turbulence_level = 15 * 0.514444
    # convert speed from knots to meters per second
    turbulence_level = 15
    length_v = height / ((0.177 + 0.000823 * height) ** (0.2))
    # length_v = 1750
    sigma_w = 0.1 * turbulence_level
    sigma_v = sigma_w / ((0.177 + 0.000823 * height) ** (0.4))
    b = sigma_v * (math.sqrt((length_v) / (math.pi * airspeed)))
    Lv_V = length_v / airspeed
    num_v = [(math.sqrt(3) * Lv_V * b), b]
    den_v = [(Lv_V**2), 2 * Lv_V, 1]
    phi_v = signal.TransferFunction(num_v, den_v)
    return phi_v


def w_transfer_fucntion(height, airspeed):
    # turbulence level defines value of wind speed in knots at 20feet
    # turbulence_level = 15 * 0.514444
    # convert speed from knots to meters per second
    turbulence_level = 15
    length_w = height
    sigma_w = 0.1 * turbulence_level
    c = sigma_w * (math.sqrt((length_w) / (math.pi * airspeed)))
    Lw_V = length_w / airspeed
    num_w = [(math.sqrt(3) * Lw_V * c), c]
    den_w = [(Lw_V**2), 2 * Lw_V, 1]
    H_v = signal.TransferFunction(num_w, den_w)
    return H_v


# dryden wind model forming


def dryden_wind_velocities(time, height, airspeed):
    mean = 0 
    std = 1

    t_p = np.linspace(0, time, 1000)
    num_sample = 1000

    # random number seed use the same as from the simulink in matlab
    # ug
    np.random.seed(23341)
    sample_1 = 10 * np.random.normal(mean, std, size=num_sample)
    # vg
    np.random.seed(23342)
    sample_2 = 10 * np.random.normal(mean, std, size=num_sample)
    # wg
    np.random.seed(23343)
    sample_3 = 10 * np.random.normal(mean, std, size=num_sample)

    # transfer velocities
    tf_u = u_transfer_function(height, airspeed)
    tf_v = v_transfer_function(height, airspeed)
    tf_w = w_transfer_fucntion(height, airspeed)

    # compute the response to transfer function
    tout1, yo1, xo1 = signal.lsim(tf_u, sample_1, t_p)
    yo1_f = [i * 0.305 for i in yo1]
    tout2, yo2, xo2 = signal.lsim(tf_v, sample_2, t_p)
    yo2_f = [i * 0.305 for i in yo2]
    tout3, yo3, xo3 = signal.lsim(tf_w, sample_3, t_p)
    yo3_f = [i * 0.305 for i in yo3]
    return yo1_f, yo2_f, yo3_f

    # main program


if __name__ == "__main__":

    rospy.init_node("wind_nodea", anonymous=False)
    pub = rospy.Publisher('/haubentaucher/wind_speed', WindSpeed, queue_size=0)    

    speed = WindSpeed()

    # pub = rospy.Publisher("/haubentaucher/wind_speed", WindSpeed, queue_size=10)
    r = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        t = random.randint(5, 10)
        h = random.uniform(1.5, 12.)
        V = random.uniform(0, 5)
        on_off = random.choice((0,1))
        # using the dryden model
        x, y, z = dryden_wind_velocities(t, h, V)

        #speed = WindSpeed()
        for xi, yi, zi in zip(x, y, z):
            speed.velocity.x = xi*on_off
            speed.velocity.y = yi*on_off
            speed.velocity.z = zi*on_off
            pub.publish(speed)
            rospy.sleep(t/1000)
        r.sleep()
