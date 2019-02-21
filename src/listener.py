#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import sys
import rospy
from geometry_msgs.msg import WrenchStamped
import time
import math
import string
from scipy import signal
from butterworth import Butter
import LowPassFiltering
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from std_msgs.msg import Float64

import numpy as np
from pylab import ylim, title, ylabel, xlabel
import matplotlib.pyplot as plt
from kalman import SingleStateKalmanFilter
from moving_average import MovingAverageFilter

A = 1  # No process innovation
C = 1  # Measurement
B = 0  # No control input
Q = 0.005  # Process covariance .005
R = 1.5  # Measurement covariance 1
x = 0  # Initial estimate
P = 1  # Initial covariance

#FX = 0

pub1 = rospy.Publisher('/filtered_dta', Float64, queue_size=10)
pub2 = rospy.Publisher('/originalwrench', Float64,queue_size=10)
pub3 = rospy.Publisher('/kalman',Float64,queue_size=10)

def callback(data):
    #global FX
    time = data.header.stamp.secs  +  data.header.stamp.nsecs * 1e-9
    #print(time)


    fx = data.wrench.force.x
    fy = data.wrench.force.y
    fz = data.wrench.force.z
    tx = data.wrench.torque.x
    ty = data.wrench.torque.y
    tz = data.wrench.torque.z

    #print(fx)

    #FX = []
    #FX.append(fx)


    #fs = 124.956672444 #sampling frequency len(fx)/57.7 subject to change

    #fc = 1 # Cut-off frequency of the filter
    #w = fc / (fs / 2) # Normalize the frequency
    #b, a = signal.butter(5, w, 'low')

    #fx = signal.lfilter(b, a, fx,axis=0) #Forward filter
    #fy = signal.lfilter(b, a, fy,axis=0)
    #fz = signal.lfilter(b, a, fz,axis=0)

    #tx = signal.lfilter(b, a, tx,axis=0)
    #ty = signal.lfilter(b, a, ty,axis=0)
    #tz = signal.lfilter(b, a, tz,axis=0)

    ############################################################################

    #filter_ = Butter(btype="lowpass", cutoff=1, rolloff=120, sampling=124.956672444)
    #filtered_data = filter_.send(FX)
    #print(filtered_data)

    ##################################################################

    #w = LowPassFiltering.LowPassFiltering(999)
    #FX = w.process(fx)
    #print(FX)
    ################################################################
    #readings = []
    #window = 99

    #def mean(nums):
     #   return float(sum(nums))/len(nums)

    #readings.append(fx)

    #if len(readings) == window:
     #   FX.pop(0)
    
    #FX = mean(readings)
    ####################################################################

   
    kalman_filter_fx = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
    kalman_filter_fy = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
    kalman_filter_fz = SingleStateKalmanFilter(A, B, C, x, P, Q, R)

    kalman_filter_tx = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
    kalman_filter_ty = SingleStateKalmanFilter(A, B, C, x, P, Q, R)
    kalman_filter_tz = SingleStateKalmanFilter(A, B, C, x, P, Q, R)

 
    kalman_filter_est_fx = []
    kalman_filter_est_fy = []
    kalman_filter_est_fz = []

    kalman_filter_est_tx = []
    kalman_filter_est_ty = []
    kalman_filter_est_tz = []

    #fx5_filter = MovingAverageFilter(1000)
    #fx50_filter = MovingAverageFilter(50)

    #fx5_filter_est = []
    #fx50_filter_est = []


    #fx5_filter.step(fx)
    #fx5_filter_est.append(fx5_filter.current_state())

    kalman_filter_fx.step(0, fx)
    kalman_filter_fy.step(0, fy)
    kalman_filter_fz.step(0, fz)

    kalman_filter_tx.step(0, tx)
    kalman_filter_ty.step(0, ty)
    kalman_filter_tz.step(0, tz)

    kalman_filter_est_fx.append(kalman_filter_fx.current_state())
    kalman_filter_est_fy.append(kalman_filter_fy.current_state())
    kalman_filter_est_fz.append(kalman_filter_fz.current_state())

    kalman_filter_est_tx.append(kalman_filter_tx.current_state())
    kalman_filter_est_ty.append(kalman_filter_ty.current_state())
    kalman_filter_est_tz.append(kalman_filter_tz.current_state())

    fx_k = kalman_filter_est_fx[0]
    fy_k = kalman_filter_est_fy[0]
    fz_k = kalman_filter_est_fz[0]

    tx_k = kalman_filter_est_tx[0]
    ty_k = kalman_filter_est_ty[0]
    tz_k = kalman_filter_est_tz[0]
    


    #fx50_filter.step(fx)
    #fx50_filter_est.append(fx50_filter.current_state())
    pub1.publish(Float64(fy))
    #pub2.publish(Float64(fx5_filter_est[0]))
    pub3.publish(Float64(kalman_filter_est_fy[0]))


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)

    rospy.Subscriber('/wrench', WrenchStamped, callback)
    #pub.publish('/filtered_dta',Float64(FX))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
