# Servo Control
import time
#import wiringpi
import math
#import settings
# use 'GPIO naming'
#wiringpi.wiringPiSetupGpio()
import rospy
from std_msgs.msg import Float64 #need for publishing
from beginner_tutorials.msg import Angles 

#Define min and max servo position (pwm pulse sizes)
s_min = 8
s_max = 32

# set the PWM mode to milliseconds stype
#wiringpi.pwmSetMode(wiringpi.GPIO.PWM_MODE_MS)
 
# divide down clock
#wiringpi.pwmSetClock(192)
#wiringpi.pwmSetRange(2000)

#Define position of servos

#Constants
pi = 3.14159
deg2rad = pi/180
deg30 = pi/6

pub_angles = rospy.Publisher('/angles',Angles,queue_size=10)

pub_ang1 = rospy.Publisher('/angle1',Float64,queue_size=10)
pub_ang2 = rospy.Publisher('/angle2',Float64,queue_size=10)
pub_ang3 = rospy.Publisher('/angle3',Float64,queue_size=10)
pub_ang4 = rospy.Publisher('/angle4',Float64,queue_size=10)
pub_ang5 = rospy.Publisher('/angle5',Float64,queue_size=10)
pub_ang6 = rospy.Publisher('/angle6',Float64,queue_size=10)


#Servo Array objects
servo_array = ([0,0,0,0,0,0])
#Zero positions of servos
servo_zeros = ([0,0,0,0,0,0])
#Desired servo position on platform 6x6 x,y,z, rotx, roty, rotz
servo_des = ([0,0,0,0,0,0])
#Servo initial angle theta_a
theta_a = ([0,0,0,0,0,0])
#Servo position
servo_pos = ([0,0,0,0,0,0])
#Rotation of Servo arm in respect to x axis
#great for x y rotation as translation
beta = ([-pi/2, pi/2, 5*pi/6, -pi/6, pi/6, -5*pi/6])
#beta = ([pi, 0, pi/3, -2*pi/3, -pi/3, 2*pi/3])
#beta = ([pi/2,-pi/2,-pi/6, 5*pi/6,-5*pi/6,pi/6])
servo_min = -pi/3
servo_max = pi/3
L1 = 25.4
L2 = 115
#L2 = 171
z_home = 110
#values are from technical documentation of the servo, it's pulse length and the resulting angle of rotation corresponding to this pulse length
servo_mult = 400/(pi/4)
PD = 70
RD = 83
theta_p = 43.7 * deg2rad
theta_r = 8.16 * deg2rad
theta_angle = (pi/3-theta_p)/2
p = ([[-PD*math.cos(deg30-theta_angle),-PD*math.cos(deg30-theta_angle),PD*math.sin(theta_angle),PD*math.cos(deg30+theta_angle),PD*math.cos(deg30+theta_angle),PD*math.sin(theta_angle)],[-PD*math.sin(deg30-theta_angle), PD*math.sin(deg30-theta_angle), PD*math.cos(theta_angle),PD*math.sin(deg30+theta_angle),-PD*math.sin(deg30+theta_angle),-PD*math.cos(theta_angle)]])
re = ([[-RD*math.sin(deg30+theta_r/2),-RD*math.sin(deg30+theta_r/2),-RD*math.sin(deg30-theta_r/2),RD*math.cos(theta_r/2),RD*math.cos(theta_r/2),-RD*math.sin(deg30-theta_r/2)],[-RD*math.cos(deg30+theta_r/2),RD*math.cos(deg30+theta_r/2),RD*math.cos(deg30-theta_r/2),RD*math.sin(theta_r/2),-RD*math.sin(theta_r/2),-RD*math.cos(deg30-theta_r/2)],[0,0,0,0,0,0]])
#Intialize Servos

#function calculating needed servo rotation value
rxp = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]
q = [0,0,0]
M = [[0,0,0],[0,0,0],[0,0,0]]
T = [0,0,0]
H = [0,0,z_home]
def getAlpha(i):
    n = 0
    #print ("Servo : " + str(i))
    th = theta_a[i]
    dl = [0,0,0]
    Min = servo_min
    Max = servo_max
    while n<20:
        th = Min + (Max-Min)/2
        q[0] = L1 * math.cos(th) * math.cos(beta[i]) + p[0][i]
        q[1] = L1 * math.cos(th) * math.sin(beta[i]) + p[1][i]
        q[2] = L1 * math.sin(th)
        dl[0] = rxp[0][i] - q[0]
        dl[1] = rxp[1][i] - q[1]
        dl[2] = rxp[2][i] - q[2]
        dl2 = math.sqrt(dl[0] * dl[0] + dl[1] * dl[1] + dl[2] * dl[2])

        #print("Min: " + str(Min * 180 / pi) + " Max: " + str(Max * 180 / pi) + " th: " + str(th * 180 / pi) + " dl2: " + str(dl2))
        if abs(L2-dl2)<.01:
            return th
        if dl2<L2:
            Max = th
        else:
            Min = th
        n += 1
        if Max == servo_min or Min == servo_max:
            return th

    return th
#Function calculating rotation matrix
def getmatrix(servo_des):
    psi = servo_des[5]
    theta = servo_des[4]
    phi = servo_des[3]
    M[0][0] = math.cos(psi) * math.cos(theta)
    M[1][0] = -math.sin(psi) * math.cos(phi) + math.cos(psi) * math.sin(theta) * math.sin(phi)
    M[2][0] = math.sin(psi) * math.sin(phi) + math.cos(psi) * math.cos(phi) * math.sin(theta)

    M[0][1] = math.sin(psi) * math.cos(theta)
    M[1][1] = math.cos(psi) * math.cos(phi) + math.sin(psi) * math.sin(theta) * math.sin(phi)
    M[2][1] = math.cos(theta) * math.sin(phi)

    M[0][2] = -math.sin(theta)
    M[1][2] = -math.cos(psi) * math.sin(phi) + math.sin(psi) * math.sin(theta) * math.cos(phi)
    M[2][2] = math.cos(theta) * math.cos(phi)
    return

def getrxp():
    for i in range (0,6):
        rxp[0][i] = T[0] + M[0][0] * (re[0][i]) + M[0][1] * (re[1][i]) + M[0][2] * (re[2][i])
        rxp[1][i] = T[1] + M[1][0] * (re[0][i]) + M[1][1] * (re[1][i]) + M[1][2] * (re[2][i])
        rxp[2][i] = T[2] + M[2][0] * (re[0][i]) + M[2][1] * (re[1][i]) + M[2][2] * (re[2][i])
    return

def getT(pe):
    T[0] = pe[0] + H[0]
    T[1] = pe[1] + H[1]
    T[2] = pe[2] + H[2]
    return

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def setPos(servo_des):
    #errors = 0
    global theta_a
    for i in range (0,6):
        getT(servo_des)
        getmatrix(servo_des)
        getrxp()
        theta_a[i]=getAlpha(i)

        pub_angles.publish(Angles(theta_a[0],theta_a[1],theta_a[2],theta_a[3],theta_a[4],theta_a[5]))


        pub_ang1.publish(Float64(theta_a[0]*180/pi))
        pub_ang2.publish(Float64(theta_a[1]*180/pi))
        pub_ang3.publish(Float64(theta_a[2]*180/pi))

        pub_ang4.publish(Float64(theta_a[3]*180/pi))
        pub_ang5.publish(Float64(theta_a[4]*180/pi))
        pub_ang6.publish(Float64(theta_a[5]*180/pi))

        if i == 0 or i == 2 or i == 4:
            servo_pos[i] = clamp(servo_zeros[i] - theta_a[i], servo_min, servo_max)
        else:
            servo_pos[i] = clamp(servo_zeros[i] + theta_a[i], servo_min, servo_max)
    #for i in range (0,6):
    #    if (theta_a[i] == servo_min or theta_a[i] == servo_max or servo_pos[i] == s_min or servo_pos[i] == s_max):
    #        errors += 1
        #settings.servos[i].moveAngle(servo_pos[i]*180/pi)
        #print(str(servo_pos[i]* 180 / pi))
    return 
