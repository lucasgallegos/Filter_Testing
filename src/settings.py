
import sys
import pigpio

from controlClass import control
from servoClass import servo
from vibrationClass import vibration

def init():
	#ex:

	#global varName
	#varName = varNameClass()

	#import settings into other scripts
	#refer to global variable by "settings.varName"

	global mainControl
	mainControl = control()
	
	servo1 = servo(4 ,0, 7, 32, "right")
	servo2 = servo(17 ,1, 7, 32, "left")
	servo3 = servo(18, 2, 7, 32, "right")
	servo4 = servo(27, 3, 7, 32, "left")
	servo5 = servo(22, 4, 7, 32, "right")
	servo6 = servo(23, 5, 7, 32, "left")
	
	global servos
	servos = [servo1, servo2, servo3, servo4, servo5, servo6]
	
	piover4 = 3.14159/4	

	vibration1 = vibration(24 ,0, 0, 100, 0 * piover4)
	vibration2 = vibration(25 ,1, 0, 100, 1 * piover4)
	vibration3 = vibration(5, 2, 0, 100, 2 * piover4)
	vibration4 = vibration(6, 3, 0, 100, 3 * piover4)
	vibration5 = vibration(13, 4, 0, 100, 4 * piover4)
	vibration6 = vibration(19, 5, 0, 100, - 3 * piover4)
	vibration7 = vibration(26, 6, 0, 100, - 2 * piover4) 
	vibration8 = vibration(16, 7, 0, 100, - 1 * piover4)
	
	global vibrations
	vibrations = [vibration1, vibration2, vibration3, vibration4, vibration5, vibration6, vibration7, vibration8]
