from lib_robotis import *
import sys
import numpy, scipy.io
import time


### r_elbow joint limit range='-1.57 1.3'
### l_elbow joint limit range='-1.3 1.57'

s_ = time.time()

dyn = USB2Dynamixel_Device('/dev/ttyUSB0', 1000000)

argc = len(sys.argv)

if argc < 2:
    print "single_servo.py [lock motor] [update dt] [servo id = 1] [p gain = 32] [d gain = 0]"
    sys.exit(0)

engage = int(sys.argv[1])
update = float(sys.argv[2])

servo_id = 1
if argc > 3:
    servo_id = int(sys.argv[3])

p_gain = 32
if argc > 4:
    p_gain = int(sys.argv[4])

d_gain = 0
if argc > 5:
    d_gain = int(sys.argv[5])


print "Tryign to connect to servo %d" % servo_id
try:
    p = Robotis_Servo(dyn, servo_id, series='MX')
except:
    print "Servo %d not found.\n" % servo_id
    sys.exit(0)


print p.write_p_gain(p_gain)
print p.write_d_gain(d_gain)
print p.set_move_speed(1023)

p.print_summary()

ctrl = numpy.array([])
qpos = numpy.array([])
qvel = numpy.array([])
tm__ = numpy.array([])

print "Init time: %f" % (time.time() - s_)

raw_input("Press Enter to continue...")
print("Running.")
s_ = time.time()

if engage == 0 :
    p.disable_torque()
else:
    p.enable_torque()

# play trajectory while collecting data
while True:

    t_ = time.time() - s_

    #print t_
    # sleep if not caught up?
    #print t, c
    t_ = time.time() - s_
    time.sleep(update)
    q, v, load, volt, temp = p.read_bulk()
    #print 'Time: %f' % (t_ - s_)

    #c = 0

    print "Qpos: %f\tQvel: %f\tload: %d\tvolt: %2.1f\ttemp: %d" % \
            (q, v, load, volt, temp)
    #print "Qvel: %f" % v
    #print "load: %d" % load
    #print "volt: %2.1f" % volt
    #print "temp: %d" % temp




