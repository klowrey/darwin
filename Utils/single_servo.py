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
    print "single_servo.py [outfile] [servo id = 1] [p gain = 32] [d gain = 0]"
    sys.exit(0)

output_file = sys.argv[1]

servo_id = 1
if argc >= 3:
    servo_id = int(sys.argv[2])

p_gain = 32
if argc >= 4:
    p_gain = int(sys.argv[3])

d_gain = 0
if argc >= 5:
    d_gain = int(sys.argv[4])


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

#f = open(output_file, 'w')
#f.write("ctrl,qpos,qvel,load,volt,temp\n") # header

#traj = open(input_file, 'r')
#input_file = '60_sec.csv'
input_file = 'traj_left.csv'
traj = (numpy.loadtxt(open(input_file, 'r'), delimiter=",", skiprows=0)).T

#print traj[0]
#print traj[1]
#print len(traj)

ctrl = numpy.array([])
qpos = numpy.array([])
qvel = numpy.array([])
tm__ = numpy.array([])

print "Init time: %f" % (time.time() - s_)

c = traj[1,0]
p.move_angle(c, blocking=True)

raw_input("Press Enter to continue...")
print("Running.")
s_ = time.time()

q, v, load, volt, temp = p.read_bulk()
ctrl = numpy.append(ctrl, [c])
qpos = numpy.append(qpos, [q])
qvel = numpy.append(qvel, [v])
tm__ = numpy.append(tm__, [0])

# play trajectory while collecting data
for t, c in zip(traj[0,1:], traj[1,1:]):
#while (time.time() - s_) < 0.2:

    t_ = time.time() - s_
    print t_, c
    p.move_angle(c)

    # sleep if not caught up?
    while t_ < t:
        #print t, c
        t_ = time.time() - s_
        #time.sleep(t-t_)
        q, v, load, volt, temp = p.read_bulk()
        ctrl = numpy.append(ctrl, [c])
        qpos = numpy.append(qpos, [q])
        qvel = numpy.append(qvel, [v])
        tm__ = numpy.append(tm__, [t_])

    #print 'Time: %f' % (t_ - s_)

    #c = 0

        #print "Qpos: %f" % q
    #print "Qvel: %f" % v
    #print "load: %d" % load
    #print "volt: %2.1f" % volt
    #print "temp: %d" % temp


print "Loop time: %f" % (time.time() - s_)

s_ = time.time()

# to csv file
#a = (numpy.vstack([ctrl,qpos,qvel])).T
#numpy.savetxt(output_file, a, delimiter=",", header="ctrl,qpos,qvel,load,volt,temp")

# to .mat file
scipy.io.savemat(output_file, oned_as='row', \
        mdict={'Q':qpos[1:],'C':ctrl[1:],'V':qvel[1:],'Y':qvel[0:],'T':tm__[1:]})
# still need Y, U, and T

p.disable_torque()

dyn.close_serial()

print "Save time: %f" % (time.time() - s_)

