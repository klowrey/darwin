from lib_robotis import *
import sys
import numpy, scipy.io
import time


### r_elbow joint limit range='-1.57 1.3'
### l_elbow joint limit range='-1.3 1.57'

s_ = time.time()

dyn = USB2Dynamixel_Device('/dev/ttyUSB0', 1000000)

argc = len(sys.argv)

if argc < 3:
    print "single_servo.py [infile] [outfile] [servo id = 1] [p gain = 32] [i gain = 0] [d gain = 0]"
    sys.exit(0)

input_file = sys.argv[1]
output_file = sys.argv[2]

servo_id = 1
if argc > 3:
    servo_id = int(sys.argv[3])

p_gain = 32
if argc > 4:
    p_gain = int(sys.argv[4])

i_gain = 0
if argc > 5:
    i_gain = int(sys.argv[5])

print argc
d_gain = 0
if argc > 6:
    d_gain = int(sys.argv[6])


print "Trying to connect to servo %d" % servo_id
try:
    p = Robotis_Servo(dyn, servo_id, series='MX')
except:
    print "Servo %d not found.\n" % servo_id
    sys.exit(0)


print p.write_p_gain(p_gain)
print p.write_i_gain(i_gain)
print p.write_d_gain(d_gain)
#print p.set_move_speed(1023)
print p.set_move_speed(0)
#print p.set_move_speed(10)


p.print_summary()

#f = open(output_file, 'w')
#f.write("ctrl,qpos,qvel,load,volt,temp\n") # header

#traj = open(input_file, 'r')
#input_file = '60_sec.csv'
#input_file = 'traj.csv'
traj = (numpy.loadtxt(open(input_file, 'r'), delimiter=",", skiprows=0)).T

max_time = traj[0, (traj.size / 2) - 1]
print "data time: %d" % max_time


#print traj[0]
#print traj[1]
#print len(traj)

ctrl = numpy.zeros((1,max_time/0.001))
qpos = numpy.zeros((1,max_time/0.001))
qvel = numpy.zeros((1,max_time/0.001))
load = numpy.zeros((1,max_time/0.001))
volt = numpy.zeros((1,max_time/0.001))
temp = numpy.zeros((1,max_time/0.001))
tm__ = numpy.zeros((1,max_time/0.001))

print numpy.shape(ctrl)

print "Init time: %f" % (time.time() - s_)

c = traj[1,0]
p.move_angle(c, blocking=True)

raw_input("Press Enter to continue...")
print("Running.")
s_ = time.time()

q, v, l, vo, te = p.read_bulk()
idx = 0
ctrl = numpy.insert(ctrl, idx, [c])
qpos = numpy.insert(qpos, idx, [q])
qvel = numpy.insert(qvel, idx, [v])
load = numpy.insert(load, idx, [l])
volt = numpy.insert(volt, idx, [vo])
temp = numpy.insert(temp, idx, [te])
tm__ = numpy.insert(tm__, idx, [0])

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
        q, v, l, vo, te = p.read_bulk()
        idx = idx + 1
        ctrl = numpy.insert(ctrl, idx, [c])
        qpos = numpy.insert(qpos, idx, [q])
        qvel = numpy.insert(qvel, idx, [v])
        load = numpy.insert(load, idx, [l])
        volt = numpy.insert(volt, idx, [vo])
        temp = numpy.insert(temp, idx, [te])
        tm__ = numpy.insert(tm__, idx, [t_])

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

#ctrl = numpy.trim_zeros(ctrl, trim='b')
#qpos = numpy.trim_zeros(qpos, trim='b')
#qvel = numpy.trim_zeros(qvel, trim='b')
#load = numpy.trim_zeros(load, trim='b')
#volt = numpy.trim_zeros(volt, trim='b')
#temp = numpy.trim_zeros(temp, trim='b')
#tm__ = numpy.trim_zeros(tm__, trim='b')
print idx
print numpy.shape(tm__)
ctrl = numpy.delete(ctrl, numpy.s_[idx:])
qpos = numpy.delete(qpos, numpy.s_[idx:])
qvel = numpy.delete(qvel, numpy.s_[idx:])
load = numpy.delete(load, numpy.s_[idx:])
volt = numpy.delete(volt, numpy.s_[idx:])
temp = numpy.delete(temp, numpy.s_[idx:])
tm__ = numpy.delete(tm__, numpy.s_[idx:])
print numpy.shape(tm__)


# to .mat file
scipy.io.savemat(output_file, oned_as='row', \
        mdict={'Q':qpos[0:],'C':ctrl[0:], \
        'V':qvel[0:],'Y':qvel[1:],'T':tm__[0:], \
        'lo':load[0:],'volt':volt[0:],'temp':temp[0:]})
# still need Y, U, and T

p.print_summary()

p.disable_torque()

dyn.close_serial()

print "Save time: %f" % (time.time() - s_)

