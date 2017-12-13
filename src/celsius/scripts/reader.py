#!/usr/bin/env python
import rospy
import re

from can_msgs.msg import Frame
from struct import unpack
from binascii import unhexlify
from std_msgs.msg import Float64, UInt8

class Enumeration(set):
    def __getattr__(self, name):
        if name in self:
            return name
        raise AttributeError
    def __setattr__(self, name, value):
        raise RuntimeError("Cannot override values")
    def __delattr__(self, name):
        raise RuntimeError("Cannot delete values")

Constants = Enumeration(['TYPE', 'TOPIC', 'START', 'SIZE', 'ENDIAN', 'SIGN', 'SCALE', 'OFFSET', 'MIN', 'MAX', 'UNITS'])

MAX_SIZE_BITS = 64

TOPICS = {
    514: {
        'SPEED': {
            Constants.TYPE: Float64
        }
    },
    145: {
        'NEW_SIGNAL_2': {
            Constants.TYPE: Float64
        },
        '_DIRECTION': {
            Constants.TYPE: Float64
        }
    },
    125: {
        'FULL_BRAKES': {
            Constants.TYPE: UInt8
        },
        'BRAKE_THROTTLE': {
            Constants.TYPE: UInt8
        }
    }
}

file = '/home/jefferson/sdc-start/src/celsius/scripts/ford.dbc'

def create_topics():
    rospy.init_node('publisher', anonymous=True)
    for topic in TOPICS.keys():
        current_topic = TOPICS[topic]
        for name in TOPICS[topic].keys():
            print 'current topic name', name
            current_topic[name][Constants.TOPIC] = rospy.Publisher(name.strip('_'), current_topic[name][Constants.TYPE], queue_size=10)

def parse_number(data, options):
    hex_num = ''.join(data).encode('hex')
    bits = int(hex_num, 16)
    spec = '{fill}{align}{width}{type}'.format(fill='0', align='>', width=64, type='b')
    bits = format(bits, spec)
    ## bits = bin(int(''.join(data).encode('hex'), 16))
    output = {}
    for elem in options.keys():
        current_settings = options[elem]

        start = current_settings[Constants.START]
        end = start + current_settings[Constants.SIZE]
        current_bits = bits[start:(end if end < MAX_SIZE_BITS else MAX_SIZE_BITS)]

        if(current_settings[Constants.ENDIAN] == 1):
            current_bits = current_bits[::-1]
    
        output[elem] = int(current_bits, 2) * current_settings[Constants.SCALE]
        current_settings[Constants.TOPIC].publish(output[elem])
    return output

def import_format(lines):
    bo_regexp = re.compile("^BO\_ (\w+) (\w+) *: (\w+) (\w+)")
    sg_regexp = re.compile("^SG\_ (\w+) : (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*)")
    sgm_regexp = re.compile("^SG\_ (\w+) (\w+) *: (\d+)\|(\d+)@(\d+)([\+|\-]) \(([0-9.+\-eE]+),([0-9.+\-eE]+)\) \[([0-9.+\-eE]+)\|([0-9.+\-eE]+)\] \"(.*)\" (.*)")

    i = 0
    while i < len(lines):
        current_line = lines[i].strip()
        i += 1
        if current_line.startswith('BO_ '):
            print current_line
            data = bo_regexp.match(current_line)
            id = int(data.group(1))
            if id in TOPICS.keys():
                print "process id:", id
                current_dic = TOPICS[id]
                while current_line != '':
                    current_line = lines[i].strip()
                    i += 1
                    print current_line
                    if current_line.startswith('SG_'):
                        data = sg_regexp.match(current_line)
                        elem = data.group(1)
                        if elem in current_dic.keys():
                            to_fill = current_dic[elem]
                            to_fill[Constants.START] = int(data.group(2))
                            to_fill[Constants.SIZE] = int(data.group(3))
                            to_fill[Constants.ENDIAN] = int(data.group(4))
                            to_fill[Constants.SIGN] = data.group(5)
                            to_fill[Constants.SCALE] = float(data.group(6))
                            to_fill[Constants.OFFSET] = float(data.group(7))
                            to_fill[Constants.MIN] = float(data.group(8))
                            to_fill[Constants.MAX] = float(data.group(9))
                            to_fill[Constants.UNITS] = data.group(10)
    print TOPICS
                            
def callback(data):
    if data.id in TOPICS.keys():
        hex = data.data
        print parse_number(hex, TOPICS[data.id])
        # print data.id, int(''.join(hex).encode('hex')[-4:], 16) * 0.01
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", ''.join(output.data).encode('hex'))
    else:
        pass
        #print data.id
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("received_messages", Frame, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    with open(file) as f:
        content = f.readlines()
        import_format(content)
    create_topics()
    listener()