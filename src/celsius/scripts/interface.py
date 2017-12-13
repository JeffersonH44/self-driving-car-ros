#!/usr/bin/env python
import rospy
from can_msgs.msg import Frame
from celsius.msg import CelsiusReport, CelsiusControl
from struct import unpack


def parse_temperature(raw_temp):
    if raw_temp[1] == 73:
        temp = 'HI'
    elif raw_temp[1] == 79:
        temp = 'LO'
    else:
        lower = raw_temp[1] & 0x0F
        higher = raw_temp[0] & 0x0F
        temp = '%s%s' % (higher, lower) 

    return temp


def parse(frame, ac=CelsiusReport()):
    if frame.id == 0x356:

        raw_data = unpack('BBBBBBBB', frame.data)
        ac.data1 = raw_data[0]
        ac.fan_speed = raw_data[1] / 4
        ac.data7 = raw_data[6]
        ac.data8 = raw_data[7]
 
        ac.driver_temp = parse_temperature(raw_data[2:4])
        ac.passenger_temp = parse_temperature(raw_data[4:6])

        if ac.driver_temp == '00':
            ac.system_on = False
        else:
            ac.system_on = True 

    elif frame.id == 0x355:
        raw_data = unpack('BBBBBBBB', frame.data)

        if raw_data[0] == 97:
            ac.dual = False
            ac.unit_on = True
        elif raw_data[0] == 96:
            ac.dual = True
            ac.unit_on = True
        elif raw_data[0] == 32:
            ac.dual = False
            ac.unit_on = False

        ac.defrost_fan0 = raw_data[1]
        ac.defrost_fan1 = raw_data[2]

        if raw_data[2] == 61:
            ac.auto = True
        else:
            ac.auto = False

        ac.data12 = raw_data[3]

        if raw_data[4] == 4:
            ac.max_defrost = True
        else:
            ac.max_defrost = False

        if raw_data[4] == 16:
            ac.max_cool = True
        else:
            ac.max_cool = False


        if raw_data[5] == 152:
            ac.feet_fan = True
            ac.head_fan = True
            ac.recirculation = True

        if raw_data[5] == 137:
            ac.feet_fan = True
            ac.head_fan = False
            ac.recirculation = True
        elif raw_data[5] == 136:
            ac.feet_fan = True
            ac.head_fan = False
            ac.recirculation = False
        else:
            ac.feet_fan = False
            ac.head_fan = False
            ac.recirculation = False
 
        if raw_data[6] == 0:
            ac.front_defrost = False
        elif raw_data[6] == 4:
            ac.front_defrost = True
        
        ac.defrost_1 = raw_data[5]
        ac.defrost_2 = raw_data[6]
        ac.data16 = raw_data[7]
    else:
        return None

    #rospy.loginfo(rospy.get_caller_id() + "%s", ac)

    return ac


def report_callback(frame):

    # Parse AC report into a ROS message.
    ac = parse(frame)

    # If parse returns None, then it is not a temperature message
    # and the callback function should terminate / return.
    if not ac:
        return

    #rospy.loginfo("An AC frame was received on the can bus: %s", frame)

    # Set up a topic to report based on the parameters defined at launch.
    report_topic = rospy.get_param('~report_topic')
    #rospy.loginfo("%s is %s", rospy.resolve_name('~report_topic'), report_topic)
    socketcan_pub = rospy.Publisher(report_topic, CelsiusReport, queue_size=10)

    # Publish the CelsiusReport message on the configured topic name.
    socketcan_pub.publish(ac)

    #rospy.loginfo("A CelsiusReport was sent on topic %s: %s", ac, report_topic)


CONTROL_CODES = {
    'ac_toggle': 0x5C,
    'ac_unit_toggle': 0x14,
    'max_ac_toggle': 0x38,
    'recirculation_toggle': 0x3C,
    'dual_temperature_toggle': 0x18,
    'passenger_temp_up': 0x24,
    'passenger_temp_down': 0x28,
    'driver_temp_up': 0x1C,
    'driver_temp_down': 0x20,
    'auto': 0x34,
    'wheel_heat_toggle': 0x78,
    'defrost_max_toggle': 0x64,
    'defrost_toggle': 0x4C,
    'rear_defrost_toggle': 0x58,
    'body_fan_toggle': 0x04,
    'feet_fan_toggle': 0x0C,
    'fan_up': 0x2C,
    'fan_down': 0x30,
}


def control_callback(control, can_device='can0'):
    
    rospy.loginfo("A control command %s was received on control topic", control)
    # not implemented yet, just send something to verify it is communicating with the can bus.

    # Note: sent_messages is the ROS topic where socketcan_bridge expects messages,
    # since it is not configurable, it is hardcoded here.
    socketcan_pub = rospy.Publisher('sent_messages', Frame, queue_size=1)

    command = control.command
    # Do not do anything if the command is invalid.
    if command not in CONTROL_CODES:
        rospy.loginfo("Command %s not found in %s", command, CONTROL_CODES.keys())
        return

    # If it is a valid command, let's get the hex code.
    code = CONTROL_CODES[command]

    rospy.loginfo("Command %s (%s) is being sent to the car", command, code)

    # Plan B.
    if can_device:
        from subprocess import call
        call(['/usr/bin/cansend', can_device, '358#00%02x000000' % code])
        return

    frame = Frame()
    frame.id = 0x358
    frame.data = [0, code, 0, 0, 0, 0, 0, 0]
    frame.dlc = 8
    frame.is_rtr = False
    frame.is_extended = False
    frame.is_error = False
    rospy.loginfo(frame)
    socketcan_pub.publish(frame)

    rospy.loginfo("A can frame was sent to 'sent_messages': %s", frame)


def main():
    rospy.init_node('celsius')

    control_topic = rospy.get_param('~control_topic')
    rospy.loginfo("%s is %s", rospy.resolve_name('~control_topic'), control_topic)
    
    # Subscribe to the control topic, parse messages and send them to the CAN interface.
    rospy.Subscriber(control_topic, CelsiusControl, control_callback)

    # Subscribe to the report topic and republish parsed messages as a CelsiusReport
    # Note: received_messages is the ROS topic where socketcan_bridge expects messages
    # since it is not configurable, it is hardcoded here.
    rospy.Subscriber("received_messages", Frame, report_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
