#!/usr/bin/env python3

import sys
import threading

import rclpy

from cyber_msgs.msg import CyberarmTarget4D

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages. It works best with a US keyboard layout.
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 1, 0, 0),
    'd': (0, -1, 0, 0),
    'k': (0, 0, -1, 0),
    'i': (0, 0, 1, 0),
    'q': (0, 0, 0, -1),
    'e': (0, 0, 0, 1),
}


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)



def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_keyboard')

    # parameters
    frame_id = node.declare_parameter('frame_id', 'base_link').value

    pub = node.create_publisher(CyberarmTarget4D, 'ctrl/target4d', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    y = 0.0
    z = 0.5
    t = 0.0
    vx = 0.0
    vy = 0.0
    vz = 0.0
    vt = 0.0

    msg = CyberarmTarget4D()

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                vx, vy, vz, vt = moveBindings[key]
            else:
                vx = 0.0
                vy = 0.0
                vz = 0.0
                if (key == '\x03'):
                    break

            x += vx * 0.005
            y += vy * 0.005
            z += vz * 0.005
            t += vt * 0.005

            msg.point.x = x
            msg.point.y = y
            msg.point.z = z
            msg.tilt = t

            pub.publish(msg)

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
