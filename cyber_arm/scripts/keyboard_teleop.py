#!/usr/bin/env python3

import sys
import threading

import geometry_msgs.msg
import rclpy

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
    'w': (1, 0, 0),
    's': (-1, 0, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'k': (0, 0, -1),
    'i': (0, 0, 1),
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

    pub = node.create_publisher(geometry_msgs.msg.PointStamped, 'ctrl/target3d', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    x = 0.0
    y = 0.0
    z = 0.5
    vx = 0.0
    vy = 0.0
    vz = 0.0

    msg = geometry_msgs.msg.PointStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = frame_id

    try:
        print(msg)
        while True:
            key = getKey(settings)
            if key in moveBindings.keys():
                vx = moveBindings[key][0]
                vy = moveBindings[key][1]
                vz = moveBindings[key][2]
            else:
                vx = 0.0
                vy = 0.0
                vz = 0.0
                if (key == '\x03'):
                    break

            x += vx * 0.005
            y += vy * 0.005
            z += vz * 0.005

            msg.header.stamp = node.get_clock().now().to_msg()

            msg.point.x = x
            msg.point.y = y
            msg.point.z = z

            pub.publish(msg)

    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
