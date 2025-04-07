#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Save terminal settings
settings = termios.tcgetattr(sys.stdin)

def getKey():
    """Capture a single key press from the keyboard."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main():
    rospy.init_node('teleop_keyboard_accumulative')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    # Start with zero velocities
    linear_velocity = 0.0
    angular_velocity = 0.0

    # Define increments for each key press
    linear_increment = 0.5   # Increase in m/s per press for linear velocity
    angular_increment = 0.5  # Increase in rad/s per press for angular velocity

    print("Control your robot!")
    print("Press:")
    print("  w : Increase forward velocity")
    print("  s : Increase backward velocity")
    print("  a : Increase left angular velocity")
    print("  d : Increase right angular velocity")
    print("  SPACE or k : Reset velocities to zero")
    print("Press CTRL+C to quit.")

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key == 'w':
                linear_velocity += linear_increment
            elif key == 's':
                linear_velocity -= linear_increment
            elif key == 'a':
                angular_velocity += angular_increment
            elif key == 'd':
                angular_velocity -= angular_increment
            elif key == ' ' or key == 'k':
                linear_velocity = 0.0
                angular_velocity = 0.0
            elif key == '\x03':  # CTRL+C
                break

            # Set the twist message with current accumulative values
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity

            # Publish the Twist message
            pub.publish(twist)

            # Monitor by printing the current twist command
            rospy.loginfo("Published Twist -> linear.x: %.2f, angular.z: %.2f", twist.linear.x, twist.angular.z)

            rate.sleep()

    except Exception as e:
        print("Exception: ", e)

    finally:
        # On shutdown, reset the robot to stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()
