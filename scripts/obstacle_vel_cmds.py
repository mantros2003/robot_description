#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node("obstacle_controller")

    pub = rospy.Publisher("/obs_cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    speed = 0.3           # m/s
    interval = 4.0        # seconds

    direction = 1
    last_switch = rospy.Time.now()

    while not rospy.is_shutdown():

        if (rospy.Time.now() - last_switch).to_sec() >= interval:
            direction *= -1
            last_switch = rospy.Time.now()

        cmd = Twist()
        cmd.linear.y = direction * speed

        pub.publish(cmd)
        rate.sleep()


if __name__ == "__main__":
    main()
