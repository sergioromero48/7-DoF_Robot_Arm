#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import time

def move_joint(pub, position, duration=2):
    """Publish a single joint position and hold for a duration."""
    msg = Float64()
    msg.data = position
    pub.publish(msg)
    time.sleep(duration)  # Wait for the joint to move and settle

def initialize_publishers():
    """Initialize ROS publishers for each joint controller."""
    pubs = []
    # Adjust the range according to your number of joints and naming convention
    for i in range(1, 4):  # Example for 3 joints
        topic = '/marvin_arm/joint{}_position_controller/command'.format(i)
        pubs.append(rospy.Publisher(topic, Float64, queue_size=10))
    return pubs

def test_joints(publishers):
    """Test each joint by moving it to a test position and back in a continuous loop."""
    test_positions = [2, 2, 2]  # Example test positions for three joints
    initial_positions = [0.0, 0.0, 0.0]  # Assuming initial positions are 0

    while not rospy.is_shutdown():
        for pub, test_pos, initial_pos in zip(publishers, test_positions, initial_positions):
            rospy.loginfo("Moving joint to position: {}".format(test_pos))
            move_joint(pub, test_pos)
            rospy.loginfo("Returning joint to initial position: {}".format(initial_pos))
            move_joint(pub, initial_pos)

def main():
    rospy.init_node('robot_arm_joint_tester', anonymous=True)

    # Initialize publishers for each joint
    publishers = initialize_publishers()
    rospy.sleep(1)  # Wait for connections to be established

    # Test each joint sequentially in a loop
    test_joints(publishers)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
