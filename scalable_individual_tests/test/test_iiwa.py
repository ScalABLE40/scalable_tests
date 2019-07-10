#!/usr/bin/env python
import rospy

import sys
import unittest
from sensor_msgs.msg import JointState

PKG = 'integration_tests'
NAME = 'test_integration_1'

## A sample python unit test
class TestIIWAStatus(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('testing_node')

    def _check_joint_status(self, target=[], delta=0.03):
        message = rospy.wait_for_message("/iiwa/joint_states", JointState, 0.5)
        self.assertEqual(len(message.position), 7)
        for position in message.position:
            self.assertAlmostEqual(position, 0, delta=0.1)

    def setUp(self):
        rospy.wait_for_message("/iiwa/joint_states", JointState)

    def test_joint_status(self):
        message = rospy.wait_for_message("/iiwa/joint_states", JointState, 0.5)
        self.assertEqual(len(message.position), 7)
        for position in message.position:
            self.assertAlmostEqual(position, 0, delta=0.1)

class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        test_relay = loader.loadTestsFromTestCase(TestIIWAStatus)

        self.addTests(test_relay)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, 'test_iiwa.SuiteTest', sys.argv)
