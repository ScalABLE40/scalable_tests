#!/usr/bin/env python
import rospy

import sys
import unittest
from osps_msgs.msg import TMHeartBeep

PKG = 'integration_tests'
NAME = 'test_integration_tm'

## A sample python unit test
class TestTaskManager(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('testing_node')
        rospy.wait_for_message("/OSPS/TM/HeartBeep", TMHeartBeep, 30.0)

    def _check_robot_heart_beep(self, name="igor"):
        message = rospy.wait_for_message("/OSPS/TM/HeartBeep", TMHeartBeep, 20.0)
        self.assertEqual(message.publisherId, name)

    def test_tm_init(self):
        self._check_robot_heart_beep()


class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testTM = loader.loadTestsFromTestCase(TestTaskManager)

        self.addTests(testTM)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, 'test_tm.SuiteTest', sys.argv)
