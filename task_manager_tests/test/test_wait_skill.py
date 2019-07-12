#!/usr/bin/env python
import rospy

import sys
import unittest

PKG = 'task_manager_tests'
NAME = 'test_wait_skill'

from osps_msgs.msg import TMHeartBeep, PMExecuteTaskReq, TMTaskStatus

## A sample python unit test
class TestTaskManager(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('testing_node')

    def _check_robot_heart_beep(self, name_robot="igor"):
        message = rospy.wait_for_message("/OSPS/TM/HeartBeep", TMHeartBeep, 20.0)
        self.assertEqual(message.publisherId, name_robot)

    def _check_start_execution(self):
        rospy.wait_for_message("/OSPS/PM/ExecuteTaskReq", PMExecuteTaskReq, 20.0)

    def _check_end_of_task(self):
        rospy.wait_for_message("/OSPS/TM/TaskStatus", TMTaskStatus, 30.0)


    def test_skill(self):
        self._check_robot_heart_beep()
        self._check_start_execution()
        self._check_end_of_task()


class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()

        testTM = loader.loadTestsFromTestCase(TestTaskManager)

        self.addTests(testTM)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, 'test_wait_skill.SuiteTest', sys.argv)
