#!/usr/bin/env python
import rospy

import sys
import unittest
import threading
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from skiros2_skill.ros.skill_layer_interface import SkillLayerInterface

PKG = 'integration_tests'
NAME = 'test_integration_tm'

## A sample python unit test
class TestSkiros2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('testing_node')
        cls.sli = SkillLayerInterface()

        rospy.wait_for_message("turtle1/pose", Pose, 10.0)
        rospy.wait_for_service('/tts_robot/command', 10.0)
        rospy.wait_for_service('/wm/get', 10.0)
        rospy.sleep(5.0)
        cls.event = threading.Event()

    def setUp(self):
        self.sli.set_monitor_cb(self._monitor_cb)

    def tearDown(self):
        self.sli.set_monitor_cb(None)
        self.event.clear()
        self.new_message = None

    def _check_turtle_pose_published(self, turtle="turtle1"):
        message = rospy.wait_for_message("%s/pose" % (turtle), Pose, 1.0)
        return True, message

    def _check_turtle_cmd_vel_published(self, turtle="turtle1"):
        message = rospy.wait_for_message("%s/cmd_vel" % (turtle), Twist, 1.0)
        return True, message

    def _monitor_cb(self, message):
        self.new_message = message
        self.event.set()

    def test_1(self):
        result, message = self._check_turtle_pose_published()
        self.assertTrue(result)
        self.assertAlmostEqual(message.x, 5.5, delta=0.1)
        self.assertAlmostEqual(message.y, 5.5, delta=0.1)
        self.assertAlmostEqual(message.theta, 0.0, delta=0.1)
        rospy.wait_for_service('/tts_robot/command', 1.0)
        rospy.wait_for_service('/wm/get', 1.0)

    def test_2(self):
        agent0 = self.sli.agents['/tts_robot']
        agent0skills = agent0.skills #Get all skills of the first skill manager
        execution_id = self.sli.execute(agent0.name, [agent0skills['wander_around']])#Execute a skill
        self._check_turtle_cmd_vel_published()
        if not(self.event.wait(1.0)):
            self.fail("No Monitored Callback")
        rospy.sleep(1.0)
        self.event.clear()
        self.new_message = None
        self.sli.preempt_one(tid=execution_id)
        if not(self.event.wait(1.0)):
            self.fail("No Monitored Callback after preemption")
        self.assertEqual(self.new_message.progress_message, "Preempted")
        result, message = self._check_turtle_pose_published()
        self.assertAlmostEqual(message.x, 6.2, delta=0.1)
        self.assertAlmostEqual(message.y, 5.5, delta=0.1)



class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()
        loader = unittest.TestLoader()
        loader.sortTestMethodsUsing = None
        testSkiros2 = loader.loadTestsFromTestCase(TestSkiros2)
        self.addTests(testSkiros2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, 'test_skiros2.SuiteTest', sys.argv)
