#!/usr/bin/env python
import rospy

import sys
import unittest
import threading
from sensor_msgs.msg import JointState
from skiros2_skill.ros.skill_layer_interface import SkillLayerInterface

PKG = 'integration_tests'
NAME = 'test_integration_tm'

## A sample python unit test
class TestIIWASkiros2(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('testing_node')
        cls.sli = SkillLayerInterface()
        rospy.wait_for_service('/wm/get', 10.0)
        rospy.sleep(5.0)
        cls.event = threading.Event()
        cls.succeeded = threading.Event()

    def _reset(self):
        self.event.clear()
        self.succeeded.clear()
        self.new_message = None

    def setUp(self):
        rospy.wait_for_message("/iiwa/joint_states", JointState, 10)
        self.sli.set_monitor_cb(self._monitor_cb)

    def tearDown(self):
        self.sli.set_monitor_cb(None)
        self._reset()

    def _monitor_cb(self, message):
        self.new_message = message
        self.event.set()
        if(message.state == 1 and message.progress_message == "Succeeded"):
            self.succeeded.set()

    def _check_joint_status(self, target=[], delta=0.03):
        message = rospy.wait_for_message("/iiwa/joint_states", JointState, 0.5)
        self.assertEqual(len(message.position), 7)
        for id, position in enumerate(message.position):
            self.assertAlmostEqual(position, target[id], delta=delta)

    def test_1(self):
        message = rospy.wait_for_message("/iiwa/joint_states", JointState, 0.5)
        self.assertEqual(len(message.position), 7)
        for position in message.position:
            self.assertAlmostEqual(position, 0, delta=0.1)
        agent0 = self.sli.agents['/test_robot']
        agent0skills = agent0.skills #Get all skills of the first skill manager
        movearticular = agent0skills['articular_move']
        movearticular.ph["TargetConfiguration"].setValue("[0.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]")
        self.sli.execute(agent0.name, [agent0skills['articular_move']])#Execute a skill
        if not(self.succeeded.wait(5.0)):
            self.fail("Error in execution of skill move articular")
        self._reset()
        self._check_joint_status(target=[0.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], delta=0.03)

    def test_2(self):
        agent0 = self.sli.agents['/test_robot']
        agent0skills = agent0.skills #Get all skills of the first skill manager
        movearticular = agent0skills['articular_move']
        movearticular.ph["TargetConfiguration"].setValue("[0.57, -1.57, 1.57, 0.0, 0.0, 0.0, 0.0]")
        self.sli.execute(agent0.name, [agent0skills['articular_move']])#Execute a skill
        if not(self.succeeded.wait(5.0)):
            self.fail("Error in execution of skill move articular")
        self._reset()
        self._check_joint_status(target=[0.57, -1.57, 1.57, 0.0, 0.0, 0.0, 0.0], delta=0.03)



class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()

        loader = unittest.TestLoader()
        loader.sortTestMethodsUsing = None
        test_iiwa_skiros2 = loader.loadTestsFromTestCase(TestIIWASkiros2)
        self.addTests(test_iiwa_skiros2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, 'test_iiwa_skiros2.SuiteTest', sys.argv)
