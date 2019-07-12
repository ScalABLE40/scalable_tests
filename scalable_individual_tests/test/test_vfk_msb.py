#!/usr/bin/env python
import rospy

import sys
import unittest

from std_msgs.msg import Empty, String


PKG = 'scalable_individual_tests'
NAME = 'test_vfk_msb_client'

class TestVFKMSBClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('vfk_msb_testing_node')
        cls.start = rospy.Publisher("/CI/Scalable/Start", Empty, queue_size=1)
        cls.next = rospy.Publisher("/CI/Scalable/Trigger_next", Empty, queue_size=1)
        cls.end = rospy.Publisher("/CI/Scalable/End", Empty, queue_size=1)
        rospy.sleep(5.0)


    def test_trigger(self):
        self.start.publish()
        rospy.wait_for_message("/CI/Scalable/AnswerTrigger", String, 5.0)
        self.next.publish()
        msg = rospy.wait_for_message("/CI/Scalable/StringPub", String, 5.0)
        self.assertEqual(msg.data, "This is a CI message. Please ignore")
        self.end.publish()
        rospy.sleep(1.0)




class SuiteTest(unittest.TestSuite):

    def __init__(self):
        super(SuiteTest, self).__init__()
        loader = unittest.TestLoader()
        loader.sortTestMethodsUsing = None
        Test_msb_client = loader.loadTestsFromTestCase(TestVFKMSBClient)
        self.addTests(Test_msb_client)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, NAME, 'test_vfk_msb.SuiteTest', sys.argv)