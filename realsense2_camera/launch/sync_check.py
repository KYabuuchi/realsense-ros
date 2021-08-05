#!/usr/bin/python2
from rospy import rostime
from sensor_msgs.msg import CompressedImage
import rospy
import message_filters


class SyncChecker:
    def __init__(self):
        rospy.init_node('sync_check')

        self.topic1 = '/left/infra1/image_rect_raw/compressed'
        self.topic2 = '/right/infra2/image_rect_raw/compressed'

        sync_sub1 = message_filters.Subscriber(self.topic1, CompressedImage)
        sync_sub2 = message_filters.Subscriber(self.topic2, CompressedImage)

        rospy.Subscriber(self.topic1, CompressedImage, self.callbackLeft)
        rospy.Subscriber(self.topic2, CompressedImage, self.callbackRight)

        ts = message_filters.ApproximateTimeSynchronizer([sync_sub1, sync_sub2], 1, slop=0.02)
        ts.registerCallback(self.callbackSync)

        self.sync_delay = 0
        self.sync_count = 0
        self.left_count = 0
        self.right_count = 0

    def callbackLeft(self, msg):
        self.left_count = self.left_count+1

    def callbackRight(self, msg):
        self.right_count = self.right_count+1

    def callbackSync(self, msg1, msg2):
        self.sync_count = self.sync_count+1
        d = (msg1.header.stamp-msg2.header.stamp).to_sec()
        self.sync_delay = self.sync_delay + abs(d)*1000

    def sleep(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            string = "Left:"+self.fpsToColor(self.left_count)\
                + " Right:"+self.fpsToColor(self.right_count)\
                + " Sync:"+self.fpsToColor(self.sync_count)
            mean_delay = 0
            if self.sync_count != 0:
                mean_delay = self.sync_delay / self.sync_count

            rospy.loginfo(string + " Delay:{:.2f}".format(mean_delay))
            self.sync_count = self.left_count = self.right_count = 0
            self.sync_delay = 0
            rate.sleep()

    def fpsToColor(self, fps):
        if fps > 10:
            return "\033[46m" + str(fps)+"\033[0m"
        if fps > 8:
            return "\033[44m" + str(fps)+"\033[0m"
        if fps > 3:
            return "\033[42m" + str(fps)+"\033[0m"
        if fps > 0:
            return "\033[43m" + str(fps)+"\033[0m"
        return "\033[41m" + str(fps)+"\033[0m"


def main():
    checker = SyncChecker()
    checker.sleep()


if __name__ == "__main__":
    main()
