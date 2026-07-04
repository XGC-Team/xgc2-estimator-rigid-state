#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval


class HighresImuRateRequester:
    def __init__(self):
        mavros_ns = rospy.get_param("~mavros_ns", "mavros")
        self.mavros_ns = rospy.resolve_name(mavros_ns).rstrip("/")
        self.message_id = int(rospy.get_param("~message_id", 105))
        self.message_rate = float(rospy.get_param("~message_rate", 250.0))
        self.retry_period_s = float(rospy.get_param("~retry_period_s", 2.0))
        self.reassert_period_s = float(rospy.get_param("~reassert_period_s", 10.0))

        self.connected = False
        self.requested_this_connection = False
        self.last_attempt = rospy.Time(0)
        self.last_success = rospy.Time(0)
        self.logged_success = False

        self.set_interval = rospy.ServiceProxy(
            self.mavros_ns + "/set_message_interval",
            MessageInterval,
        )
        rospy.Subscriber(
            self.mavros_ns + "/state",
            State,
            self._state_cb,
            queue_size=1,
        )

    def _state_cb(self, msg):
        if self.connected and not msg.connected:
            self.requested_this_connection = False
        self.connected = msg.connected

    def _attempt_request(self):
        try:
            rospy.wait_for_service(
                self.mavros_ns + "/set_message_interval",
                timeout=self.retry_period_s,
            )
            result = self.set_interval(
                message_id=self.message_id,
                message_rate=self.message_rate,
            )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(
                10.0,
                "HIGHRES_IMU rate request waiting for %s/set_message_interval: %s",
                self.mavros_ns,
                exc,
            )
            return

        if not result.success:
            rospy.logwarn_throttle(
                10.0,
                "HIGHRES_IMU rate request rejected: message_id=%s rate=%.1f",
                self.message_id,
                self.message_rate,
            )
            return

        self.requested_this_connection = True
        self.last_success = rospy.Time.now()
        if not self.logged_success:
            rospy.loginfo(
                "requested MAVLink message %s at %.1f Hz via %s/set_message_interval",
                self.message_id,
                self.message_rate,
                self.mavros_ns,
            )
            self.logged_success = True
        else:
            rospy.logdebug(
                "reasserted MAVLink message %s at %.1f Hz via %s/set_message_interval",
                self.message_id,
                self.message_rate,
                self.mavros_ns,
            )

    def run(self):
        rospy.loginfo(
            "waiting for MAVROS connection under %s before requesting message %s at %.1f Hz",
            self.mavros_ns,
            self.message_id,
            self.message_rate,
        )
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            elapsed = (now - self.last_attempt).to_sec()
            should_retry = elapsed >= self.retry_period_s
            should_reassert = (
                self.requested_this_connection
                and self.reassert_period_s > 0.0
                and (now - self.last_success).to_sec() >= self.reassert_period_s
            )

            if self.connected and (
                (not self.requested_this_connection and should_retry) or should_reassert
            ):
                self.last_attempt = now
                self._attempt_request()

            rate.sleep()


def main():
    rospy.init_node("request_highres_imu_rate")
    HighresImuRateRequester().run()


if __name__ == "__main__":
    main()
