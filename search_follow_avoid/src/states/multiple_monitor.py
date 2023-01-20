import rospy
import smach
import threading

class MultipleMonitorState(smach.State):
    def __init__(self, topic, msg_type, timeout = 0, outcomes = []):
        if(timeout != 0):
            super().__init__(outcomes=(outcomes + ["timeout"]))
        else:
            super().__init__(outcomes=(outcomes))
        self._topic = topic
        self._msg_type = msg_type
        self._timeout = rospy.Duration(timeout)
        self._outcome = None
        self._poll_rate = rospy.Duration(0.05) 
        self._trigger_event = threading.Event()

    def execute(self, ud):
        self._outcome = None
        start_time = rospy.Time.now()

        self._trigger_event.clear()
        self._sub = rospy.Subscriber(self._topic, self._msg_type, self._cb)

        if(not self._timeout.is_zero()):
            while self._outcome is None:
                if rospy.Time.now() - start_time > self._timeout: 
                    self._outcome = 'timeout'
                    break
                rospy.sleep(self._poll_rate)
        else:
            self._trigger_event.wait()

        self._sub.unregister()
        return self._outcome

    def _cb(self,msg):
        try:
            self._outcome = self.on_receive(msg)
            if(self._outcome is not None):
                self._trigger_event.set()
        except Exception as e:
            rospy.logerr("Error thrown while executing on_receive: %s" % (e))
            self._trigger_event.set()

    def on_receive(self, msg):
        pass