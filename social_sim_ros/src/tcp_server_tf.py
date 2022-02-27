import ros_tcp_endpoint
import tf
from geometry_msgs.msg import PoseStamped


class RosTFBroadcaster(ros_tcp_endpoint.communication.RosSender):
    """
    Class to publish to the TF tree
    """

    def __init__(self, child, parent):
        ros_tcp_endpoint.communication.RosSender.__init__(self)
        self.msg = PoseStamped()
        self.child = child
        self.parent = parent
        self.last_ts = 0

    def send(self, data):
        br = tf.TransformBroadcaster()
        self.msg.deserialize(data)
        ts = self.msg.header.stamp.to_sec()
        #print(f"got transform: {self.parent} -> {self.child}")
        if ts <= self.last_ts:
            return
        self.last_ts = ts
        p = self.msg.pose.position
        o = self.msg.pose.orientation
        br.sendTransform((p.x, p.y, p.z),
                        (o.x, o.y, o.z, o.w),
                        self.msg.header.stamp,
                        self.child,
                        self.parent)

        return None