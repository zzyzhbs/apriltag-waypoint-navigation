"""ROS glue: /detections + TF -> TagObs (shared by hw2_probe and hw2_nav).

Detections are only queued in the callback; the TF lookup happens later in
collect(), so the tag TF (published right after /detections) has arrived.
The lookup uses the image stamp, never "latest": TF keeps a tag's last pose
long after the tag has left the view.
"""
import threading
from collections import deque

from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_control.tag_obs import obs_from_transform

KEEP_S = 10.0


class TagListener:
    def __init__(self, node, z_sign, base_frame='base_link', frame_fmt='tag_{id}',
                 topic='/detections'):
        self.node = node
        self.z_sign = z_sign
        self.base_frame = base_frame
        self.frame_fmt = frame_fmt
        self.buffer = Buffer(cache_time=Duration(seconds=KEEP_S))
        self.tf_listener = TransformListener(self.buffer, node)
        self.pending = deque()      # (tag_id, image stamp, receive time)
        self.lock = threading.Lock()
        self.lookup_failures = 0
        node.create_subscription(AprilTagDetectionArray, topic, self.on_detections, 10)

    def now(self):
        return self.node.get_clock().now()

    def on_detections(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        received = self.now()
        with self.lock:
            for d in msg.detections:
                self.pending.append((d.id, stamp, received))
            self.prune(received)

    def prune(self, now):
        oldest = now - Duration(seconds=KEEP_S)
        while self.pending and self.pending[0][1] < oldest:
            self.pending.popleft()

    def lookup(self, tag_id, stamp):
        """base_link -> tag transform at the image stamp, or None."""
        frame = self.frame_fmt.format(id=tag_id)
        try:
            tf = self.buffer.lookup_transform(self.base_frame, frame, stamp)
        except TransformException:
            self.lookup_failures += 1
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)

    def collect(self, since):
        """Observations with image stamp >= since -> (list of TagObs, list of latencies s)."""
        with self.lock:
            items = [p for p in self.pending if p[1] >= since]
        obs, latencies = [], []
        for tag_id, stamp, received in items:
            tf = self.lookup(tag_id, stamp)
            if tf is None:
                continue
            obs.append(obs_from_transform(tag_id, stamp.nanoseconds * 1e-9, *tf, self.z_sign))
            latencies.append((received - stamp).nanoseconds * 1e-9)
        return obs, latencies
