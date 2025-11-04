#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time
"""
class ReactiveFollowGap(object):

    Follow-the-Gap with wall-biasing, TTC braking, smoothing, and simple recovery.
    ROS1 / rospy


    def __init__(self):
        rospy.init_node('reactive_follow_gap', anonymous=False)

        # Topics
        self.lidarscan_topic = rospy.get_param('~scan_topic', '/scan')
        self.drive_topic     = rospy.get_param('~drive_topic', '/drive')

        # Gap-follow params
        self.radians_per_elem       = None
        self.BUBBLE_RADIUS          = rospy.get_param('~bubble_radius', 140)
        self.PREPROCESS_CONV_SIZE   = rospy.get_param('~preprocess_conv_size', 3)
        self.BEST_POINT_CONV_SIZE   = rospy.get_param('~best_point_conv_size', 140)
        self.MAX_LIDAR_DIST         = rospy.get_param('~max_lidar_dist', 3.0)
        self.STRAIGHTS_SPEED        = rospy.get_param('~straights_speed', 4.5)
        self.CORNERS_SPEED          = rospy.get_param('~corners_speed', 1.5)
        self.STRAIGHTS_STEER_THRESH = rospy.get_param('~straights_steer_angle', np.pi/18.0)  # 10°
        self.MAX_STEERING_ANGLE     = rospy.get_param('~max_steering_angle', 40.0*np.pi/180.0)

        # New: safety + behavior
        self.MIN_FRONT_DIST_STOP    = rospy.get_param('~min_front_dist_stop', 0.35)  # hard stop
        self.MIN_FRONT_DIST_SLOW    = rospy.get_param('~min_front_dist_slow', 0.70)  # slow down
        self.TTC_THRESHOLD          = rospy.get_param('~ttc_threshold', 0.6)        # sec
        self.TTC_SOFT_BRAKE         = rospy.get_param('~ttc_soft_brake', 0.9)       # speed factor
        self.SPEED_MIN              = rospy.get_param('~speed_min', 0.6)
        self.SPEED_MAX              = rospy.get_param('~speed_max', 4.5)

        # New: wall repulsion / center bias (0..1 higher = stronger center preference)
        self.CENTER_BIAS_ALPHA      = rospy.get_param('~center_bias_alpha', 0.35)
        self.SIDE_REPULSION_GAIN    = rospy.get_param('~side_repulsion_gain', 0.25)

        # New: steering smoothing (EMA) + deadband
        self.STEER_EMA              = rospy.get_param('~steer_ema', 0.3)   # 0..1, higher=more smoothing
        self.STEER_DEADBAND         = rospy.get_param('~steer_deadband', 0.02)

        # New: stuck recovery (very simple heuristic)
        self.RECOVERY_TURN_TIME     = rospy.get_param('~recovery_turn_time', 0.8)  # sec
        self.RECOVERY_SPEED         = rospy.get_param('~recovery_speed', -0.8)     # reverse
        self.RECOVERY_STEER         = rospy.get_param('~recovery_steer', 0.6)      # rad
        self.SAT_STEER_THRESH       = rospy.get_param('~sat_steer_thresh', 0.6)    # near max
        self.STUCK_WINDOW           = rospy.get_param('~stuck_window', 0.9)        # sec
        self.FRONT_VAR_THRESH       = rospy.get_param('~front_var_thresh', 1e-3)   # low variance means not changing

        # State
        self.prev_cmd_steer = 0.0
        self.front_history = []   # rolling (time, front_min)
        self.in_recovery   = False
        self.recovery_until= 0.0
        self.recovery_sign = 1

        # Pub/Sub
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.scan_sub  = rospy.Subscriber(self.lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)

        rospy.loginfo("ReactiveFollowGap+ ready | scan:=%s drive:=%s", self.lidarscan_topic, self.drive_topic)

    # ---------------- helpers ----------------

    def preprocess_lidar(self, ranges):
        n = len(ranges)
        if n == 0:
            return np.array([], dtype=float)
        self.radians_per_elem = (2*np.pi) / n

        # Crop to front sector (like your original)
        proc = np.array(ranges[135:-135], dtype=float)

        # Smooth
        if self.PREPROCESS_CONV_SIZE > 1:
            k = self.PREPROCESS_CONV_SIZE
            proc = np.convolve(proc, np.ones(k), 'same') / float(k)

        proc = np.clip(proc, 0.0, float(self.MAX_LIDAR_DIST))
        return proc

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0.0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        if not slices:
            return 0, 0
        chosen = max(slices, key=lambda sl: (sl.stop - sl.start))
        return chosen.start, chosen.stop

    def find_best_point(self, start_i, end_i, ranges):
        if end_i <= start_i + 1:
            return start_i
        segment = ranges[start_i:end_i]
        if self.BEST_POINT_CONV_SIZE > 1:
            k = self.BEST_POINT_CONV_SIZE
            avg = np.convolve(segment, np.ones(k), 'same') / float(k)
        else:
            avg = segment
        return int(np.argmax(avg)) + start_i

    def idx_to_steer_angle(self, idx, length):
        lidar_angle = (idx - (length / 2.0)) * self.radians_per_elem
        return lidar_angle / 2.0

    def center_bias_weights(self, length):
        # 0 at edges, 1 at center → raise to alpha to tune how strongly we prefer center
        x = np.linspace(-1, 1, length)
        w = 1.0 - np.abs(x)        # triangle: 0 at edges, 1 at center
        w = (1 - self.CENTER_BIAS_ALPHA) + self.CENTER_BIAS_ALPHA * w
        return w

    def side_repulsion(self, arr):
        # Penalize near ±90° sectors if they are very close (reduces wall hugging)
        L = len(arr)
        third = max(1, L // 3)
        left  = arr[:third]
        right = arr[-third:]
        penalty_left  = np.clip((1.0 - left / max(1e-6, np.max(left))), 0, 1)
        penalty_right = np.clip((1.0 - right/ max(1e-6, np.max(right))),0, 1)
        out = arr.copy()
        out[:third]  *= (1.0 - self.SIDE_REPULSION_GAIN * penalty_left)
        out[-third:] *= (1.0 - self.SIDE_REPULSION_GAIN * penalty_right)
        return out

    def ttc(self, ranges, angle_idx, speed):
        # crude TTC: distance / closing_speed_along_beam
        if speed <= 0.0:
            return np.inf
        theta = (angle_idx - (len(ranges)/2.0)) * self.radians_per_elem
        closing = max(1e-3, speed * np.cos(theta))
        return ranges[angle_idx] / closing

    def push_front_history(self, front_min):
        now = time.time()
        self.front_history.append((now, float(front_min)))
        # drop old
        window_start = now - self.STUCK_WINDOW
        while len(self.front_history) and self.front_history[0][0] < window_start:
            self.front_history.pop(0)

    def is_stuck(self, steer_cmd):
        if abs(steer_cmd) < self.SAT_STEER_THRESH:
            return False
        if len(self.front_history) < 3:
            return False
        vals = np.array([v for _, v in self.front_history])
        return np.var(vals) < self.FRONT_VAR_THRESH

    # ---------------- callback ----------------

    def lidar_callback(self, msg):
        ranges = msg.ranges
        if not ranges:
            return

        proc = self.preprocess_lidar(ranges)
        if proc.size == 0:
            return

        # --- safety checks on front sector ---
        mid = proc.size // 2
        front_slice = proc[mid - 20: mid + 20] if proc.size > 40 else proc
        front_min = float(np.min(front_slice)) if front_slice.size else float(np.min(proc))
        self.push_front_history(front_min)

        # ---- bubble around closest obstacle (on processed array) ----
        closest_idx = int(np.argmin(proc))
        lo = max(0, closest_idx - self.BUBBLE_RADIUS)
        hi = min(proc.size - 1, closest_idx + self.BUBBLE_RADIUS)
        proc[lo:hi+1] = 0.0

        # ---- wall repulsion + center bias before gap finding ----
        biased = self.side_repulsion(proc)
        biased *= self.center_bias_weights(biased.size)

        # ---- find gap & best point on biased field ----
        g0, g1 = self.find_max_gap(biased)
        best_idx = self.find_best_point(g0, g1, biased)

        # ---- steering selection with smoothing/hysteresis ----
        steer_raw = self.idx_to_steer_angle(best_idx, biased.size)
        steer_raw = max(-self.MAX_STEERING_ANGLE, min(self.MAX_STEERING_ANGLE, steer_raw))

        # EMA smoothing + deadband
        steer = (1.0 - self.STEER_EMA) * steer_raw + self.STEER_EMA * self.prev_cmd_steer
        if abs(steer) < self.STEER_DEADBAND:
            steer = 0.0
        self.prev_cmd_steer = steer

        # ---- speed policy: geometry + safety (front distance + TTC) ----
        # base by curvature
        base_speed = self.CORNERS_SPEED if abs(steer) > self.STRAIGHTS_STEER_THRESH else self.STRAIGHTS_SPEED
        # scale by free distance ahead
        dist_scale = np.clip(front_min / self.MAX_LIDAR_DIST, 0.2, 1.0)
        speed = np.clip(base_speed * dist_scale, self.SPEED_MIN, self.SPEED_MAX)

        # Hard/soft brake by distance
        if front_min < self.MIN_FRONT_DIST_STOP:
            speed = 0.0
        elif front_min < self.MIN_FRONT_DIST_SLOW:
            speed = min(speed, self.CORNERS_SPEED)

        # TTC soft brake (use best ray TTC as proxy)
        ttc_val = self.ttc(proc, best_idx, max(speed, 1e-3))
        if ttc_val < self.TTC_THRESHOLD:
            speed *= self.TTC_SOFT_BRAKE

        # ---- simple recovery if stuck while steering saturated ----
        now = time.time()
        if self.in_recovery:
            if now < self.recovery_until:
                # keep reversing with fixed steer (flip each time)
                steer_rec = self.RECOVERY_STEER * self.recovery_sign
                self.publish_drive(self.RECOVERY_SPEED, steer_rec)
                return
            else:
                self.in_recovery = False

        if self.is_stuck(steer):
            self.in_recovery = True
            self.recovery_until = now + self.RECOVERY_TURN_TIME
            self.recovery_sign *= -1  # alternate direction each attempt
            self.publish_drive(self.RECOVERY_SPEED, self.RECOVERY_STEER * self.recovery_sign)
            return

        # ---- normal command ----
        self.publish_drive(speed, steer)

    def publish_drive(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = float(steering_angle) * 1.2
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

def main():
    ReactiveFollowGap()
    rospy.spin()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import time
"""
class ReactiveFollowGap(object):
    """
    Follow-the-Gap with wall-biasing, TTC braking, smoothing, and simple recovery.
    ROS1 / rospy
    """

    def __init__(self):
        rospy.init_node('reactive_follow_gap', anonymous=False)

        # Topics
        self.lidarscan_topic = rospy.get_param('~scan_topic', '/scan')
        self.drive_topic     = rospy.get_param('~drive_topic', '/drive')

        # Gap-follow params
        self.radians_per_elem       = None
        self.BUBBLE_RADIUS          = rospy.get_param('~bubble_radius', 140)
        self.PREPROCESS_CONV_SIZE   = rospy.get_param('~preprocess_conv_size', 3)
        self.BEST_POINT_CONV_SIZE   = rospy.get_param('~best_point_conv_size', 140)
        self.MAX_LIDAR_DIST         = rospy.get_param('~max_lidar_dist', 3.0)
        self.STRAIGHTS_SPEED        = rospy.get_param('~straights_speed', 4.5)
        self.CORNERS_SPEED          = rospy.get_param('~corners_speed', 1.5)
        self.STRAIGHTS_STEER_THRESH = rospy.get_param('~straights_steer_angle', np.pi/18.0)  # 10°
        self.MAX_STEERING_ANGLE     = rospy.get_param('~max_steering_angle', 40.0*np.pi/180.0)

        # New: safety + behavior
        self.MIN_FRONT_DIST_STOP    = rospy.get_param('~min_front_dist_stop', 0.35)  # hard stop
        self.MIN_FRONT_DIST_SLOW    = rospy.get_param('~min_front_dist_slow', 0.70)  # slow down
        self.TTC_THRESHOLD          = rospy.get_param('~ttc_threshold', 0.6)        # sec
        self.TTC_SOFT_BRAKE         = rospy.get_param('~ttc_soft_brake', 0.9)       # speed factor
        self.SPEED_MIN              = rospy.get_param('~speed_min', 0.6)
        self.SPEED_MAX              = rospy.get_param('~speed_max', 4.5)

        # New: wall repulsion / center bias (0..1 higher = stronger center preference)
        self.CENTER_BIAS_ALPHA      = rospy.get_param('~center_bias_alpha', 0.35)
        self.SIDE_REPULSION_GAIN    = rospy.get_param('~side_repulsion_gain', 0.25)

        # New: steering smoothing (EMA) + deadband
        self.STEER_EMA              = rospy.get_param('~steer_ema', 0.3)   # 0..1, higher=more smoothing
        self.STEER_DEADBAND         = rospy.get_param('~steer_deadband', 0.02)

        # New: stuck recovery (very simple heuristic)
        self.RECOVERY_TURN_TIME     = rospy.get_param('~recovery_turn_time', 0.8)  # sec
        self.RECOVERY_SPEED         = rospy.get_param('~recovery_speed', -0.8)     # reverse
        self.RECOVERY_STEER         = rospy.get_param('~recovery_steer', 0.6)      # rad
        self.SAT_STEER_THRESH       = rospy.get_param('~sat_steer_thresh', 0.6)    # near max
        self.STUCK_WINDOW           = rospy.get_param('~stuck_window', 0.9)        # sec
        self.FRONT_VAR_THRESH       = rospy.get_param('~front_var_thresh', 1e-3)   # low variance means not changing

        # State
        self.prev_cmd_steer = 0.0
        self.front_history = []   # rolling (time, front_min)
        self.in_recovery   = False
        self.recovery_until= 0.0
        self.recovery_sign = 1

        # Pub/Sub
        self.drive_pub = rospy.Publisher(self.drive_topic, AckermannDriveStamped, queue_size=1)
        self.scan_sub  = rospy.Subscriber(self.lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)

        rospy.loginfo("ReactiveFollowGap+ ready | scan:=%s drive:=%s", self.lidarscan_topic, self.drive_topic)

    # ---------------- helpers ----------------

    def preprocess_lidar(self, ranges):
        n = len(ranges)
        if n == 0:
            return np.array([], dtype=float)
        self.radians_per_elem = (2*np.pi) / n

        # Crop to front sector (like your original)
        proc = np.array(ranges[135:-135], dtype=float)

        # Smooth
        if self.PREPROCESS_CONV_SIZE > 1:
            k = self.PREPROCESS_CONV_SIZE
            proc = np.convolve(proc, np.ones(k), 'same') / float(k)

        proc = np.clip(proc, 0.0, float(self.MAX_LIDAR_DIST))
        return proc

    def find_max_gap(self, free_space_ranges):
        masked = np.ma.masked_where(free_space_ranges == 0.0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        if not slices:
            return 0, 0
        chosen = max(slices, key=lambda sl: (sl.stop - sl.start))
        return chosen.start, chosen.stop

    def find_best_point(self, start_i, end_i, ranges):
        if end_i <= start_i + 1:
            return start_i
        segment = ranges[start_i:end_i]
        if self.BEST_POINT_CONV_SIZE > 1:
            k = self.BEST_POINT_CONV_SIZE
            avg = np.convolve(segment, np.ones(k), 'same') / float(k)
        else:
            avg = segment
        return int(np.argmax(avg)) + start_i

    def idx_to_steer_angle(self, idx, length):
        lidar_angle = (idx - (length / 2.0)) * self.radians_per_elem
        return lidar_angle / 2.0

    def center_bias_weights(self, length):
        # 0 at edges, 1 at center → raise to alpha to tune how strongly we prefer center
        x = np.linspace(-1, 1, length)
        w = 1.0 - np.abs(x)        # triangle: 0 at edges, 1 at center
        w = (1 - self.CENTER_BIAS_ALPHA) + self.CENTER_BIAS_ALPHA * w
        return w

    def side_repulsion(self, arr):
        # Penalize near ±90° sectors if they are very close (reduces wall hugging)
        L = len(arr)
        third = max(1, L // 3)
        left  = arr[:third]
        right = arr[-third:]
        penalty_left  = np.clip((1.0 - left / max(1e-6, np.max(left))), 0, 1)
        penalty_right = np.clip((1.0 - right/ max(1e-6, np.max(right))),0, 1)
        out = arr.copy()
        out[:third]  *= (1.0 - self.SIDE_REPULSION_GAIN * penalty_left)
        out[-third:] *= (1.0 - self.SIDE_REPULSION_GAIN * penalty_right)
        return out

    def ttc(self, ranges, angle_idx, speed):
        # crude TTC: distance / closing_speed_along_beam
        if speed <= 0.0:
            return np.inf
        theta = (angle_idx - (len(ranges)/2.0)) * self.radians_per_elem
        closing = max(1e-3, speed * np.cos(theta))
        return ranges[angle_idx] / closing

    def push_front_history(self, front_min):
        now = time.time()
        self.front_history.append((now, float(front_min)))
        # drop old
        window_start = now - self.STUCK_WINDOW
        while len(self.front_history) and self.front_history[0][0] < window_start:
            self.front_history.pop(0)

    def is_stuck(self, steer_cmd):
        if abs(steer_cmd) < self.SAT_STEER_THRESH:
            return False
        if len(self.front_history) < 3:
            return False
        vals = np.array([v for _, v in self.front_history])
        return np.var(vals) < self.FRONT_VAR_THRESH

    # ---------------- callback ----------------

    def lidar_callback(self, msg):
        ranges = msg.ranges
        if not ranges:
            return

        proc = self.preprocess_lidar(ranges)
        if proc.size == 0:
            return

        # --- safety checks on front sector ---
        mid = proc.size // 2
        front_slice = proc[mid - 20: mid + 20] if proc.size > 40 else proc
        front_min = float(np.min(front_slice)) if front_slice.size else float(np.min(proc))
        self.push_front_history(front_min)

        # ---- bubble around closest obstacle (on processed array) ----
        closest_idx = int(np.argmin(proc))
        lo = max(0, closest_idx - self.BUBBLE_RADIUS)
        hi = min(proc.size - 1, closest_idx + self.BUBBLE_RADIUS)
        proc[lo:hi+1] = 0.0

        # ---- wall repulsion + center bias before gap finding ----
        biased = self.side_repulsion(proc)
        biased *= self.center_bias_weights(biased.size)

        # ---- find gap & best point on biased field ----
        g0, g1 = self.find_max_gap(biased)
        best_idx = self.find_best_point(g0, g1, biased)

        # ---- steering selection with smoothing/hysteresis ----
        steer_raw = self.idx_to_steer_angle(best_idx, biased.size)
        steer_raw = max(-self.MAX_STEERING_ANGLE, min(self.MAX_STEERING_ANGLE, steer_raw))

        # EMA smoothing + deadband
        steer = (1.0 - self.STEER_EMA) * steer_raw + self.STEER_EMA * self.prev_cmd_steer
        if abs(steer) < self.STEER_DEADBAND:
            steer = 0.0
        self.prev_cmd_steer = steer

        # ---- speed policy: geometry + safety (front distance + TTC) ----
        # base by curvature
        base_speed = self.CORNERS_SPEED if abs(steer) > self.STRAIGHTS_STEER_THRESH else self.STRAIGHTS_SPEED
        # scale by free distance ahead
        dist_scale = np.clip(front_min / self.MAX_LIDAR_DIST, 0.2, 1.0)
        speed = np.clip(base_speed * dist_scale, self.SPEED_MIN, self.SPEED_MAX)

        # Hard/soft brake by distance
        if front_min < self.MIN_FRONT_DIST_STOP:
            speed = 0.0
        elif front_min < self.MIN_FRONT_DIST_SLOW:
            speed = min(speed, self.CORNERS_SPEED)

        # TTC soft brake (use best ray TTC as proxy)
        ttc_val = self.ttc(proc, best_idx, max(speed, 1e-3))
        if ttc_val < self.TTC_THRESHOLD:
            speed *= self.TTC_SOFT_BRAKE

        # ---- simple recovery if stuck while steering saturated ----
        now = time.time()
        if self.in_recovery:
            if now < self.recovery_until:
                # keep reversing with fixed steer (flip each time)
                steer_rec = self.RECOVERY_STEER * self.recovery_sign
                self.publish_drive(self.RECOVERY_SPEED, steer_rec)
                return
            else:
                self.in_recovery = False

        if self.is_stuck(steer):
            self.in_recovery = True
            self.recovery_until = now + self.RECOVERY_TURN_TIME
            self.recovery_sign *= -1  # alternate direction each attempt
            self.publish_drive(self.RECOVERY_SPEED, self.RECOVERY_STEER * self.recovery_sign)
            return

        # ---- normal command ----
        self.publish_drive(speed, steer)

    def publish_drive(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = float(steering_angle) * 1.2
        msg.drive.speed = float(speed)
        self.drive_pub.publish(msg)

def main():
    ReactiveFollowGap()
    rospy.spin()

if __name__ == '__main__':
    main()
