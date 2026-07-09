#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import time
import numpy as np
from i2cdev import I2C
from geometry_msgs.msg import Vector3Stamped

# ---------- ¼Ä´æÆ÷¶¨Òå ----------
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43


class MPU6050Driver:
    def __init__(self, bus, addr=MPU6050_ADDR):
        self.bus = bus
        self.addr = addr
        self.i2c = I2C(addr, bus)

    def write_byte(self, reg, val):
        self.i2c.write(bytes([reg, val]))

    def read_byte(self, reg):
        self.i2c.write(bytes([reg]))
        return self.i2c.read(1)[0]

    def read_word(self, reg):
        self.i2c.write(bytes([reg]))
        data = self.i2c.read(2)
        val = (data[0] << 8) | data[1]
        if val >= 0x8000:
            val -= 0x10000
        return val

    def init(self):
        try:
            self.write_byte(PWR_MGMT_1, 0)
            rospy.loginfo("MPU6050 initialized on bus %d", self.bus)
            return True
        except Exception as e:
            rospy.logerr("MPU6050 init failed: %s", e)
            return False

    def read_gyro(self):
        gx = self.read_word(GYRO_XOUT_H)
        gy = self.read_word(GYRO_XOUT_H + 2)
        gz = self.read_word(GYRO_XOUT_H + 4)
        return gx / 131.0, gy / 131.0, gz / 131.0

    def read_accel(self):
        ax = self.read_word(ACCEL_XOUT_H)
        ay = self.read_word(ACCEL_XOUT_H + 2)
        az = self.read_word(ACCEL_XOUT_H + 4)
        return ax / 16384.0, ay / 16384.0, az / 16384.0


class ImprovedKalman1D:
    def __init__(self, q=0.001, r=0.5, p0=1.0):
        self.q = q
        self.r = r
        self.p = p0
        self.k = 0.0
        self.x = 0.0

    def predict(self, u, dt):
        self.x += u * dt
        self.p += self.q

    def update(self, z):
        self.k = self.p / (self.p + self.r)
        self.x += self.k * (z - self.x)
        self.p = (1 - self.k) * self.p

    def get_angle(self):
        return self.x


class MPU6050AnglePublisher:
    def __init__(self):
        self.bus = rospy.get_param("~i2c_bus", 5)
        self.addr = rospy.get_param("~device_addr", 0x68)
        self.rate = rospy.get_param("~publish_rate", 100)
        self.alpha = rospy.get_param("~filter_alpha", 0.98)
        self.topic_name = rospy.get_param("~topic_name", "/mpu6050/angles")
        self.frame_id = rospy.get_param("~frame_id", "imu_link")

        self.kf_q = rospy.get_param("~kf_q", 0.001)
        self.kf_r = rospy.get_param("~kf_r", 0.5)

        self.gyro_threshold = rospy.get_param("~gyro_threshold", 1.5)
        self.accel_threshold = rospy.get_param("~accel_threshold", 0.3)
        self.stationary_required = rospy.get_param("~stationary_required", 5)

        self.driver = MPU6050Driver(self.bus, self.addr)
        if not self.driver.init():
            rospy.signal_shutdown("MPU6050 init failed")
            return

        self.gyro_bias_z = 0.0
        self.calibrate_gyro(samples=500, outlier_remove=True)

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw_kf = ImprovedKalman1D(q=self.kf_q, r=self.kf_r, p0=1.0)

        self.stationary_counter = 0

        self.pub = rospy.Publisher(self.topic_name, Vector3Stamped, queue_size=10)
        self.last_time = time.time()

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate), self.timer_callback)
        rospy.loginfo("MPU6050 publisher started (optimized yaw with Kalman + zero-velocity update)")

    def calibrate_gyro(self, samples=500, outlier_remove=True):
        rospy.loginfo("Calibrating gyro Z axis... Keep sensor absolutely still!")
        raw_data = []
        for _ in range(samples):
            _, _, gz = self.driver.read_gyro()
            raw_data.append(gz)
            rospy.sleep(0.005)

        if outlier_remove:
            median = np.median(raw_data)
            std = np.std(raw_data)
            filtered = [v for v in raw_data if abs(v - median) < 3 * std]
            bias = np.mean(filtered)
            rospy.loginfo("Removed %d outliers from %d samples", len(raw_data) - len(filtered), samples)
        else:
            bias = np.mean(raw_data)

        self.gyro_bias_z = bias
        rospy.loginfo("Gyro Z bias: %.4f deg/s (std: %.4f)", bias, np.std(raw_data))

    def is_stationary(self, gx, gy, gz, ax, ay, az):
        gyro_mag = math.sqrt(gx * gx + gy * gy + gz * gz)
        accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
        accel_dev = abs(accel_mag - 1.0)
        return gyro_mag < self.gyro_threshold and accel_dev < self.accel_threshold

    def timer_callback(self, event):
        now = time.time()
        dt = now - self.last_time
        if dt <= 0:
            return
        dt = min(dt, 0.02)

        gx, gy, gz = self.driver.read_gyro()
        ax, ay, az = self.driver.read_accel()

        # Roll / Pitch
        accel_roll = math.atan2(ay, az) * 180.0 / math.pi
        accel_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az)) * 180.0 / math.pi

        gyro_roll = self.roll + gx * dt
        gyro_pitch = self.pitch + gy * dt

        self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch

        # Yaw
        gz_corrected = gz - self.gyro_bias_z
        self.yaw_kf.predict(gz_corrected, dt)

        if self.is_stationary(gx, gy, gz, ax, ay, az):
            self.stationary_counter += 1
        else:
            self.stationary_counter = 0

        if self.stationary_counter >= self.stationary_required:
            self.yaw_kf.update(self.yaw_kf.get_angle())
            self.stationary_counter = self.stationary_required - 1

        yaw = self.yaw_kf.get_angle()
        if yaw > 180.0:
            yaw -= 360.0
        elif yaw < -180.0:
            yaw += 360.0

        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.vector.x = self.roll
        msg.vector.y = self.pitch
        msg.vector.z = yaw
        self.pub.publish(msg)

        self.last_time = now


def main():
    rospy.init_node('mpu6050_angle_publisher', anonymous=True)
    node = MPU6050AnglePublisher()
    rospy.spin()


if __name__ == '__main__':
    main()
