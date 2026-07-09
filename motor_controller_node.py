#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
import Hobot.GPIO as GPIO
from motor_control.msg import MotorCmd
from geometry_msgs.msg import Vector3Stamped, Vector3   # Ê¹ÓÃ Vector3 Ìæ´ú Vector2
from std_msgs.msg import Bool

# ---------- Òý½ÅÅäÖÃ£¨BOARD ±àÂë£© ----------
YAW_PWM_PIN = 33
YAW_DIR_PIN = 31
YAW_EN_PIN = 29
PITCH_PWM_PIN = 32
PITCH_DIR_PIN = 36
PITCH_EN_PIN = 38

# ---------- µç»ú×´Ì¬Àà ----------
class MotorState:
    def __init__(self):
        self.mode = 0
        self.current_pos = 0
        self.target_pos = 0
        self.target_speed = 0
        self.current_speed = 0
        self.accel = 10000.0
        self.decel = 10000.0
        self.max_speed = 9000
        self.min_speed = 0
        self.steps_to_go = 0
        self.state = 0
        self.accel_count = 0
        self.step_delay = 0.0

yaw = MotorState()
pitch = MotorState()
pwm_yaw = None
pwm_pitch = None

# ---------- PID ²ÎÊý£¨´Ó launch ¶ÁÈ¡£©----------
kp_yaw = 20.0
kd_yaw = 200.0
kp_pitch = 20.0
kd_pitch = 140.0
err_ref = 20.0
spd_ref = 15.0
err_gain = 2.0
spd_gain = 1.5
lp_alpha = 0.3
dt = 0.02
max_freq = 9000
min_freq = 0
accel_rate = 10000.0

# ---------- É¨ÃèÓëÏÞÎ»²ÎÊý ----------
scan_speed = 300
roll_min = -45.0
roll_max = 45.0
pitch_min = -30.0
pitch_max = 30.0
scan_direction = 1

# ---------- ¸Ä½ø²ÎÊý ----------
angle_lp_alpha = 0.5          # ½Ç¶ÈµÍÍ¨ÂË²¨ÏµÊý
speed_smooth_alpha = 0.3      # ËÙ¶ÈÆ½»¬ÏµÊý
max_speed_change = 500.0      # Ã¿ÖÜÆÚ×î´óËÙ¶È±ä»¯£¨ÆµÂÊµ¥Î»£©
hysteresis = 1.0              # ÏÞÎ»ÖÍ»Ø£¨¶È£©
use_angle_compensation = False
angle_comp_gain = 0.0

# ---------- È«¾Ö±äÁ¿ ----------
pixel_speed_x = 0.0
pixel_speed_y = 0.0
last_error_x = 0.0
last_error_y = 0.0
error_x_global = 0.0
error_y_global = 0.0
current_roll = 0.0
current_pitch = 0.0
filtered_roll = 0.0
filtered_pitch = 0.0
smooth_speed_x = 0.0
smooth_speed_y = 0.0

# ---------- ÉãÏñÍ·×´Ì¬ ----------
target_visible = False
target_error_x = 0.0
target_error_y = 0.0

# ---------- »Øµ÷º¯Êý ----------
def target_visible_cb(msg):
    global target_visible
    target_visible = msg.data

def target_error_cb(msg):
    global target_error_x, target_error_y
    target_error_x = msg.x
    target_error_y = msg.y

def angle_callback(msg):
    global current_roll, current_pitch, filtered_roll, filtered_pitch
    current_roll = msg.vector.x
    current_pitch = msg.vector.y
    # ½Ç¶ÈµÍÍ¨ÂË²¨£¨ÓÃÓÚÏÞÎ»ÅÐ¶Ï£©
    filtered_roll = angle_lp_alpha * filtered_roll + (1.0 - angle_lp_alpha) * current_roll
    filtered_pitch = angle_lp_alpha * filtered_pitch + (1.0 - angle_lp_alpha) * current_pitch

def motor_cmd_callback(msg):
    global error_x_global, error_y_global
    if msg.mode == 0:
        error_x_global = msg.target_x
        error_y_global = msg.target_y
    else:
        yaw.target_pos = int(msg.target_x)
        yaw.steps_to_go = yaw.target_pos - yaw.current_pos
        yaw.state = 0
        yaw.mode = 1
        pitch.target_pos = int(msg.target_y)
        pitch.steps_to_go = pitch.target_pos - pitch.current_pos
        pitch.state = 0
        pitch.mode = 1

# ---------- GPIO ³õÊ¼»¯ ----------
def init_gpio():
    global pwm_yaw, pwm_pitch
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(YAW_DIR_PIN, GPIO.OUT)
    GPIO.setup(PITCH_DIR_PIN, GPIO.OUT)
    GPIO.setup(YAW_EN_PIN, GPIO.OUT)
    GPIO.setup(PITCH_EN_PIN, GPIO.OUT)
    GPIO.output(YAW_EN_PIN, GPIO.LOW)
    GPIO.output(PITCH_EN_PIN, GPIO.LOW)
    pwm_yaw = GPIO.PWM(YAW_PWM_PIN, 1000)
    pwm_pitch = GPIO.PWM(PITCH_PWM_PIN, 1000)
    pwm_yaw.start(0)
    pwm_pitch.start(0)
    return pwm_yaw, pwm_pitch

def clamp_speed(speed, max_speed):
    return max(-max_speed, min(max_speed, speed))

# ---------- ÏÞÎ»±£»¤£¨¸ú×ÙÄ£Ê½£©----------
def apply_angle_limits(speed_x, speed_y):
    global filtered_roll, filtered_pitch, roll_min, roll_max, pitch_min, pitch_max, hysteresis
    if filtered_roll >= roll_max - hysteresis and speed_x > 0:
        speed_x = 0
    elif filtered_roll <= roll_min + hysteresis and speed_x < 0:
        speed_x = 0
    if filtered_pitch >= pitch_max - hysteresis and speed_y > 0:
        speed_y = 0
    elif filtered_pitch <= pitch_min + hysteresis and speed_y < 0:
        speed_y = 0
    return speed_x, speed_y

# ---------- ºËÐÄ¿ØÖÆÑ­»· ----------
def control_loop(event):
    global pwm_yaw, pwm_pitch, pixel_speed_x, pixel_speed_y
    global last_error_x, last_error_y
    global error_x_global, error_y_global
    global target_visible, target_error_x, target_error_y
    global filtered_roll, filtered_pitch
    global smooth_speed_x, smooth_speed_y
    global kp_yaw, kd_yaw, kp_pitch, kd_pitch
    global err_ref, spd_ref, err_gain, spd_gain, lp_alpha, dt
    global max_freq, min_freq, accel_rate
    global scan_speed, scan_direction, roll_min, roll_max, pitch_min, pitch_max
    global speed_smooth_alpha, max_speed_change, use_angle_compensation, angle_comp_gain

    # ---- 1. ¸ù¾ÝÉãÏñÍ·×´Ì¬È·¶¨Ä¿±êÎó²î ----
    if target_visible:
        error_x_global = target_error_x
        error_y_global = target_error_y
    else:
        error_x_global = 0.0
        error_y_global = 0.0

    error_x = error_x_global
    error_y = error_y_global

    # ---- 2. ¿ÉÑ¡ÇãÐ±²¹³¥ ----
    if use_angle_compensation:
        error_x += angle_comp_gain * filtered_pitch

    # ---- 3. ¿¹»ý·Ö±¥ºÍ£ºÏÞÎ»Ê±Ç¯ÖÆÎó²î ----
    if target_visible:
        if filtered_roll >= roll_max and error_x > 0:
            error_x = 0
        elif filtered_roll <= roll_min and error_x < 0:
            error_x = 0
        if filtered_pitch >= pitch_max and error_y > 0:
            error_y = 0
        elif filtered_pitch <= pitch_min and error_y < 0:
            error_y = 0

    # ---- 4. ¼ÆËã PID ËÙ¶ÈÖ¸Áî ----
    raw_speed_x = (error_x - last_error_x) / dt
    raw_speed_y = (error_y - last_error_y) / dt
    pixel_speed_x += lp_alpha * (raw_speed_x - pixel_speed_x)
    pixel_speed_y += lp_alpha * (raw_speed_y - pixel_speed_y)
    last_error_x = error_x
    last_error_y = error_y

    err_factor_x = 1.0 + err_gain * (1.0 - math.exp(-abs(error_x) / err_ref))
    err_factor_y = 1.0 + err_gain * (1.0 - math.exp(-abs(error_y) / err_ref))
    spd_factor_x = 1.0 + spd_gain * (1.0 - math.exp(-abs(pixel_speed_x) / spd_ref))
    spd_factor_y = 1.0 + spd_gain * (1.0 - math.exp(-abs(pixel_speed_y) / spd_ref))
    kd_scale_x = 1.0 / (1.0 + abs(pixel_speed_x) / spd_ref)
    kd_scale_y = 1.0 / (1.0 + abs(pixel_speed_y) / spd_ref)

    dyn_kp_yaw = kp_yaw * err_factor_x * spd_factor_x
    dyn_kd_yaw = kd_yaw * err_factor_x * kd_scale_x
    dyn_kp_pitch = kp_pitch * err_factor_y * spd_factor_y
    dyn_kd_pitch = kd_pitch * err_factor_y * kd_scale_y

    speed_cmd_x = dyn_kp_yaw * error_x - dyn_kd_yaw * pixel_speed_x
    speed_cmd_y = dyn_kp_pitch * error_y - dyn_kd_pitch * pixel_speed_y

    # ---- 5. É¨ÃèÄ£Ê½¸²¸ÇÓë·½Ïò¹ÜÀí ----
    if not target_visible:
        # Ê¹ÓÃÂË²¨ºóµÄ½Ç¶ÈÅÐ¶Ï±ß½ç£¬´øÖÍ»Ø
        if filtered_roll >= roll_max - hysteresis and scan_direction > 0:
            scan_direction = -1
        elif filtered_roll <= roll_min + hysteresis and scan_direction < 0:
            scan_direction = 1
        speed_cmd_x = scan_direction * scan_speed
        speed_cmd_y = 0.0   # ´¹Ö±±£³Ö¾²Ö¹£¨¿ÉÀ©Õ¹£©
    else:
        # ¸ú×ÙÄ£Ê½£ºÓ¦ÓÃÏÞÎ»±£»¤
        speed_cmd_x, speed_cmd_y = apply_angle_limits(speed_cmd_x, speed_cmd_y)

    # ÏÞ·ùµ½×î´óÆµÂÊ
    speed_cmd_x = clamp_speed(speed_cmd_x, max_freq)
    speed_cmd_y = clamp_speed(speed_cmd_y, max_freq)

    # ---- 6. ËÙ¶ÈÆ½»¬£¨ËÙÂÊÏÞÖÆ + Ò»½×ÂË²¨£© ----
    # ËÙÂÊÏÞÖÆ
    if speed_cmd_x - smooth_speed_x > max_speed_change:
        speed_cmd_x = smooth_speed_x + max_speed_change
    elif smooth_speed_x - speed_cmd_x > max_speed_change:
        speed_cmd_x = smooth_speed_x - max_speed_change
    if speed_cmd_y - smooth_speed_y > max_speed_change:
        speed_cmd_y = smooth_speed_y + max_speed_change
    elif smooth_speed_y - speed_cmd_y > max_speed_change:
        speed_cmd_y = smooth_speed_y - max_speed_change

    # Ò»½×µÍÍ¨ÂË²¨
    smooth_speed_x = smooth_speed_x + speed_smooth_alpha * (speed_cmd_x - smooth_speed_x)
    smooth_speed_y = smooth_speed_y + speed_smooth_alpha * (speed_cmd_y - smooth_speed_y)

    # ½«Æ½»¬ºóµÄËÙ¶È¸³¸øµç»ú
    yaw.target_speed = smooth_speed_x
    pitch.target_speed = smooth_speed_y

    # ---- 7. Ö´ÐÐµç»úÔË¶¯£¨Yaw£© ----
    if yaw.mode == 0:
        speed_cmd = yaw.target_speed
        if speed_cmd > yaw.current_speed:
            yaw.current_speed += accel_rate * dt
            if yaw.current_speed > speed_cmd:
                yaw.current_speed = speed_cmd
        elif speed_cmd < yaw.current_speed:
            yaw.current_speed -= accel_rate * dt
            if yaw.current_speed < speed_cmd:
                yaw.current_speed = speed_cmd
        yaw.current_speed = clamp_speed(yaw.current_speed, max_freq)
        if abs(yaw.current_speed) <= min_freq:
            yaw.current_speed = 0

        GPIO.output(YAW_DIR_PIN, GPIO.HIGH if yaw.current_speed > 0 else GPIO.LOW)
        freq = abs(yaw.current_speed)
        if freq < 1.0:
            freq = 0
        if freq > 0:
            pwm_yaw.ChangeDutyCycle(50)
            pwm_yaw.ChangeFrequency(freq)
        else:
            pwm_yaw.ChangeDutyCycle(0)
        yaw.current_pos += yaw.current_speed * dt
    else:
        # Î»ÖÃÄ£Ê½£¨ÍêÕûÌÝÐÎ¼Ó¼õËÙ£©
        if yaw.state == 0:
            if yaw.steps_to_go != 0:
                yaw.state = 1
                yaw.accel_count = 0
                yaw.current_speed = 0
                yaw.target_speed = max_freq if yaw.steps_to_go > 0 else -max_freq
                GPIO.output(YAW_DIR_PIN, GPIO.HIGH if yaw.steps_to_go > 0 else GPIO.LOW)
        elif yaw.state == 1:
            yaw.current_speed += accel_rate * dt
            if abs(yaw.current_speed) >= abs(yaw.target_speed):
                yaw.current_speed = yaw.target_speed
                yaw.state = 2
            yaw.current_pos += yaw.current_speed * dt
            yaw.steps_to_go -= yaw.current_speed * dt
            if abs(yaw.steps_to_go) < 1:
                yaw.state = 0
                yaw.current_speed = 0
                yaw.steps_to_go = 0
        elif yaw.state == 2:
            remaining = abs(yaw.steps_to_go)
            decel_distance = (abs(yaw.current_speed)**2) / (2 * accel_rate)
            if remaining < decel_distance:
                yaw.state = 3
        elif yaw.state == 3:
            if yaw.current_speed > 0:
                yaw.current_speed -= accel_rate * dt
                if yaw.current_speed < 0:
                    yaw.current_speed = 0
            else:
                yaw.current_speed += accel_rate * dt
                if yaw.current_speed > 0:
                    yaw.current_speed = 0
            yaw.current_pos += yaw.current_speed * dt
            yaw.steps_to_go -= yaw.current_speed * dt
            if abs(yaw.steps_to_go) < 1 or abs(yaw.current_speed) < min_freq:
                yaw.state = 0
                yaw.current_speed = 0
                yaw.steps_to_go = 0
        freq = abs(yaw.current_speed)
        if freq < 1.0:
            freq = 0
        if freq > 0:
            pwm_yaw.ChangeDutyCycle(50)
            pwm_yaw.ChangeFrequency(freq)
        else:
            pwm_yaw.ChangeDutyCycle(0)

    # ---- 8. Ö´ÐÐµç»úÔË¶¯£¨Pitch£© ----
    if pitch.mode == 0:
        speed_cmd = pitch.target_speed
        if speed_cmd > pitch.current_speed:
            pitch.current_speed += accel_rate * dt
            if pitch.current_speed > speed_cmd:
                pitch.current_speed = speed_cmd
        elif speed_cmd < pitch.current_speed:
            pitch.current_speed -= accel_rate * dt
            if pitch.current_speed < speed_cmd:
                pitch.current_speed = speed_cmd
        pitch.current_speed = clamp_speed(pitch.current_speed, max_freq)
        if abs(pitch.current_speed) <= min_freq:
            pitch.current_speed = 0

        GPIO.output(PITCH_DIR_PIN, GPIO.HIGH if pitch.current_speed > 0 else GPIO.LOW)
        freq = abs(pitch.current_speed)
        if freq < 1.0:
            freq = 0
        if freq > 0:
            pwm_pitch.ChangeDutyCycle(50)
            pwm_pitch.ChangeFrequency(freq)
        else:
            pwm_pitch.ChangeDutyCycle(0)
        pitch.current_pos += pitch.current_speed * dt
    else:
        # Î»ÖÃÄ£Ê½
        if pitch.state == 0:
            if pitch.steps_to_go != 0:
                pitch.state = 1
                pitch.accel_count = 0
                pitch.current_speed = 0
                pitch.target_speed = max_freq if pitch.steps_to_go > 0 else -max_freq
                GPIO.output(PITCH_DIR_PIN, GPIO.HIGH if pitch.steps_to_go > 0 else GPIO.LOW)
        elif pitch.state == 1:
            pitch.current_speed += accel_rate * dt
            if abs(pitch.current_speed) >= abs(pitch.target_speed):
                pitch.current_speed = pitch.target_speed
                pitch.state = 2
            pitch.current_pos += pitch.current_speed * dt
            pitch.steps_to_go -= pitch.current_speed * dt
            if abs(pitch.steps_to_go) < 1:
                pitch.state = 0
                pitch.current_speed = 0
                pitch.steps_to_go = 0
        elif pitch.state == 2:
            remaining = abs(pitch.steps_to_go)
            decel_distance = (abs(pitch.current_speed)**2) / (2 * accel_rate)
            if remaining < decel_distance:
                pitch.state = 3
        elif pitch.state == 3:
            if pitch.current_speed > 0:
                pitch.current_speed -= accel_rate * dt
                if pitch.current_speed < 0:
                    pitch.current_speed = 0
            else:
                pitch.current_speed += accel_rate * dt
                if pitch.current_speed > 0:
                    pitch.current_speed = 0
            pitch.current_pos += pitch.current_speed * dt
            pitch.steps_to_go -= pitch.current_speed * dt
            if abs(pitch.steps_to_go) < 1 or abs(pitch.current_speed) < min_freq:
                pitch.state = 0
                pitch.current_speed = 0
                pitch.steps_to_go = 0
        freq = abs(pitch.current_speed)
        if freq < 1.0:
            freq = 0
        if freq > 0:
            pwm_pitch.ChangeDutyCycle(50)
            pwm_pitch.ChangeFrequency(freq)
        else:
            pwm_pitch.ChangeDutyCycle(0)

# ---------- Ö÷º¯Êý ----------
def main():
    global pwm_yaw, pwm_pitch, kp_yaw, kd_yaw, kp_pitch, kd_pitch
    global err_ref, spd_ref, err_gain, spd_gain, lp_alpha, dt
    global max_freq, min_freq, accel_rate
    global scan_speed, roll_min, roll_max, pitch_min, pitch_max
    global angle_lp_alpha, speed_smooth_alpha, max_speed_change, hysteresis
    global use_angle_compensation, angle_comp_gain
    global filtered_roll, filtered_pitch

    rospy.init_node('motor_controller', anonymous=True)
    rospy.loginfo("Motor controller starting...")

    # ¶ÁÈ¡²ÎÊý£¨ÓëÄúµÄÔ­Ê¼ launch ÍêÈ«Æ¥Åä£©
    kp_yaw = rospy.get_param("~kp_yaw", 20.0)
    kd_yaw = rospy.get_param("~kd_yaw", 200.0)
    kp_pitch = rospy.get_param("~kp_pitch", 20.0)
    kd_pitch = rospy.get_param("~kd_pitch", 140.0)
    err_ref = rospy.get_param("~err_ref", 20.0)
    spd_ref = rospy.get_param("~spd_ref", 15.0)
    err_gain = rospy.get_param("~err_gain", 2.0)
    spd_gain = rospy.get_param("~spd_gain", 1.5)
    lp_alpha = rospy.get_param("~lp_alpha", 0.3)
    dt = rospy.get_param("~dt", 0.02)
    max_freq = rospy.get_param("~max_freq", 9000)
    min_freq = rospy.get_param("~min_freq", 0)
    accel_rate = rospy.get_param("~accel_rate", 10000.0)

    # É¨ÃèÓëÏÞÎ»
    scan_speed = rospy.get_param("~scan_speed", 300)
    roll_min = rospy.get_param("~roll_min", -45.0)
    roll_max = rospy.get_param("~roll_max", 45.0)
    pitch_min = rospy.get_param("~pitch_min", -30.0)
    pitch_max = rospy.get_param("~pitch_max", 30.0)

    # ¸Ä½ø²ÎÊý
    angle_lp_alpha = rospy.get_param("~angle_lp_alpha", 0.5)
    speed_smooth_alpha = rospy.get_param("~speed_smooth_alpha", 0.3)
    max_speed_change = rospy.get_param("~max_speed_change", 500.0)
    hysteresis = rospy.get_param("~hysteresis", 1.0)
    use_angle_compensation = rospy.get_param("~use_angle_compensation", False)
    angle_comp_gain = rospy.get_param("~angle_comp_gain", 0.0)

    # ³õÊ¼»¯ÂË²¨±äÁ¿£¨Ê¹ÓÃµ±Ç°½Ç¶È³õÖµ£©
    filtered_roll = 0.0
    filtered_pitch = 0.0

    pwm_yaw, pwm_pitch = init_gpio()

    # ¶©ÔÄ»°Ìâ
    rospy.Subscriber('/motor_cmd', MotorCmd, motor_cmd_callback)
    rospy.Subscriber('/mpu6050/angles', Vector3Stamped, angle_callback)
    rospy.Subscriber('/target_visible', Bool, target_visible_cb)
    rospy.Subscriber('/target_error', Vector3, target_error_cb)   # Ê¹ÓÃ Vector3

    rospy.Timer(rospy.Duration(dt), control_loop)

    rospy.loginfo("Motor controller ready. Scan speed: %d, Limits: roll[%.1f, %.1f], pitch[%.1f, %.1f]",
                  scan_speed, roll_min, roll_max, pitch_min, pitch_max)
    rospy.spin()

    if pwm_yaw:
        pwm_yaw.stop()
    if pwm_pitch:
        pwm_pitch.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
