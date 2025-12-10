#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading

import rospy
from geometry_msgs.msg import Twist
import speech_recognition as sr

MIC_INDEX = 0  # Change if needed


class VoiceTeleop:
    def __init__(self):
        rospy.loginfo("VoiceTeleop __init__ started")

        rospy.init_node("voice_cmd_vel")
        rospy.loginfo("rospy.init_node succeeded")

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("/cmd_vel Publisher created")

        # Speech recognizer
        self.r = sr.Recognizer()
        self.r.dynamic_energy_threshold = True
        self.r.energy_threshold = 300
        self.r.pause_threshold = 0.5

        # Current velocity to maintain
        self.current_cmd = Twist()
        self.lock = threading.Lock()

        # Start publishing thread (10Hz)
        self.publish_rate = 10.0
        t = threading.Thread(target=self.publish_loop)
        t.daemon = True
        t.start()

        rospy.loginfo("VoiceTeleop initialization completed")

    def publish_loop(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            with self.lock:
                cmd = Twist()
                cmd.linear.x = self.current_cmd.linear.x
                cmd.angular.z = self.current_cmd.angular.z
            self.pub.publish(cmd)
            rate.sleep()

    def listen_once(self):
        try:
            with sr.Microphone(device_index=MIC_INDEX) as source:
                rospy.loginfo("Please speak... (forward/backward/left/right/stop)")
                audio = self.r.listen(source, phrase_time_limit=2.5)
        except AttributeError as e:
            rospy.logerr("Microphone AttributeError (ignoring): %s", e)
            rospy.sleep(0.5)
            return ""
        except Exception as e:
            rospy.logerr("Microphone/audio error: %s", e)
            rospy.sleep(0.5)
            return ""

        try:
            text = self.r.recognize_google(audio, language="ko-KR")
            text = text.strip()
            rospy.loginfo("Recognition result: %s", text)
            return text
        except Exception as e:
            rospy.logwarn("Speech recognition failed: %s", e)
            return ""

    def text_to_cmd(self, text: str):
        t = text.replace(" ", "")

        linear = 0.0
        angular = 0.0

        if any(k in t for k in ["앞으로", "전진", "앞으로가", "앞으로가줘"]):
            linear = 0.15
        elif any(k in t for k in ["뒤로", "후진", "뒤로가"]):
            linear = -0.15
        elif any(k in t for k in ["왼쪽", "좌회전", "왼쪽으로"]):
            angular = 0.6
        elif any(k in t for k in ["오른쪽", "우회전", "오른쪽으로"]):
            angular = -0.6
        elif any(k in t for k in ["정지", "멈춰", "스톱", "그만", "서"]):
            linear = 0.0
            angular = 0.0
        else:
            rospy.loginfo("Unknown command. (maintaining velocity)")
            return

        rospy.loginfo(f"Velocity updated: v={linear:.2f}, w={angular:.2f}")

        with self.lock:
            self.current_cmd.linear.x = linear
            self.current_cmd.angular.z = angular

    def run(self):
        rospy.loginfo("run() main loop started")
        while not rospy.is_shutdown():
            text = self.listen_once()
            if not text:
                continue
            self.text_to_cmd(text)
        rospy.loginfo("run() loop ended")


if __name__ == "__main__":
    try:
        node = VoiceTeleop()
        node.run()
    except Exception as e:
        print("==== Exception occurred, program terminated ====")
        import traceback
        traceback.print_exc()