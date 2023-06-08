import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

import csv
import copy
#import argparse
import itertools
from collections import Counter
from collections import deque

import cv2 as cv
import numpy as np
import mediapipe as mp

#from utils import CvFpsCalc
#import ure.KeyPointClassifier
#import PointHistoryClassifier

class KeyPointClassifier(object):
    def __init__(
        self,
        model_path='keypoint_classifier/keypoint_classifier.tflite',
        num_threads=1,
    ):
        self.interpreter = tf.lite.Interpreter(model_path=model_path,
                                               num_threads=num_threads)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

    def __call__(
        self,
        landmark_list,
    ):
        input_details_tensor_index = self.input_details[0]['index']
        self.interpreter.set_tensor(
            input_details_tensor_index,
            np.array([landmark_list], dtype=np.float32))
        self.interpreter.invoke()

        output_details_tensor_index = self.output_details[0]['index']

        result = self.interpreter.get_tensor(output_details_tensor_index)

        result_index = np.argmax(np.squeeze(result))

        return result_index

class PointHistoryClassifier(object):
    def __init__(
        self,
        model_path='point_history_classifier/point_history_classifier.tflite',
        score_th=0.5,
        invalid_value=0,
        num_threads=1,
    ):
        self.interpreter = tf.lite.Interpreter(model_path=model_path,
                                               num_threads=num_threads)

        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.score_th = score_th
        self.invalid_value = invalid_value

    def __call__(
        self,
        point_history,
    ):
        input_details_tensor_index = self.input_details[0]['index']
        self.interpreter.set_tensor(
            input_details_tensor_index,
            np.array([point_history], dtype=np.float32))
        self.interpreter.invoke()

        output_details_tensor_index = self.output_details[0]['index']

        result = self.interpreter.get_tensor(output_details_tensor_index)

        result_index = np.argmax(np.squeeze(result))

        if np.squeeze(result)[result_index] < self.score_th:
            result_index = self.invalid_value

        return result_index

class HandGesture(Node):

    def init_not_working(self):
        super().__init__('hand_gesture')
        self.publisher_ = self.create_publisher(PoseStamped, '/tracking_pose', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.eef_x = 0.28
        self.eef_y = -0.2
        self.eef_z = 0.5
        self.eef_t = 0.0
        self.eef_u = 0.0
        self.eef_v = 0.0
        self.eef_w = 1.0
        self.cap = cv.VideoCapture(2)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 320)
        self.mp_hands = mp.solutions
        # By using python3_mediapipe binding
        # It errors out: module 'mediapipe.python.solutions' has no attribute 'Hands'
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.keypoint_classifier = KeyPointClassifier()

        self.point_history_classifier = PointHistoryClassifier()

        # Read labels
        with open('keypoint_classifier/keypoint_classifier_label.csv',
                encoding='utf-8-sig') as f:
            self.keypoint_classifier_labels = csv.reader(f)
            self.keypoint_classifier_labels = [
                row[0] for row in self.keypoint_classifier_labels
            ]
        with open(
            'point_history_classifier/point_history_classifier_label.csv',
                encoding='utf-8-sig') as f:
            self.point_history_classifier_labels = csv.reader(f)
            self.point_history_classifier_labels = [
                row[0] for row in self.point_history_classifier_labels
            ]

        # FPS Measurement
        #self.cvFpsCalc = CvFpsCalc(buffer_len=10)

        # history
        history_length = 16
        self.point_history = deque(maxlen=history_length)

        self.finger_gesture_history = deque(maxlen=history_length)

    def __init__(self):
        super().__init__('hand_gesture')
        self.publisher_ = self.create_publisher(PoseStamped, '/tracking_pose', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.eef_x = 0.28
        self.eef_y = -0.2
        self.eef_z = 0.5
        self.eef_t = 0.0
        self.eef_u = 0.0
        self.eef_v = 0.0
        self.eef_w = 1.0
        self.cap = cv.VideoCapture(2)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 320)

    def timer_callback(self):

        pose = PoseStamped()
        pose.pose.position.x = self.eef_x
        pose.pose.position.y = self.eef_y
        pose.pose.position.z = self.eef_z
        pose.pose.position.z = self.eef_z
        pose.pose.orientation.x = self.eef_t
        pose.pose.orientation.y = self.eef_u
        pose.pose.orientation.z = self.eef_v
        pose.pose.orientation.w = self.eef_w

        self.publisher_.publish(pose)
        self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1


    def timer_callback_ros(self):
        #fps = self.cvFpsCalc.get()
        ret, image = self.cap.read()
        if not ret:
            return
        image = cv.flip(image, 1)  # Mirror display
        debug_image = copy.deepcopy(image)

        image = cv.cvtColor(image, cv.COLOR_BGR2RGB)

        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True


        if results.multi_hand_landmarks is not None:
            for hand_landmarks, handedness in zip(results.multi_hand_landmarks,
                                                  results.multi_handedness):
                # Bounding box calculation
                brect = calc_bounding_rect(debug_image, hand_landmarks)
                # Landmark calculation
                landmark_list = calc_landmark_list(debug_image, hand_landmarks)

                # Conversion to relative coordinates / normalized coordinates
                pre_processed_landmark_list = pre_process_landmark(
                    landmark_list)
                pre_processed_point_history_list = pre_process_point_history(
                    debug_image, point_history)

                # Hand sign classification
                hand_sign_id = keypoint_classifier(pre_processed_landmark_list)
                if hand_sign_id == 2:  # Point gesture
                    point_history.append(landmark_list[8])  # coordinate
                else:
                    point_history.append([0, 0])

                # Finger gesture classification
                finger_gesture_id = 0
                point_history_len = len(pre_processed_point_history_list)
                if point_history_len == (history_length * 2):
                    finger_gesture_id = point_history_classifier(
                        pre_processed_point_history_list)

                # Calculates the gesture IDs in the latest detection
                finger_gesture_history.append(finger_gesture_id)
                most_common_fg_id = Counter(
                    finger_gesture_history).most_common()

                # drawing
                debug_image = draw_bounding_rect(use_brect, debug_image, brect)
                debug_image = draw_landmarks(debug_image, landmark_list)
                debug_image = draw_info_text(
                    debug_image,
                    brect,
                    handedness,
                    keypoint_classifier_labels[hand_sign_id],
                    point_history_classifier_labels[most_common_fg_id[0][0]],
                )
                print(str(count) + ':' + keypoint_classifier_labels[hand_sign_id])
        else:
            point_history.append([0, 0])

        #debug_image = draw_point_history(debug_image, point_history)
        #debug_image = draw_info(debug_image, fps, 1, number)
        debug_image = draw_info(debug_image, 1, number)

        cv.imshow('WMD2023', debug_image)
        count = count + 1

        pose = PoseStamped()
        pose.pose.position.x = self.eef_x
        pose.pose.position.y = self.eef_y
        pose.pose.position.z = self.eef_z
        pose.pose.position.z = self.eef_z
        pose.pose.orientation.x = self.eef_t
        pose.pose.orientation.y = self.eef_u
        pose.pose.orientation.z = self.eef_v
        pose.pose.orientation.w = self.eef_w
        self.publisher_.publish(pose)
        self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1

    def calc_bounding_rect(image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_array = np.empty((0, 2), int)

        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)

            landmark_point = [np.array((landmark_x, landmark_y))]

            landmark_array = np.append(landmark_array, landmark_point, axis=0)

            x, y, w, h = cv.boundingRect(landmark_array)

        return [x, y, x + w, y + h]

    def calc_landmark_list(image, landmarks):
        image_width, image_height = image.shape[1], image.shape[0]

        landmark_point = []

        # Keypoint
        for _, landmark in enumerate(landmarks.landmark):
            landmark_x = min(int(landmark.x * image_width), image_width - 1)
            landmark_y = min(int(landmark.y * image_height), image_height - 1)
            # landmark_z = landmark.z

            landmark_point.append([landmark_x, landmark_y])

        return landmark_point

    def pre_process_landmark(landmark_list):
        temp_landmark_list = copy.deepcopy(landmark_list)

        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, landmark_point in enumerate(temp_landmark_list):
            if index == 0:
                base_x, base_y = landmark_point[0], landmark_point[1]

            temp_landmark_list[index][0] = temp_landmark_list[index][0] - base_x
            temp_landmark_list[index][1] = temp_landmark_list[index][1] - base_y

        # Convert to a one-dimensional list
        temp_landmark_list = list(
            itertools.chain.from_iterable(temp_landmark_list))

        # Normalization
        max_value = max(list(map(abs, temp_landmark_list)))

        def normalize_(n):
            return n / max_value

        temp_landmark_list = list(map(normalize_, temp_landmark_list))

        return temp_landmark_list

    def pre_process_point_history(image, point_history):
        image_width, image_height = image.shape[1], image.shape[0]

        temp_point_history = copy.deepcopy(point_history)

        # Convert to relative coordinates
        base_x, base_y = 0, 0
        for index, point in enumerate(temp_point_history):
            if index == 0:
                base_x, base_y = point[0], point[1]

            temp_point_history[index][0] = (temp_point_history[index][0] -
                                        base_x) / image_width
            temp_point_history[index][1] = (temp_point_history[index][1] -
                                        base_y) / image_height

        temp_point_history = list(
            itertools.chain.from_iterable(temp_point_history))

        return temp_point_history

    def draw_landmarks(image, landmark_point):
        # connecting lines
        if len(landmark_point) > 0:
            # thumb
            cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[3]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[3]), tuple(landmark_point[4]),
                (255, 255, 255), 2)

            # index finger
            cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[6]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[6]), tuple(landmark_point[7]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[7]), tuple(landmark_point[8]),
                (255, 255, 255), 2)

            # middle finger
            cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[10]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[10]), tuple(landmark_point[11]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[11]), tuple(landmark_point[12]),
                (255, 255, 255), 2)

            # ring finger
            cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[14]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[14]), tuple(landmark_point[15]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[15]), tuple(landmark_point[16]),
                (255, 255, 255), 2)

            # little finger
            cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[18]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[18]), tuple(landmark_point[19]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[19]), tuple(landmark_point[20]),
                (255, 255, 255), 2)

            # palm
            cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[0]), tuple(landmark_point[1]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[1]), tuple(landmark_point[2]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[2]), tuple(landmark_point[5]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[5]), tuple(landmark_point[9]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[9]), tuple(landmark_point[13]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[13]), tuple(landmark_point[17]),
                (255, 255, 255), 2)
            cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (0, 0, 0), 6)
            cv.line(image, tuple(landmark_point[17]), tuple(landmark_point[0]),
                (255, 255, 255), 2)

        # key points
        for index, landmark in enumerate(landmark_point):
            if index == 0:  # # Wrist 1
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 1:  # Wrist 2
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 2:  # Thumb: Root
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 3:  # Thumb: 1st joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 4:  # Thumb: fingertip
                cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
            if index == 5:  # Index finger: Root
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 6:  # Index finger: 2nd joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 7:  # Index finger: 1st joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 8:  # Index finger: fingertip
                cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
            if index == 9:  # Middle finger: Root
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 10:  # Middle finger: 2nd joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 11:  # Middle finger: 1st joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 12:  # Middle finger: point first
                cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
            if index == 13:  # Ring finger: Root
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 14:  # Ring finger: 2nd joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 15:  # Ring finger: 1st joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 16:  # Ring finger: fingertip
                cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)
            if index == 17:  # Little finger: base
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 18:  # Little finger: 2nd joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 19:  # Little finger: 1st joint
                cv.circle(image, (landmark[0], landmark[1]), 5, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 5, (0, 0, 0), 1)
            if index == 20:  # Little finger: point first
                cv.circle(image, (landmark[0], landmark[1]), 8, (255, 255, 255),
                      -1)
                cv.circle(image, (landmark[0], landmark[1]), 8, (0, 0, 0), 1)

        return image

    def draw_bounding_rect(use_brect, image, brect):
        if use_brect:
            cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[3]),
                     (0, 0, 0), 1)

        return image

    def draw_info_text(image, brect, handedness, hand_sign_text,
                   finger_gesture_text):
        cv.rectangle(image, (brect[0], brect[1]), (brect[2], brect[1] - 22),
                 (0, 0, 0), -1)

        info_text = handedness.classification[0].label[0:]
        if hand_sign_text != "":
            info_text = info_text + ':' + hand_sign_text
        cv.putText(image, info_text, (brect[0] + 5, brect[1] - 4),
               cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

        if finger_gesture_text != "":
            cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv.LINE_AA)
            cv.putText(image, "Finger Gesture:" + finger_gesture_text, (10, 60),
                   cv.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2,
                   cv.LINE_AA)

        return image

    #def draw_info(image, fps, mode, number):
    def draw_info(image, mode, number):
       # cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
       #        1.0, (0, 0, 0), 4, cv.LINE_AA)
       # cv.putText(image, "FPS:" + str(fps), (10, 30), cv.FONT_HERSHEY_SIMPLEX,
       #        1.0, (255, 255, 255), 2, cv.LINE_AA)

        return image

def main(args=None):
    rclpy.init(args=args)

    hand_gesture = HandGesture()

    rclpy.spin(hand_gesture)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_gesture.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
