import cv2
import mediapipe as mp
import pyautogui
import math
from enum import IntEnum

pyautogui.FAILSAFE = False
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands


class Gest(IntEnum):
    FIST = 0
    PINKY = 1
    RING = 2
    MID = 4
    LAST3 = 7
    INDEX = 8
    FIRST2 = 12
    LAST4 = 15
    THUMB = 16
    PALM = 31
    V_GEST = 33
    TWO_FINGER_CLOSED = 34


class HLabel(IntEnum):
    MINOR = 0
    MAJOR = 1


class HandRecog:
    def __init__(self, hand_label):
        self.finger = 0
        self.ori_gesture = Gest.PALM
        self.prev_gesture = Gest.PALM
        self.frame_count = 0
        self.hand_result = None
        self.hand_label = hand_label

    def update_hand_result(self, hand_result):
        self.hand_result = hand_result

    def get_signed_dist(self, point):
        sign = -1
        if self.hand_result.landmark[point[0]].y < self.hand_result.landmark[point[1]].y:
            sign = 1
        dist = (self.hand_result.landmark[point[0]].x - self.hand_result.landmark[point[1]].x) ** 2
        dist += (self.hand_result.landmark[point[0]].y - self.hand_result.landmark[point[1]].y) ** 2
        dist = math.sqrt(dist)
        return dist * sign

    def get_dist(self, point):
        dist = (self.hand_result.landmark[point[0]].x - self.hand_result.landmark[point[1]].x) ** 2
        dist += (self.hand_result.landmark[point[0]].y - self.hand_result.landmark[point[1]].y) ** 2
        dist = math.sqrt(dist)
        return dist

    def get_dz(self, point):
        return abs(self.hand_result.landmark[point[0]].z - self.hand_result.landmark[point[1]].z)

    def set_finger_state(self):
        if self.hand_result is None:
            return
        points = [[8, 5, 0], [12, 9, 0], [16, 13, 0], [20, 17, 0]]
        self.finger = 0
        self.finger = self.finger | 0  # thumb
        for idx, point in enumerate(points):
            dist = self.get_signed_dist(point[:2])
            dist2 = self.get_signed_dist(point[1:])
            try:
                ratio = round(dist / dist2, 1)
            except:
                ratio = round(dist / 0.01, 1)
            self.finger = self.finger << 1
            if ratio > 0.5:
                self.finger = self.finger | 1

    def get_gesture(self):
        if self.hand_result is None:
            return Gest.PALM
        current_gesture = Gest.PALM
        if Gest.FIRST2 == self.finger:
            point = [[8, 12], [5, 9]]
            dist1 = self.get_dist(point[0])
            dist2 = self.get_dist(point[1])
            ratio = dist1 / dist2
            if ratio > 1.7:
                current_gesture = Gest.V_GEST
            else:
                if self.get_dz([8, 12]) < 0.1:
                    current_gesture = Gest.TWO_FINGER_CLOSED
                else:
                    current_gesture = Gest.MID
        else:
            current_gesture = self.finger
        if current_gesture == self.prev_gesture:
            self.frame_count += 1
        else:
            self.frame_count = 0
        self.prev_gesture = current_gesture
        if self.frame_count > 4:
            self.ori_gesture = current_gesture
        return self.ori_gesture


class Controller:
    tx_old = 0
    ty_old = 0
    trial = True
    flag = False
    grabflag = False
    framecount = 0
    prev_hand = None

    def get_position(hand_result):
        point = 9
        position = [hand_result.landmark[point].x, hand_result.landmark[point].y]
        sx, sy = pyautogui.size()
        x_old, y_old = pyautogui.position()
        x = int(position[0] * sx)
        y = int(position[1] * sy)
        if Controller.prev_hand is None:
            Controller.prev_hand = x, y
        delta_x = x - Controller.prev_hand[0]
        delta_y = y - Controller.prev_hand[1]
        distsq = delta_x ** 2 + delta_y ** 2
        ratio = 1
        Controller.prev_hand = [x, y]
        if distsq <= 25:
            ratio = 0
        elif distsq <= 900:
            ratio = 0.07 * (distsq ** (1 / 2))
        else:
            ratio = 2.1
        x, y = x_old + delta_x * ratio, y_old + delta_y * ratio
        return (x, y)

    def handle_controls(gesture, hand_result):
        x, y = None, None
        if gesture != Gest.PALM:
            x, y = Controller.get_position(hand_result)
        if gesture != Gest.FIST and Controller.grabflag:
            Controller.grabflag = False
            pyautogui.mouseUp(button="left")
        if gesture == Gest.V_GEST:
            Controller.flag = True
            pyautogui.moveTo(x, y, duration=0.1)
        elif gesture == Gest.FIST:
            if not Controller.grabflag:
                Controller.grabflag = True
                pyautogui.mouseDown(button="left")
            pyautogui.moveTo(x, y, duration=0.1)
        elif gesture == Gest.MID and Controller.flag:
            print("Left Click")
            pyautogui.click()
            Controller.flag = False
        elif gesture == Gest.INDEX and Controller.flag:
            print("Right Click")
            pyautogui.click(button='right')
            Controller.flag = False
        elif gesture == Gest.TWO_FINGER_CLOSED and Controller.flag:
            print("Double Click")
            pyautogui.doubleClick()
            Controller.flag = False


class GestureController:
    gc_mode = 0
    cap = None
    CAM_HEIGHT = None
    CAM_WIDTH = None
    hr = None

    def __init__(self):
        GestureController.gc_mode = 1
        GestureController.cap = cv2.VideoCapture(0)
        GestureController.CAM_HEIGHT = GestureController.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        GestureController.CAM_WIDTH = GestureController.cap.get(cv2.CAP_PROP_FRAME_WIDTH)

    def start(self):
        handmajor = HandRecog(HLabel.MAJOR)
        with mp_hands.Hands(max_num_hands=1,
                            min_detection_confidence=0.5,
                            min_tracking_confidence=0.5) as hands:
            while GestureController.cap.isOpened() and GestureController.gc_mode:
                success, image = GestureController.cap.read()
                if not success:
                    print("Ignoring empty camera frame.")
                    continue
                image = cv2.cvtColor(cv2.flip(image, 1), cv2.COLOR_BGR2RGB)
                image.flags.writeable = False
                results = hands.process(image)
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                if results.multi_hand_landmarks:
                    GestureController.hr = results.multi_hand_landmarks[0]
                    handmajor.update_hand_result(GestureController.hr)
                    handmajor.set_finger_state()
                    gest_name = handmajor.get_gesture()
                    Controller.handle_controls(gest_name, handmajor.hand_result)
                    for hand_landmarks in results.multi_hand_landmarks:
                        mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                else:
                    Controller.prev_hand = None
                cv2.imshow('Gesture Controller', image)
                if cv2.waitKey(5) & 0xFF == ord('q'):
                    break
        GestureController.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    gc1 = GestureController()
    gc1.start()
