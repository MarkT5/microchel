import serial
import time
import threading as thr
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from homogen import matrixFromVectors
import cv2
from camera_props import *
from cv2 import aruco
import math

ANGLE_SPEED_SETTER = 0
TRAVEL_POINTS = 1


class ArUcoDetector:
    def __init__(self):
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.dictionary, self.parameters)
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.thread = thr.Thread(target=self.main)
        self.found_markers = {}
        self.id_updater = {}
        self.thread.start()
        self.robots = {}
        self.marker_sizes = {}

    def main(self):
        while thr.main_thread().is_alive():
            self.found_markers = {}
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray)
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            matrices = []

            if ids is not None:
                ids = ids.T[0]
                for i, c in enumerate(corners):
                    if ids[i] in self.marker_sizes.keys():
                        size = self.marker_sizes[ids[i]]
                    else:
                        size = 0.072
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, size, camera_matrix, distortion_coefficient)
                    image = cv2.drawFrameAxes(frame_markers, camera_matrix, distortion_coefficient, rvec, tvec, 0.08)
                    matrices.append(matrixFromVectors(tvec[0, 0] * 1000, rvec[0, 0]))
                cv2.imshow('frame', frame_markers)
                if 0 in ids:
                    invZero = np.linalg.inv(matrices[np.where(ids == 0)[0][0]])
                    for i in range(len(matrices)):
                        if ids[i] == 0:
                            continue
                        mat = matrices[i]
                        tr = invZero @ mat
                        tr = tr / tr[3, 3]
                        self.found_markers[ids[i]] = tr
                        if ids[i] in self.id_updater.keys():
                            self.id_updater[ids[i]](tr)
                for i in self.robots.keys():
                    if i in ids:
                        self.robots[i].movementAllowed = True
                    else:
                        self.robots[i].movementAllowed = False
                        self.robots[i].move(0, 0)

            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                cv2.destroyAllWindows()


class Robot:
    def __init__(self, port="/dev/ttyUSB0"):

        self.ser = serial.Serial(port, 115200, timeout=.1)
        time.sleep(2)

        self.angPID = np.array([0.0, 0, 0])
        self.angPIDk = np.array([110.0, 0, 0]) # 1 - 110, 0, 6000
        self.targetAng = 0
        self.lastAng = 0
        self.currAng = 0
        self.lastAngTime = time.time()

        self.distPID = np.array([0.0, 0, 0])
        self.distPIDk = np.array([0.6, 0., 0]) # 1 - 0.6, 0.4, 0
        self.lastDist = 0
        self.lastDistTime = time.time()
        self.maxSpeed = 70

        self.movementAllowed = False

        self.targetSpeed = 0
        self.pos = np.array([0, 0])
        self.robotID = 1
        self.mode = TRAVEL_POINTS
        self.targetPoint = [0, 0]
        self.nowTravelling = False
        self.pointThreshold = 40

    def move(self, fov, side):
        assert fov == int(fov)
        assert side == int(side)
        fov = int(fov)
        side = int(side)
        if self.ser.inWaiting():
            print(self.ser.readline())
        self.ser.write(b'm' + int(self.robotID-1).to_bytes(2, 'big', signed=True)+b'l' + (fov + side).to_bytes(2, 'big', signed=True)+b'r' + (fov - side).to_bytes(2, 'big', signed=True)+b'\r')
        #time.sleep(0.06)
        #print(b'm' + int(self.robotID-1).to_bytes(2, 'big', signed=True)+b'l' + (fov + side).to_bytes(2, 'big', signed=True)+b'r' + (fov - side).to_bytes(2, 'big', signed=True))
        # self.ser.write(b'l' + (fov + side).to_bytes(2, 'big', signed=True))
        # time.sleep(0.06)
        # self.ser.write(b'r' + (fov - side).to_bytes(2, 'big', signed=True))
        # time.sleep(0.06)

    def lift(self, val):
        assert val == int(val)
        assert 90 <= val <= 210
        self.ser.write(b'm' + int(self.robotID-1).to_bytes(2, 'big', signed=True))
        self.ser.write(b'u' + int(val).to_bytes(2, 'big', signed=True))

    def grab(self, val):
        assert val == int(val)
        assert 110 <= val <= 240
        self.ser.write(b'm' + int(self.robotID-1).to_bytes(2, 'big', signed=True))
        self.ser.write(b'g' + int(val).to_bytes(2, 'big', signed=True))

    def holdAng(self, ang):
        self.targetAng = ang

    def setSpeed(self, speed):
        self.targetSpeed = speed

    def goTo(self, point):
        self.targetPoint = point
        self.nowTravelling = True
        self.movementAllowed = True
        self.mode = TRAVEL_POINTS

    def update(self, tr):
        rob_rot = Rot.from_matrix(tr[:3, :3]).as_rotvec()
        self.pos = tr[:2, 3]
        self.currAng = rob_rot[2]
        self.updateAng(tr)
        if self.mode == TRAVEL_POINTS and self.nowTravelling:
            target = self.targetPoint - self.pos
            dist = np.linalg.norm(target)
            self.distPID[0] = dist
            if dist < 140:
                self.distPID[1] += dist
                self.distPID[1] = min(max(-150, self.distPID[1]), 150)
            else:
                self.distPID[1] = 0
            res = dist - self.lastDist
            if abs(res) < 500:
                self.distPID[2] = res * (time.time() - self.lastDistTime)
            else:
                self.distPID[2] = 0
            self.lastDist = dist
            self.lastDistTime = time.time()
            self.setSpeed(min(self.maxSpeed, int(np.sum(self.distPID * self.distPIDk))))
            self.holdAng(math.atan2(*target[::-1]))
            if int(np.linalg.norm(target)) < self.pointThreshold:
                self.nowTravelling = False
                self.move(0, 0)
            else:
                self.nowTravelling = True

    def updateAng(self, tr):
        if self.movementAllowed and (
                (self.mode == TRAVEL_POINTS and self.nowTravelling) or self.mode == ANGLE_SPEED_SETTER):
            aAng = self.currAng - self.targetAng
            if aAng > math.pi:
                aAng -= 2 * math.pi
            elif aAng < -math.pi:
                aAng += 2 * math.pi
            if abs(aAng) > 0.05:
                self.angPID[0] = aAng
                if abs(aAng) < 0.3:
                    self.angPID[1] += aAng
                    self.angPID[1] = min(max(-80, self.angPID[1]), 80)
                else:
                    self.angPID[1] = 0
                res = self.currAng - self.lastAng
                if self.lastAng > math.pi / 2 and self.currAng < -math.pi / 2:
                    res = math.pi * 2 + self.currAng - self.lastAng
                elif self.lastAng < -math.pi / 2 and self.currAng > math.pi / 2:
                    res = -math.pi / 2 * 2 + self.currAng - self.lastAng

                self.angPID[2] = res * (time.time() - self.lastAngTime)
                self.lastAng = self.currAng
                self.lastAngTime = time.time()
                if 2 * self.targetSpeed < abs(np.sum(self.angPID * self.angPIDk)):
                    resSpeed = 0
                else:
                    resSpeed = self.targetSpeed
                print(resSpeed, max(-self.targetSpeed, min(self.targetSpeed, -int(np.sum(self.angPID * self.angPIDk)))))
                self.move(resSpeed,
                          max(-self.targetSpeed, min(self.targetSpeed, -int(np.sum(self.angPID * self.angPIDk)))))

            else:
                self.move(self.targetSpeed, 0)
        else:
            self.move(0, 0)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
