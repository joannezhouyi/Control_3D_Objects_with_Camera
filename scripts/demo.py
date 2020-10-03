"""
Example script using PyOpenPose.
"""
import os
import time
import cv2
import math
import numpy as np
import ctypes
import pyglet
from pyglet.gl import *
import pywavefront
import threading
#OPENPOSE_ROOT = os.environ["OPENPOSE_ROOT"]
OPENPOSE_ROOT = "/home/jxgu/github/CamControl_3D_Objects/openpose"
import sys
sys.path.append("/home/jxgu/github/CamControl_3D_Objects/build/PyOpenPoseLib")
#os.environ["PYTHONPATH"] = "/home/jxgu/github/CamControl_3D_Objects/build/PyOpenPoseLib"
import LibShowModel
import PyOpenPose as OP

def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60., float(width)/height, 1., 100.)
    glMatrixMode(GL_MODELVIEW)
    return True


def update(dt):
    global rotation
    rotation += 90*dt
    if rotation > 720: rotation = 0

def angle(pt1, pt2):
    x1, y1 = pt1
    x2, y2 = pt2
    inner_product = x1*x2 + y1*y2
    len1 = math.hypot(x1, y1)
    len2 = math.hypot(x2, y2)
    #print(inner_product, len1, len2)
    if len1>0.0 and len2>0.0:
        if float(inner_product)/(float(len1)*float(len2))<1.0 and float(inner_product)/(float(len1)*float(len2))>0.0:
            return math.acos(float(inner_product)/(float(len1)*float(len2)))
        else:
            return 0.0
    else:
        return 0.0

def hand_open(hands_cord):
    i=0
    no1_0_x, no1_0_y = hands_cord[i + 0, 0], hands_cord[i + 0, 1]
    no1_1_x, no1_1_y = hands_cord[i + 1, 0], hands_cord[i + 1, 1]
    no1_2_x, no1_2_y = hands_cord[i + 2, 0], hands_cord[i + 2, 1]
    no1_3_x, no1_3_y = hands_cord[i + 3, 0], hands_cord[i + 3, 1]
    no1_4_x, no1_4_y = hands_cord[i + 4, 0], hands_cord[i + 4, 1]
    i = i + 5
    no2_0_x, no2_0_y = hands_cord[i + 0, 0], hands_cord[i + 0, 1]
    no2_1_x, no2_1_y = hands_cord[i + 1, 0], hands_cord[i + 1, 1]
    no2_2_x, no2_2_y = hands_cord[i + 2, 0], hands_cord[i + 2, 1]
    no2_3_x, no2_3_y = hands_cord[i + 3, 0], hands_cord[i + 3, 1]
    i = i + 4
    no3_0_x, no3_0_y = hands_cord[i + 0, 0], hands_cord[i + 0, 1]
    no3_1_x, no3_1_y = hands_cord[i + 1, 0], hands_cord[i + 1, 1]
    no3_2_x, no3_2_y = hands_cord[i + 2, 0], hands_cord[i + 2, 1]
    no3_3_x, no3_3_y = hands_cord[i + 3, 0], hands_cord[i + 3, 1]
    i = i + 4
    no4_0_x, no4_0_y = hands_cord[i + 0, 0], hands_cord[i + 0, 1]
    no4_1_x, no4_1_y = hands_cord[i + 1, 0], hands_cord[i + 1, 1]
    no4_2_x, no4_2_y = hands_cord[i + 2, 0], hands_cord[i + 2, 1]
    no4_3_x, no4_3_y = hands_cord[i + 3, 0], hands_cord[i + 3, 1]
    i = i + 4
    no5_0_x, no5_0_y = hands_cord[i + 0, 0], hands_cord[i + 0, 1]
    no5_1_x, no5_1_y = hands_cord[i + 1, 0], hands_cord[i + 1, 1]
    no5_2_x, no5_2_y = hands_cord[i + 2, 0], hands_cord[i + 2, 1]
    no5_3_x, no5_3_y = hands_cord[i + 3, 0], hands_cord[i + 3, 1]
    angle12 = angle([no1_4_y - no1_0_y, no1_4_x - no2_0_x], [no1_4_y - no2_0_y, no2_3_x - no2_0_x])
    angle23 = angle([no2_3_y - no2_0_y, no2_3_x - no2_0_x], [no3_3_y - no3_0_y, no3_3_x - no3_0_x])
    angle34 = angle([no3_3_y - no3_0_y, no3_3_x - no3_0_x], [no4_3_y - no4_0_y, no4_3_x - no4_0_x])
    angle45 = angle([no4_3_y - no4_0_y, no4_3_x - no4_0_x], [no5_3_y - no5_0_y, no5_3_x - no5_0_x])
    #print(angle23, angle34, angle45)
    if angle12 is not None and angle23 is not None and angle34 is not None:
        if angle12 + angle23 + angle34 + angle45<2.0:
            return False
        else:
            return True
    else:
        return False

def calculate(pt, ls):
    i = 2
    ang = -1.0
    for x in ls:
        pt2 = (x, i)
        i = i+1
        ang = angle(pt, pt2)*180/math.pi
        ang = ang * (-1)
    return ang

def showHeatmaps(hm, prefix="HeatMap "):
    h = np.sum(hm, axis=0)
    cv2.imshow(prefix, h)
    return h


def showPAFs(PAFs, startIdx=0, endIdx=16):

    for idx in range(startIdx, endIdx):
        X = PAFs[idx*2]
        Y = PAFs[idx*2+1]
        tmp = np.dstack((X, Y, np.zeros_like(X)))

        cv2.imshow("PAF "+str(idx), tmp)

def ComputeBB(hand, padding=1.5):
    minX = np.min(hand[:, 0])
    minY = np.min(hand[:, 1])

    maxX = np.max(hand[:, 0])
    maxY = np.max(hand[:, 1])

    width = maxX - minX
    height = maxY - minY

    cx = minX + width/2
    cy = minY + height/2

    width = height = max(width, height)
    width = height = width * padding

    minX = cx - width/2
    minY = cy - height/2


    score = np.mean(hand[:, 2])
    return score, [int(minX), int(minY), int(width), int(height)]

def run():
    ModelShow = LibShowModel.stShowModel()

    print('Model showing!')
    ThreadC = threading.Thread(target=ModelShow.vReflashModel)
    ThreadC.start()

    #cap = cv2.VideoCapture('/home/jxgu/github/CamControl_3D_Objects/test.mp4')
    cap = cv2.VideoCapture(0)
    download_heatmaps = False
    with_face = False 
    with_hands = True
    op = OP.OpenPose((320, 240), (240, 240), (640, 480), "COCO", OPENPOSE_ROOT + os.sep + "models" + os.sep, 0,
                     download_heatmaps, OP.OpenPose.ScaleMode.ZeroToOne, with_face, with_hands)

    actual_fps = 0
    paused = False
    delay = {True: 0, False: 1}
    hand_open_r = False
    hand_open_l = False

    print("Entering main Loop.")
    while True:
        start_time = time.time()
        try:
            ret, frame = cap.read()
            rgb = frame
            print(rgb.shape)

        except Exception as e:
            print("Failed to grab", e)
            break

        t = time.time()
        op.detectPose(rgb)
        #op.detectFace(rgb)
        op.detectHands(rgb)
        #op.detectHands(rgb, np.array(handBB + [0, 0, 0, 0], dtype=np.int32).reshape((1, 8)))
        t = time.time() - t
        op_fps = 1.0 / t

        res = op.render(rgb)
        cv2.putText(res, 'UI FPS = %f, OP FPS = %f, L:%s, R:%s' % (actual_fps, op_fps, hand_open_l, hand_open_r), (20, 20), 0, 0.5, (0, 0, 255))
        persons = op.getKeypoints(op.KeypointType.POSE)[0]

        if download_heatmaps and persons is not None:

            #faces = op.getFaceHeatmaps()
            left_hands, right_hands = op.getHandHeatmaps()
            #print("all face maps: ", faces.shape)
            #print("Left hand maps: ", left_hands.shape)
            #print("Right hand maps: ", right_hands.shape)

            # if no face or hand is detected for a person then the
            # corresponding heatmaps contain invalid data.
            # check the results from the hand/face keypoints to make
            # sure the corresponding object is correctly detected.
            #for pidx in range(len(persons)):
            #    hm = faces[pidx]
            #    showHeatmaps(hm, "Face"+str(pidx))
            #    showHeatmaps(left_hands[pidx], "Left Hand"+str(pidx))
            #    showHeatmaps(right_hands[pidx], "Right Hand"+str(pidx))

            # hm = op.getHeatmaps()
            # parts = hm[:18]
            # background = hm[18]
            # PAFs = hm[19:]  # each PAF has two channels (total 16 PAFs)
            # cv2.imshow("Right Wrist", parts[4])
            # cv2.imshow("background", background)
            # showPAFs(PAFs)

        if persons is not None and len(persons) > 0:
            print("First Person: ", persons[0].shape)

        cv2.imshow("OpenPose result", res)
        if op.getKeypoints(op.KeypointType.HAND)[0] is not None:
            leftHand = op.getKeypoints(op.KeypointType.HAND)[0].reshape(-1, 3)
            score_l, newHandBB_l = ComputeBB(leftHand)
            print("Left hand: Res Score, HandBB: ", score_l, newHandBB_l)
            hand_open_l = hand_open(op.getKeypoints(op.KeypointType.HAND)[0].reshape(-1, 3))

        if op.getKeypoints(op.KeypointType.HAND)[1] is not None:
            rightHand = op.getKeypoints(op.KeypointType.HAND)[1].reshape(-1, 3)
            score_r, newHandBB_r = ComputeBB(rightHand)
            print("Right hand: Res Score, HandBB: ", score_r, newHandBB_r)
            hand_open_r = hand_open(op.getKeypoints(op.KeypointType.HAND)[1].reshape(-1, 3))

            ModelShow.vCameraControl(int(persons[0][4][0])-320, int(persons[0][4][1])-240, int(persons[0][7][0])-320, int(persons[0][7][1])-240, 1, 1 if hand_open_l and hand_open_r else 0)

        key = cv2.waitKey(delay[paused])
        if key & 255 == ord('p'):
            paused = not paused

        if key & 255 == ord('q'):
            break

        actual_fps = 1.0 / (time.time() - start_time)

        os.system('wmctrl -a GLUT')

if __name__ == '__main__':
    run()
