#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
import os
import sys
import cv2
import argparse
import torch
import torch.backends.cudnn as cudnn
from playsound import playsound
from pathlib import Path
from utils import google_utils
from utils.datasets import *
from utils.utils import *
from std_msgs.msg import String
from std_msgs.msg import Int32
import Judge2Rooms
import Judge3Rooms

# 统计向量
statisticVec1 = {}
statisticVec2 = {}
statistic_vec_B = {}
statistic_vec_C = {}
statistic_vec_D = {}
statistic_vec_Fake = {}

# 路径
img_path = "/home/ucar/pic"
weights_path = "/home/ucar/yolov5-v1.0/weights/0730.pt"

# global
num = 0
ans = {}
flag = 0

#房间方案 F: 2R, T: 3R
cell_flag = False

# 每个房间拍照张数
picPerRoom = 8

# 拍照
def Take_Photo(position):
    global num
    num += 1
    cap = cv2.VideoCapture("/dev/ucar_video")
    rospy.loginfo("Picture-%s", str(num))
    # 设定图片尺寸，格式
    width = 640
    height = 480
    cap.set(3, width)
    cap.set(4, height)
    codec = cv2.VideoWriter.fourcc('M', 'J', 'P', 'G')
    cap.set(cv2.CAP_PROP_FOURCC, codec)
    ret, frame = cap.read()
    # cv2.waitKey(20)
    cv2.imwrite("/home/ucar/pic/" + "%02d" % num + ".jpg", frame)
    # if position == Int32(1):
    #     cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
    #     #cv2.imwrite("/home/ucar/pic/Room_B/" + "%02d"%num + ".jpg", frame)
    # elif position == Int32(2):
    #     cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
    #     #cv2.imwrite("/home/ucar/pic/Room_C/" + "%02d"%num + ".jpg", frame)
    # elif position == Int32(3):
    #     cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
    #     #cv2.imwrite("/home/ucar/pic/Room_D/" + "%02d"%num + ".jpg", frame)
    # else:
    #     rospy.loginfo("error!")
    cap.release()
    cv2.destroyAllWindows()


def detect(weights, source, conf_thre, save_img=False):
    opt = parse_opt(weights, source, conf_thre)
    out, source, weights, view_img, save_txt, imgsz = \
        opt.output, opt.source, opt.weights, opt.view_img, opt.save_txt, opt.img_size
    webcam = source == '0' or source.startswith('rtsp') or source.startswith('http') or source.endswith('.txt')

    global sofa
    global tableware
    global pet
    # Initialize
    device = torch_utils.select_device(opt.device)
    if os.path.exists(out):
        shutil.rmtree(out)  # delete output folder
    os.makedirs(out)  # make new output folder
    half = device.type != 'cpu'  # half precision only supported on CUDA

    # Load model
    google_utils.attempt_download(weights)
    model = torch.load(weights, map_location=device)['model'].float()  # load to FP32
    # torch.save(torch.load(weights, map_location=device), weights)  # update model if SourceChangeWarning
    # model.fuse()
    model.to(device).eval()
    if half:
        model.half()  # to FP16

    # Second-stage classifier
    classify = False
    if classify:
        modelc = torch_utils.load_classifier(name='resnet101', n=2)  # initialize
        modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model'])  # load weights
        modelc.to(device).eval()

    # Set Dataloader
    vid_path, vid_writer = None, None
    if webcam:
        view_img = True
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz)
    else:
        save_img = True
        dataset = LoadImages(source, img_size=imgsz)

    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]

    # Run inference
    t0 = time.time()
    img = torch.zeros((1, 3, imgsz, imgsz), device=device)  # init img
    _ = model(img.half() if half else img) if device.type != 'cpu' else None  # run once
    __count = 0
    for path, img, im0s, vid_cap in dataset:
        __count += 1
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = torch_utils.time_synchronized()
        pred = model(img, augment=opt.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
        t2 = torch_utils.time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
            else:
                p, s, im0 = path, '', im0s

            save_path = str(Path(out) / Path(p).name)
            s += '%gx%g ' % img.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  #  normalization gain whwh
            if det is not None and len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                for *xyxy, conf, cls in det:
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()
                    # 执行统计
                    # 2Rooms:
                    if cell_flag == False:
                        rospy.loginfo("judge 2")
                        # print(names[int(cls)])
                        # print("%g * 4"%*xywh)
                        if __count <= picPerRoom:
                            Judge2Rooms.statistic(statistic_vector=statisticVec1, label=names[int(cls)], val=conf, xywh=xywh)
                            print(names[int(cls)] + " in Room1    " + "p=%.4f" % conf + "\n")
                        elif picPerRoom < __count <= picPerRoom * 2:
                            Judge2Rooms.statistic(statistic_vector=statisticVec2, label=names[int(cls)], val=conf, xywh=xywh)
                            print(names[int(cls)] + " in Room2   " + "p=%.4f" % conf + "\n")

                    # 3Rooms:
                    else:
                        rospy.loginfo("judge 3")
                        print(names[int(cls)])
                        if __count <= picPerRoom:
                            Judge3Rooms.statistic(statistic_vector=statistic_vec_B, label=names[int(cls)], val=conf, xywh=xywh)
                            print(names[int(cls)] + " in RoomB    " + "p=%.4f" % conf + "\n")
                        elif picPerRoom < __count <= picPerRoom * 2:
                            Judge3Rooms.statistic(statistic_vector=statistic_vec_C, label=names[int(cls)], val=conf, xywh=xywh)
                            print(names[int(cls)] + " in RoomC   " + "p=%.4f" % conf + "\n")
                        else:
                            Judge3Rooms.statistic(statistic_vector=statistic_vec_D, label=names[int(cls)], val=conf, xywh=xywh)
                            print(names[int(cls)] + " in RoomD   " + "p=%.4f" % conf + "\n")


            print('%sDone. (%.3fs)' % (s, t2 - t1))

        print('Done. (%.3fs)' % (time.time() - t0))

        # print(statistic_vec)

def yolo_callback(msg):
    global cell_flag
    global flag
    global ans
    time1 = time.time()
    # 2Rooms
    # print(msg)
    # print(type(msg))
    if cell_flag == False:
        rospy.loginfo("Two Room!")
        Judge2Rooms.__init(statisticVec1, statisticVec2, msg)
        # 分别进行识别
        detect(source=img_path, weights=weights_path, conf_thre=0.3, save_img=False)
        # 进行判断
        ans = Judge2Rooms.judge(statisticVec1, statisticVec2)
    # 3Rooms
    else:
        rospy.loginfo("Three Room!")
        Judge3Rooms.__init(statistic_vec_B, statistic_vec_C, statistic_vec_D)
        detect(source=img_path, weights=weights_path, conf_thre=0.3, save_img=False)
        ans = Judge3Rooms.judge(statistic_vec_B, statistic_vec_C, statistic_vec_D)

    time2 = time.time()
    print("识别特征用时：", time2 - time1)
    print('\r\n')
    # time.sleep(0.5)    
    playsound("/home/ucar/WAV/open.wav")
    # time.sleep(0.5)
    bo_bao('room_B')
    # time.sleep(0.5)
    bo_bao('room_C')
    # time.sleep(0.5)
    bo_bao('room_D')
    # time.sleep(0.5)

def bo_bao(room_flag):
    global ans
    if room_flag == 'room_B':
        if ans['B'] == 'kitchen':
            playsound("/home/ucar/WAV/B_dinner.wav")
        elif ans['B'] == 'parlour':
            playsound("/home/ucar/WAV/B_parlour.wav")
        elif ans['B'] == 'bedroom':
            playsound("/home/ucar/WAV/B_bedroom.wav")

    elif room_flag == 'room_C':
        if ans['C'] == 'kitchen':
            playsound("/home/ucar/WAV/C_dinner.wav")
        elif ans['C'] == 'parlour':
            playsound("/home/ucar/WAV/C_parlour.wav")
        elif ans['C'] == 'bedroom':
            playsound("/home/ucar/WAV/C_bedroom.wav")
        
    elif room_flag == 'room_D':
        if ans['D'] == 'kitchen':
            playsound("/home/ucar/WAV/D_dinner.wav")
        elif ans['D'] == 'parlour':
            playsound("/home/ucar/WAV/D_parlour.wav")
        elif ans['D'] == 'bedroom':
            playsound("/home/ucar/WAV/D_bedroom.wav")
    else:
        rospy.loginfo("bobao error!")

def Room_Callback(tot_flag):
    global cell_flag
    if tot_flag == Int32(1):
        cell_flag = True
        rospy.loginfo("Edit successfully!")
    elif tot_flag == Int32(0):
        cell_flag = False
        rospy.loginfo("Successful edit!")
    else:
        rospy.loginfo("detect:Room selection error!")
    return cell_flag

def rosyolo():
    # 初始化ROS节点
    rospy.init_node('yolo', anonymous=True)
    print('yolo')
    _sign0=cv2.imread("/home/ucar/xuefeng/0.jpg")
    cv2.imshow('ready',_sign0)
    cv2.waitKey(10000)
    cv2.destroyAllWindows()
    # 创建话题订阅者
    # rospy.Subscriber('enter_room', Int32, Room_Callback)
    rospy.Subscriber('yolo_wake', Int32, yolo_callback)
    rospy.Subscriber('photo', Int32, Take_Photo)
    while not rospy.is_shutdown():
        rospy.spin()


def parse_opt(weights, source_path, conf_thre):
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default=weights, help='model.pt path')
    parser.add_argument('--source', type=str, default=source_path, help='source')  # file/folder, 0 for webcam
    parser.add_argument('--output', type=str, default='/home/ucar/pic-out/', help='output folder')  # output folder
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=conf_thre, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.5, help='IOU threshold for NMS')
    parser.add_argument('--fourcc', type=str, default='mp4v', help='output video codec (verify ffmpeg support)')
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--save-txt', default=False, action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt = parser.parse_args()
    opt.img_size = check_img_size(opt.img_size)
    # print(opt)
    return opt


if __name__ == '__main__':
    detect(weights=weights_path, source=r"/home/ucar/xuefeng", conf_thre=0.5, save_img=False)
    with torch.no_grad():
        rosyolo()

        # Update all models
        # for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt', 'yolov3-spp.pt']:
        #    detect()
        #    create_pretrained(opt.weights, opt.weights)