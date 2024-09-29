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
from Judge import judge,statistic,__init

# 统计向量
statistic_vec_B = {}
statistic_vec_C = {}
statistic_vec_D = {}
statistic_vec_Fake = {}
#
img_path = "/home/ucar/pic"
weights_path = "/home/ucar/yolov5-v1.0/weights/20220713.pt"

# global
num = 0

# 拍照
def Take_Photo(Position):
    global num
    num += 1
    cap = cv2.VideoCapture("/dev/ucar_video")
    rospy.loginfo("Picture-%s", str(num))
    # 设定图片尺寸，格式
    width = 640
    height = 480
    cap.set(3,  width)
    cap.set(4,  height)
    codec = cv2.VideoWriter.fourcc('M',  'J',  'P',  'G')
    cap.set(cv2.CAP_PROP_FOURCC, codec)           
    ret, frame = cap.read()
    cv2.waitKey(20)
    if Position == Int32(1):
        cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
        #cv2.imwrite("/home/ucar/pic/Room_B/" + "%02d"%num + ".jpg", frame)
    elif Position == Int32(2):
        cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
        #cv2.imwrite("/home/ucar/pic/Room_C/" + "%02d"%num + ".jpg", frame)
    elif Position == Int32(3):
        cv2.imwrite("/home/ucar/pic/" + "%02d"%num + ".jpg", frame)
        #cv2.imwrite("/home/ucar/pic/Room_D/" + "%02d"%num + ".jpg", frame)
    else:
        rospy.loginfo("error!")
    cap.release()
    cv2.destroyAllWindows()

def detect(weights,source,conf_thre,save_img = False):
    opt = parse_opt(weights,source,conf_thre)
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
        __count+=1
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

                # # Print results
                # for c in det[:, -1].unique():
                #     n = (det[:, -1] == c).sum()  # detections per class
                #     if(int(c) == 6):
                #         sofa += 1
                #     if(int(c) == 1):
                #         tableware += 1
                #     if(int(c) == 4):
                #         pet += 1
                    
                    # s += '%g %ss, ' % (n, names[int(c)])  # add to string

                # Write results
                
                for *xyxy, conf, cls in det:
                    
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()

                    # 执行统计
                    print(names[int(cls)])
                    if __count<=8:
                        statistic(statistic_vector=statistic_vec_B,label=names[int(cls)],val=conf,xywh=xywh)
                        print(names[int(cls)] +"in RoomB    "+"p=%.4f"%conf)
                    elif __count >8 and __count< 16:
                        statistic(statistic_vector=statistic_vec_C,label=names[int(cls)],val=conf,xywh=xywh)
                        print(names[int(cls)] +"in RoomC   "+"p=%.4f"%conf)
                    else:
                        statistic(statistic_vector=statistic_vec_D,label=names[int(cls)],val=conf,xywh=xywh)
                        print(names[int(cls)] +"in RoomD   "+"p=%.4f"%conf)


                    #if save_txt:  # Write to file
                    #    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    #    with open(save_path[:save_path.rfind('.')] + '.txt', 'a') as file:
                     #       file.write(('%g ' * 5 + '\n') % (cls, *xywh))  # label format

                    #if save_img or view_img:  # Add bbox to image
                    #    label = '%s %.2f' % (names[int(cls)], conf)
                    #    plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=3)

            # Print time (inference + NMS)
            print('%sDone. (%.3fs)' % (s, t2 - t1))

            # Stream results
           # if view_img:
            #    cv2.imshow(p, im0)
            #    if cv2.waitKey(1) == ord('q'):  # q to quit
             #       raise StopIteration

            # Save results (image with detections)
            #if save_img:
              #  if dataset.mode == 'images':
             #       cv2.imwrite(save_path, im0)
             #   else:
              #      if vid_path != save_path:  # new video
              #          vid_path = save_path
              #          if isinstance(vid_writer, cv2.VideoWriter):
              #              vid_writer.release()  # release previous video writer

              #          fps = vid_cap.get(cv2.CAP_PROP_FPS)
              #          w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
              #          h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
              #          vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*opt.fourcc), fps, (w, h))
              #      vid_writer.write(im0)

        #if save_txt or save_img:
            #print('Results saved to %s' % os.getcwd() + os.sep + out)
            #if platform == 'darwin':  # MacOS
                #os.system('open ' + save_path)
                #print('Done. (%.3fs)' % (time.time() - t0))
        print('Done. (%.3fs)' % (time.time() - t0))

        # print(statistic_vec)

def bobao(obj,num):
    name = './' + obj +'.wav'
    if num == 0:
        playsound("./math0.wav")
    elif num == 1:
        playsound("./math1.wav")
    elif num == 2:
        playsound("./math2.wav")
    elif num == 3:
        playsound("./math3.wav")
    elif num == 4:
        playsound("./math4.wav")
    elif num >= 5:
        playsound("./math5.wav")
    playsound(name)



def yolo_callback(msg):

    time1 = time.time()
    __init(statistic_vec_B,statistic_vec_C,statistic_vec_D)
    # 分别进行识别
    detect(source=img_path, weights=weights_path,conf_thre=0.5,save_img = False)
    #detect(source=img_path + "/"+"Room_C/", statistic_vec=statistic_vec_C, weights=weights_path,conf_thre=0.5,save_img = False)
    #detect(source=img_path + "/"+"Room_D/", statistic_vec=statistic_vec_D, weights=weights_path,conf_thre=0.5,save_img = False)
    # 进行判断
    ans = judge(statistic_vec_B, statistic_vec_C, statistic_vec_D)

    time2 = time.time()
    print("识别特征用时：",time2 - time1)
    print('\r\n')
    flag = 1


def final_callback(msg):
    global flag
    global tableware
    global sofa
    global pet
    while flag == 0:
        time.sleep(1)
        pass
    time.sleep(0.5)
    playsound("./open.wav")
    time.sleep(0.5)
    bobao('tableware',tableware)
    time.sleep(0.5)
    bobao('sofa',sofa)
    time.sleep(0.5)
    bobao('pet',pet)
    time.sleep(0.5)
    flag = 0


def rosyolo():
  #初始化ROS节点
  rospy.init_node('yolo', anonymous=True)
  print('yolo')
  # 创建话题订阅者
  rospy.Subscriber('yolo_wake', Int32, yolo_callback)
  #rospy.Subscriber('bobao', Int32, final_callback)
  rospy.Subscriber('photo', Int32, Take_Photo)
  while not rospy.is_shutdown():
    rospy.spin()

def parse_opt(weights,source_path,conf_thre):

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
    parser.add_argument('--save-txt',default=False,  action='store_true', help='save results to *.txt')
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    opt = parser.parse_args()
    opt.img_size = check_img_size(opt.img_size)
    #print(opt)
    return opt


if __name__ == '__main__':

    detect(weights=weights_path,source=r"/home/ucar/xuefeng",conf_thre=0.5,save_img = False)
    with torch.no_grad():
        rosyolo()

        # Update all models
        # for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt', 'yolov3-spp.pt']:
        #    detect()
        #    create_pretrained(opt.weights, opt.weights)
