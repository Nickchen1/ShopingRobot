# coding:utf-8 
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################
import sys, os

sy_root = '/home/nvidia/hjnew/librealsense-2.9.1/build/wrappers/python'

sys.path.insert(0, sy_root)


import _init_paths
from fast_rcnn.config import cfg
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from utils.timer import Timer
import matplotlib.pyplot as plt
import numpy as np
import scipy.io as sio
import caffe, os, sys, cv2
import argparse
import time
# First import the library
import pyrealsense2 as rs

# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import datetime
import cv2
import numpy as np
import time
from skimage import data, segmentation, measure, morphology, color

#240 and 240
class car_pic():
    hpix_on=300
    hpix_up=80
    hr = 70
    hg = 70
    hb = 75
    hppix1 = 520
    hppix2 = 30 #use cutline
    b_left=120
    b_right=120
    cutline_o_u = 22
    cutline_pix = 225
    def __init__(self):
        self.photo_path = '/home/nvidia/hjnew/hjcaff/py-faster-rcnn/data/demo/'
        self.op = 0
        self.starttimes = 5
        self.sleeptime = 1
        self.order_number = 0
        self.cannyshang = 150
        self.cannyxia = 240
        self.houghmax = 100



    def get_PhotoPic(self):
        return self.photo_path

    def hough(self, picture):
        img = cv2.cvtColor(picture, cv2.COLOR_RGB2GRAY)
        img = cv2.blur(img, (3, 3))
        edges = cv2.Canny(img, 30, 50, apertureSize=3)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 118)


        result = img.copy()
        shuipingx = []
        if (lines is not None):
            for lop in range(int(lines.size / 2)):
                for line in lines[lop]:
                    rho = line[0]
                    theta = line[1]

                    if (theta < (np.pi / 4.)) or (theta > (3. * np.pi / 4.0)):  # ´¹Ö±Ö±Ïß

                        pt1 = (int(rho / np.cos(theta)), 0)

                        pt2 = (int((rho - result.shape[0] * np.sin(theta)) / np.cos(theta)), result.shape[0])

                        cv2.line(result, pt1, pt2, 0, 1)
                    else:

                        pt1 = (0, int(rho / np.sin(theta)))

                        pt2 = (result.shape[1], int((rho - result.shape[1] * np.cos(theta)) / np.sin(theta)))

                        if (pt2[0] - pt1[0] > 0):
                            cv2.line(result, pt1, pt2, 0, 5)
                            shuipingx.append(pt1[1])



            line_pix=max(shuipingx)
        else:
            line_pix = -1
        return result, shuipingx, line_pix

    def imgread(self, picture, imgcutxia, r, g, b):
        x = (imgcutxia[:, :, 0] > r)
        y = (imgcutxia[:, :, 1] > g)
        z = (imgcutxia[:, :, 2] > b)
        imgcutxia[(x & y & z), :] = 0

        x = (imgcutxia[:, :, 0] > 30)
        y = (imgcutxia[:, :, 1] > 30)
        z = (imgcutxia[:, :, 2] > 30)

        p = np.sum((x | y |z))
        return p

    def huojia(self, picture, areas, hpix_shang,hpix_xia, r, g, b):
        shang = 0
        xia = 0
        img = cv2.imread(picture)
        cv2.imwrite(picture+'_'+areas+'.jpg',img)
        imgcutshang = img[140:300, 150:550]
        _, shuipingx, line_pix = self.hough(imgcutshang)
        if len(shuipingx) == 0:
            max1 = self.houghmax
            min2 = -1
        else:
            max1 = max(shuipingx)
            min2 = min(shuipingx)

        # print('shuipilllllllllllllllllllllllllllllllllllllllllllllllllllllngx', max1, min2)
        # imgcutshang = img[(140+max1-130):(140+max1+4), 150:550]
        imgcutshang = img[(140 + max1 - 130):(140 + max1 + 10), 140:500]

        # imgcutxia= img[140+max1+70:480, 150:550]
        imgcutxia = img[140 + max1 + 70:480, 140:500]


        p1 = self.imgread(picture + 'shang', imgcutshang, r, g, b)
        if (p1 > hpix_shang):
            shang = 1

        p2 = self.imgread(picture + 'xia', imgcutxia, r, g, b)
        if (p2 > hpix_xia):
            xia = 1


        return shang, xia, p1, p2, line_pix

    # Streaming loop
    def huojia_load_pipeline(self):
        self.pipeline = rs.pipeline()
        # Create a config and configure the pipeline to stream
        #  different resolutions of color and depth streams
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: ", depth_scale)

        # We will be removing the background of objects more than
        #  clipping_distance_in_meters meters away
        clipping_distance_in_meters = 0.65  # 1 meter
        self.clipping_distance = clipping_distance_in_meters / depth_scale

        # Create an align object
        # rs.align allows us to perform alignment of depth frames to others frames
        # The "align_to" is the stream type to which we plan to align depth frames.
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image
        # Align the depth frame to color frame
        aligned_frames = self.align.proccess(frames)
        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        print('pipeline set finish sleep 2 s ')

        time.sleep(1)

    def huojia_take_pipeline(self, order, pic_time):
        pipstart = time.clock()
        if not os.path.isdir('picture'):
            os.makedirs('picture')
            print('create dir picture')
        while True:
            # Get frameset of color and depth
            frames = self.pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image
            # Align the depth frame to color frame
            aligned_frames = self.align.proccess(frames)
            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()
            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 0
            depth_image_3d = np.dstack(
                (depth_image, depth_image, depth_image))  # depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > self.clipping_distance) | (depth_image_3d <= 0), grey_color,
                                  color_image)
            # Render images
            # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.09), cv2.COLORMAP_JET)
            # images = np.hstack((bg_removed, depth_colormap))
            # cv2.namedWindow('Align Example', cv2.WINDOW_AUTOSIZE)
            # cv2.imshow('Align Example', bg_removed)
            # key = cv2.waitKey(110)
            # print key
            bg_removed[0:100, 0:640] = 255
            cv2.imwrite('picture/%d.jpg' % (order), bg_removed)
            """
            nowi = datetime.datetime.now()
            if not os.path.isdir('picture1'):
                os.makedirs('picture1')
                print('create dir picture')
            cv2.imwrite('picture1/%s_%s_%s_%s_%s_%s.jpg' % (
            nowi.year, nowi.month, nowi.day, nowi.hour, nowi.minute, nowi.second), bg_removed)
            """
            if time.clock() - pipstart > pic_time:
                print(order, '.jpg	has taken')
                break

    def huojia_close_pipeline(self):
        print("huojia cap close")
        self.pipeline.stop()

    def cutline(self, gray2):
        labels = measure.label(gray2)
        x = 0
        for regio in measure.regionprops(labels):
            if ((regio.bbox[3] - regio.bbox[1]) > self.cutline_pix and (regio.bbox[2] - regio.bbox[0]) < self.cutline_o_u):
                # gray2[regio.coords]=0
                for xp in regio.coords:
                    gray2[xp[0], xp[1]] = 0
        return gray2
    def blackarea(self, picture, areas, judgepix1, judgepix2):
        try:
            image = cv2.imread(picture, 0)

            (_, thresh) = cv2.threshold(image, 15, 255, type=cv2.THRESH_BINARY)

            rect = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
            bw = cv2.dilate(thresh, rect)
            bw = cv2.dilate(bw, rect)
            bw = cv2.dilate(bw, rect)
            (_, bw) = cv2.threshold(bw, 15, 255, type=cv2.THRESH_BINARY_INV)
            cleared = bw.copy()
            segmentation.clear_border(cleared)
            labels = measure.label(cleared)
            #bnj = color.label2rgb(labels)

            black_area = []
            # print('regions number:', labels.max() + 1)
            pixblack_area=[]
            for regio in measure.regionprops(labels):
                if (regio.area > 6000 and regio.centroid[1] < 510 and regio.centroid[1] > 150 and regio.centroid[0]>80):
                    # print('regio    ', regio.centroid)
                    #                     # print('--------------')
                    black_area.append(regio.centroid[0])
                    black_area.append(regio.centroid[1])
                if (regio.area > 3000 and regio.centroid[1] < 600 and regio.centroid[1] > 50 and regio.centroid[0]>80):
                    pixblack_area.append([regio.centroid[0],regio.centroid[1],regio.area])

            #       print(regio.area)
            # cv2.waitKey(0)
            # return  black_area
            if (len(black_area) > 4):
                print(len(black_area), picture, "  has a problem")
                x1 = 0
                y1 = 0
                judge1 = 0
                judge2 = 0
                pix1 = 0
                pix2 = 0
                for lppl in range(len(black_area)):
                    if (lppl % 2 == 0):
                        if (black_area[lppl] < 260 ):
                            x1 = x1 + 1
                        else:
                            y1 = y1 + 1

                if (x1 > 1):
                    judge1 = 1
                if (y1 > 1):
                    judge2 = 1
                if (judge1 == 1 and judge2 == 1):
                    print('start  shang  and  xia  mode')
                    pix1 = -1
                    pix2 = -1
                    zhixiny = -1
                    return judge1, judge2, pix1, pix2, zhixiny
                elif ((judge1 == 1 and judge2 == 0)):
                    print('start   xia  mode')
                    pp = cv2.imread(picture)
                    blackareazhixin = int(black_area[len(black_area) - 1])
                    img2 = pp[int(black_area[len(black_area) - 2]):480, blackareazhixin - self.b_left:blackareazhixin + self.b_right]
                    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
                    gray2 = cv2.blur(gray2, (3, 3))
                    gray2 = cv2.Canny(gray2, 50, self.cannyxia, apertureSize=3)
                    gray2=self.cutline(gray2)
                    g2 = (gray2[:, :] > 10)
                    pix2 = np.sum(g2)

                    # cv2.imshow('gray2', gray2)

                    if pix2 > judgepix2:
                        judge2 = 1
                    zhixiny = blackareazhixin
                    return judge1, judge2, -1, pix2, zhixiny


                elif ((judge1 == 0 and judge2 == 1)):
                    print('start   shang  mode')
                    pp = cv2.imread(picture)
                    blackareazhixin = black_area[1]
                    img1 = pp[int(black_area[0]):280, blackareazhixin - 100:blackareazhixin + 100]
                    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
                    gray1 = cv2.blur(gray1, (3, 3))
                    gray1 = cv2.Canny(gray1, 50, self.cannyshang, apertureSize=3)

                    g1 = (gray1[:, :] > 10)
                    pix1 = np.sum(g1)
                    # print('  g1  =', np.sum(g1))
                    # cv2.imshow('gray1', gray1)

                    if pix1 > judgepix1:
                        judge1 = 1
                    zhixiny = black_area[1]
                    return judge1, judge2, pix1, -1, zhixiny
            elif (len(black_area) ==0 ):
                print('start  black_area  zero   area  mode')
                return 1, 1, -1, -1, -1
            elif (len(black_area) ==2):

                print('start  black_area  one   area  mode')
                if black_area[0]<260:
                    pp = cv2.imread(picture)
                    blackareazhixin = black_area[1]
                    img1 = pp[int(black_area[0]):280, blackareazhixin - 100:blackareazhixin + 100]
                    gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
                    gray1 = cv2.blur(gray1, (3, 3))
                    gray1 = cv2.Canny(gray1, 50, self.cannyshang, apertureSize=3)

                    g1 = (gray1[:, :] > 10)
                    pix1 = np.sum(g1)
                    # print('  g1  =', np.sum(g1))
                    # cv2.imshow('gray1', gray1)

                    if pix1 > judgepix1:
                        judge1 = 1
                    zhixiny = black_area[1]
                    return judge1, 1, pix1, -1, zhixiny
                else :
                    pp = cv2.imread(picture)
                    blackareazhixin = int(black_area[len(black_area) - 1])
                    img2 = pp[int(black_area[len(black_area) - 2]):480, blackareazhixin - self.b_left:blackareazhixin + self.b_right]
                    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
                    gray2 = cv2.blur(gray2, (3, 3))
                    gray2 = cv2.Canny(gray2, 50, self.cannyxia, apertureSize=3)
                    gray2=self.cutline(gray2)
                    g2 = (gray2[:, :] > 10)
                    pix2 = np.sum(g2)

                    # cv2.imshow('gray2', gray2)

                    if pix2 > judgepix2:
                        judge2 = 1
                    zhixiny = blackareazhixin
                    return 1, judge2, -1, pix2, zhixiny


            else:
                print('start  true   mode')
                judge1 = 0
                judge2 = 0
                g1 = 0
                g2 = 0
                zhixiny = 0
                pp = cv2.imread(picture)
                blackareazhixin = int((black_area[1] + black_area[3]) / 2)
                img1 = pp[int(black_area[0]):280, blackareazhixin - 100:blackareazhixin + 100]
                img2 = pp[int(black_area[2]):480, blackareazhixin - self.b_left:blackareazhixin + self.b_right]
                gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
                gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
                gray1 = cv2.blur(gray1, (3, 3))
                gray2 = cv2.blur(gray2, (3, 3))
                gray1 = cv2.Canny(gray1, 50, self.cannyshang, apertureSize=3)
                gray2 = cv2.Canny(gray2, 50, self.cannyxia, apertureSize=3)
                gray2 = self.cutline(gray2)
                g1 = (gray1[:, :] > 10)
                g2 = (gray2[:, :] > 10)
                pix1 = np.sum(g1)
                pix2 = np.sum(g2)
                # print('  g1  =', np.sum(g1))
                # print('  g2  =', np.sum(g2))

                if pix1 > judgepix1:
                    judge1 = 1
                if pix2 > judgepix2:
                    judge2 = 1
                zhixiny = blackareazhixin
                return judge1, judge2, pix1, pix2, zhixiny
        except:
            print('  yi chang  error  ')
            return 1, 1, -1, -1, -1

    def huop(self, area, hpix_shang=hpix_on,hpix_xia=hpix_up, r=hr, g=hg, b=hb, ppix1=hppix1, ppix2=hppix2):

        pix1 = []
        pix2 = []
        for kl in range(1, 7):
            # print('huojia pic in picture/%d.jpg' % (kl))
            shang, xia, p1, p2, line_pix = self.huojia('picture/%d.jpg' % (kl), area, hpix_shang,hpix_xia, r, g, b)
            judge1, judge2, bpix1, bpix2, zhixiny = self.blackarea('picture/%d.jpg' % (kl), area, ppix1, ppix2)

            print('the colorarea is', shang, xia, p1, p2, line_pix)
            print('the backarea is', judge1, judge2, bpix1, bpix2, zhixiny)
            pix1.append(shang + judge1)
            pix2.append(xia + judge2)

        pixt = pix1 + pix2
        pix3 = pix1 + pix2
        print('origin  pix', pixt)
        pix3.sort()
        qp = 0
        xqp = 0
        v_line = []
        while xqp < 3:
            for vline in range(12):
                if pixt[vline] == pix3[qp] and vline not in v_line and xqp < 3:
                    v_line.append(vline)
                    xqp = xqp + 1
            qp = qp + 1
        for vline in range(12):
            if vline in v_line:
                pixt[vline] = 0
            else:
                pixt[vline] = 1

        last_tiem = [[pixt[0], pixt[1], pixt[2], pixt[3], pixt[4], pixt[5]],
                     [pixt[6], pixt[7], pixt[8], pixt[9], pixt[10], pixt[11]
                      ]]
        return last_tiem

    def middle_load_cap(self):
        self.cap = cv2.VideoCapture(3)
        print(self.cap)
        self.cap.set(3, 1920)
        self.cap.set(4, 1080)
        if not self.cap.isOpened():
            self.cap.open()

        ret, img = self.cap.read()
        cv2.imwrite(self.photo_path + 'temp.jpg', img)

        if self.cap.isOpened():
            print('Open Camera Successful')
        if not os.path.isdir('middle_recpic'):
            os.makedirs('middle_recpic')
            print('create dir middle_recpic')

        print('start capture ', img.shape)

    def middle_take_cap(self, order_number=0):
        print('start take picture ')

        for l in range(self.starttimes):
            ret, frame = self.cap.read()
            time.sleep(self.sleeptime)
            frame[0:500, 0:1920] = 255
        self.ppimg = frame
        nowi = datetime.datetime.now()
        if not os.path.isdir('middle_takepic'):
            os.makedirs('middle_takepic')
            print('create dir middle_takepic')
        cv2.imwrite('middle_takepic/%s_%s_%s_%s_%s_%s.jpg' % (
            nowi.year, nowi.month, nowi.day, nowi.hour, nowi.minute, nowi.second), frame)
        print( 'middle_%d.jpg' % (order_number) + 'has been saves')

    def middle_close_cap(self):
        print('middle cap close')
        self.cap.release()

    def zuobiao(self, x1, x2, x3, x4):
        return x1, x2, x3, x4

    def vis_detections(self, im, class_name, dets, ax, thresh=0.5):
        """Draw detected bounding boxes."""
        inds = np.where(dets[:, -1] >= thresh)[0]
        x1 = 0
        x2 = 0
        x3 = 0
        x4 = 0
        if len(inds) == 0:
            return x1, x2, x3, x4, class_name

        # im = im[:, :, (2, 1, 0)]
        # fig, ax = plt.subplots(figsize=(12, 12))
        # ax.imshow(im, aspect='equal')
        for i in inds:
            bbox = dets[i, :4]
            score = dets[i, -1]
            x1, x2, x3, x4 = self.zuobiao(bbox[0], bbox[1], bbox[2], bbox[3])
            ax.add_patch(
                plt.Rectangle((bbox[0], bbox[1]),
                              bbox[2] - bbox[0],
                              bbox[3] - bbox[1], fill=False,
                              edgecolor='red', linewidth=3.5)
            )
            ax.text(bbox[0], bbox[1] - 2,
                    '{:s} {:.3f}'.format(class_name, score),
                    bbox=dict(facecolor='blue', alpha=0.5),
                    fontsize=14, color='white')

        ax.set_title(('{} detections with '
                      'p({} | box) >= {:.1f}').format(class_name, class_name,
                                                      thresh),
                     fontsize=14)
        # plt.axis('off')
        # plt.tight_layout()
        # plt.draw()
        # print('kkkk', class_name, ' ', x1, ' ', x2, ' ', x3, ' ', x4)
        # if class_name != None :
        return x1, x2, x3, x4, class_name

    def demo(self, net, image_name):
        """Detect object classes in an image using pre-computed object proposals."""

        # Load the demo image
        #im_file = os.path.join(cfg.DATA_DIR, 'demo', image_name)
        im = self.ppimg
        # Detect all object classes and regress object bounds
        timer = Timer()
        timer.tic()
        scores, boxes = im_detect(net, im)
        timer.toc()
        print('Detection took {:.3f}s for '
              '{:d} object proposals').format(timer.total_time, boxes.shape[0])

        # Visualize detections for each class
        CONF_THRESH = 0.6
        NMS_THRESH = 0.3
        im = im[:, :, (2, 1, 0)]
        fig, ax = plt.subplots(figsize=(12, 12))
        ax.imshow(im, aspect='equal')
        for cls_ind, cls in enumerate(CLASSES[1:]):
            cls_ind += 1  # because we skipped background
            cls_boxes = boxes[:, 4 * cls_ind:4 * (cls_ind + 1)]
            cls_scores = scores[:, cls_ind]
            dets = np.hstack((cls_boxes,
                              cls_scores[:, np.newaxis])).astype(np.float32)
            keep = nms(dets, NMS_THRESH)
            dets = dets[keep, :]

            x1, x2, x3, x4, class_name = self.vis_detections(im, cls, dets, ax, thresh=CONF_THRESH)

            if (x1 >= 50 and x1 < 1750):
                self.sortdict.append(x1)
                self.sortdict.append(class_name)

                print(class_name, 'the  pic  x1=   ', x1)
                if x1 < 800:
                    # print >> f, 'left', class_name
                    self.dictf[image_name + 'left'] = class_name
                elif x1 < 1200:
                    # print >> f, 'mid', class_name
                    self.dictf[image_name + 'mid'] = class_name
                else:
                    # print >> f, 'right', class_name
                    self.dictf[image_name + 'right'] = class_name
        plt.axis('off')
        plt.tight_layout()
        plt.draw()

    def load_model(self):
        t = time.time()
        cfg.TEST.HAS_RPN = True  # Use RPN for proposals

        prototxt = os.path.join(cfg.MODELS_DIR, NETS["zf"][0],
                                'faster_rcnn_alt_opt', 'faster_rcnn_test.pt')
        caffemodel = os.path.join(cfg.DATA_DIR, 'faster_rcnn_models',
                                  NETS["zf"][1])

        if not os.path.isfile(caffemodel):
            raise IOError(('{:s} not found.\nDid you run ./data/script/'
                           'fetch_faster_rcnn_models.sh?').format(caffemodel))

        caffe.set_mode_gpu()
        caffe.set_device(0)
        cfg.GPU_ID = 0
        self.net = caffe.Net(prototxt, caffemodel, caffe.TEST)
        print('\n\nLoaded network {:s}'.format(caffemodel))

    def recon(self, area):
        # Warmup on a dummy image
        im = 128 * np.ones((300, 500, 3), dtype=np.uint8)
        for i in xrange(2):
            _, _ = im_detect(self.net, im)
        im_names = ['1.jpg']

        # im_names = ['1.jpg', '2.jpg', '3.jpg', '4.jpg']

        for im_name in im_names:
            print
            '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
            # print 'Demo for data/demo/{}'.format(im_name)
            print('Demo for data/demo/'+area+'.jpg')
            self.dictf = {}
            self.dictf['1.jpgleft']='None'
            self.dictf['1.jpgmid']='None'
            self.dictf['1.jpgright']='None'
            self.sortdict = []
            itemlistp = {}
            self.demo(self.net, im_name)
            if len(self.sortdict) == 6:
                if self.sortdict[0] > self.sortdict[2]:
                    temp = self.sortdict[2]
                    self.sortdict[2] = self.sortdict[0]
                    self.sortdict[0] = temp
                    temp = self.sortdict[3]
                    self.sortdict[3] = self.sortdict[1]
                    self.sortdict[1] = temp
                if self.sortdict[0] > self.sortdict[4]:
                    temp = self.sortdict[4]
                    self.sortdict[4] = self.sortdict[0]
                    self.sortdict[0] = temp
                    temp = self.sortdict[5]
                    self.sortdict[5] = self.sortdict[1]
                    self.sortdict[1] = temp
                if self.sortdict[2] > self.sortdict[4]:
                    temp = self.sortdict[2]
                    self.sortdict[2] = self.sortdict[4]
                    self.sortdict[4] = temp
                    temp = self.sortdict[3]
                    self.sortdict[3] = self.sortdict[5]
                    self.sortdict[5] = temp
                result = {"Left": self.sortdict[1], "Middle": self.sortdict[3],
                          "Right": self.sortdict[5]}
            else:
                result = {"Left": self.dictf['1.jpgleft'], "Middle": self.dictf['1.jpgmid'],
                          "Right": self.dictf['1.jpgright']}
        nowi = datetime.datetime.now()
        plt.savefig(area+'_h.jpg')
        plt.savefig('middle_recpic/%s_%s_%s_%s_%s_%s.jpg' % (
            nowi.year, nowi.month, nowi.day, nowi.hour, nowi.minute, nowi.second))
        return result


CLASSES = ('__background__',  # always index 0
           'RedCube', 'GreenCube', 'BlueCube', 'YellowCube', 'YangLeDuo', 'Sprite', 'Snow', 'ShuangWaiWai',
           'SteelBall', 'Tennis', 'Badminton', 'Apple')
NETS = {'vgg16': ('VGG16',
                  'VGG16_faster_rcnn_final.caffemodel'),
        'zf': ('ZF',
               '42416000ZF_faster_rcnn_final.caffemodel')}
if __name__ == '__main__':
    qstart = time.clock()
    tt = car_pic()

    tt.middle_load_cap()
    tt.load_model()  # load fasterrcnn model

    # take middle pictur
    tt.middle_take_cap()
    result = tt.recon("A")
    print('A  :', result)
    #tt.middle_take_cap(2)
    tt.middle_take_cap()
    result = tt.recon("B")
    print('B :', result)
    tt.middle_take_cap()
    result = tt.recon("C")
    print('C  :', result)
    tt.middle_take_cap()
    result = tt.recon("D")
    print('D :', result)
    tt.middle_close_cap()

 


  
    tt.huojia_load_pipeline()
   
    for lll in range(1, 7):
        tt.huojia_take_pipeline(lll, 0.5)

    result = tt.huop("A")
    print(result)

    for lll in range(1, 7):
        tt.huojia_take_pipeline(lll, 0.5)
    result = tt.huop("B")
    print(result)
 
    for lll in range(1, 7):
        tt.huojia_take_pipeline(lll, 0.5)
    result = tt.huop("C")
    print(result)

    for lll in range(1, 7):
        tt.huojia_take_pipeline(lll, 0.5)
    result = tt.huop("D")
    print(result)
    
    tt.huojia_close_pipeline()

    print('cost time is    ', time.clock() - qstart)
    #








