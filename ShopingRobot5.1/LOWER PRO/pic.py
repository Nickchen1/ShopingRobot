# coding:utf-8 
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################
import sys

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


class car_pic():
    def __init__(self):
        self.photo_path = '/home/nvidia/hjnew/hjcaff/py-faster-rcnn/data/demo/'
        self.op = 0
        self.starttimes = 5
        self.sleeptime = 1
        self.order_number = 0



    def get_PhotoPic(self):
        return self.photo_path


    def hough(self, picture):
        img = cv2.cvtColor(picture, cv2.COLOR_RGB2GRAY)
        # uu=rgb2gray(picture)
        #cv2.imshow("gray", img)
        # gradX = cv2.Sobel(gray, ddepth=cv2.CV_32F, dx = 1, dy = 0, ksize = -1)
        # gradY = cv2.Sobel(gray, ddepth = cv2.CV_32F, dx = 0, dy = 1, ksize = -1)

        # cv2.imshow("gradX", gradX)
        # x=cv2.waitKey(1)& 0xFF
        # cv2.imshow("gradY", gradY)
        # x=cv2.waitKey(1)
        # subtract the y-gradient from the x-gradient
        # gradient = cv2.subtract(gradX, gradY)
        # img = cv2.convertScaleAbs(gradient)
        img = cv2.blur(img, (3, 3))
        edges = cv2.Canny(img, 50, 150, apertureSize=3)
        # cv2.imshow('cannyuu',edges)
        lines = cv2.HoughLines(edges, 1, np.pi / 180, 118)  # ÕâÀï¶Ô×îºóÒ»¸ö²ÎÊýÊ¹ÓÃÁË¾­ÑéÐÍµÄÖµ
        if (lines is not None):
            for line in lines[0]:
                line_pix=line[0]
        else:
            line_pix=-1
        result = img.copy()
        shuipingx = []
        if (lines is not None):
            for lop in range(int(lines.size / 2)):
                for line in lines[lop]:
                    rho = line[0]  # µÚÒ»¸öÔªËØÊÇ¾àÀërho
                    theta = line[1]  # µÚ¶þ¸öÔªËØÊÇ½Ç¶Ètheta
                    print(rho)
                    print(theta)
                    if (theta < (np.pi / 4.)) or (theta > (3. * np.pi / 4.0)):  # ´¹Ö±Ö±Ïß
                        # ¸ÃÖ±ÏßÓëµÚÒ»ÐÐµÄ½»µã
                        pt1 = (int(rho / np.cos(theta)), 0)
                        # ¸ÃÖ±ÏßÓë×îºóÒ»ÐÐµÄ½¹µã
                        pt2 = (int((rho - result.shape[0] * np.sin(theta)) / np.cos(theta)), result.shape[0])
                        # »æÖÆÒ»Ìõ°×Ïß
                        cv2.line(result, pt1, pt2, 0, 1)
                    else:  # Ë®Æ½Ö±Ïß
                        # ¸ÃÖ±ÏßÓëµÚÒ»ÁÐµÄ½»µã
                        pt1 = (0, int(rho / np.sin(theta)))
                        # ¸ÃÖ±ÏßÓë×îºóÒ»ÁÐµÄ½»µã
                        pt2 = (result.shape[1], int((rho - result.shape[1] * np.cos(theta)) / np.sin(theta)))
                        # »æÖÆÒ»ÌõÖ±Ïß
                        if (pt2[0] - pt1[0] > 0):
                            cv2.line(result, pt1, pt2, 0, 5)
                            shuipingx.append(pt1[1])
                        print(('pt2=', pt2, 'pt1  =', pt1))
                # if shuipingx>0:
                #   break
        nowiq=datetime.datetime.now()
        if not os.path.isdir('hough'):
            os.makedirs('hough')
            print('create dir picture')
        cv2.imwrite('hough/%s_%s_%s_%s_%s_%s.jpg'%(nowiq.year,nowiq.month,nowiq.day,nowiq.hour,nowiq.minute,nowiq.second), result)
        return result, shuipingx,line_pix

    def imgread(self, picture, imgcutxia, r, g, b):
        x = (imgcutxia[:, :, 0] > r)
        y = (imgcutxia[:, :, 1] > g)
        z = (imgcutxia[:, :, 2] > b)
        imgcutxia[(x & y & z), :] = 0
        x = (imgcutxia[:, :, 0] < 8)
        y = (imgcutxia[:, :, 1] < 8)
        z = (imgcutxia[:, :, 2] < 8)
        imgcutxia[(x & y & z), :] = 0
        """
        NpKernel = np.uint8(np.zeros((3, 3)))
        for i in range(3):
            NpKernel[2, i] = 1  # ¸ÐÐ»chenpingjun1990µÄÌáÐÑ£¬ÏÖÔÚÊÇÕýÈ·µÄ
            NpKernel[i, 2] = 1
        imgcutxia = cv2.erode(imgcutxia, NpKernel)
        """
        x = (imgcutxia[:, :, 0] > 10)
        y = (imgcutxia[:, :, 1] > 10)
        z = (imgcutxia[:, :, 2] > 10)

        p = np.sum((x & y & z))
        return p

    def huojia(self, picture, pix, r, g, b):
        shang = 0
        xia = 0
        img = cv2.imread(picture)
        imgcutshang = img[140:300, 150:550]
        _, shuipingx,line_pix = self.hough(imgcutshang)
        if len(shuipingx) == 0:
            max1=120
            min2=-1
        else:
        	max1 = max(shuipingx)
        	min2 = min(shuipingx)
        
        print('shuipilllllllllllllllllllllllllllllllllllllllllllllllllllllngx', max1, min2)
        # imgcutshang = img[(140+max1-130):(140+max1+4), 150:550]
        imgcutshang = img[(140 + max1 - 130):(140 + max1 - 15), 200:480]
        cv2.imwrite('%s_yuanshang.jpg' % (picture), imgcutshang)
        # imgcutxia= img[140+max1+70:480, 150:550]
        imgcutxia = img[140 + max1 + 70:480, 200:480]
        cv2.imwrite('%s_yuanxia.jpg' % (picture), imgcutxia)

        # cv2.imwrite('%s_yuanxia.jpg' % (picture), imgcutxia)
        p1 = self.imgread(picture + 'shang', imgcutshang, r, g, b)
        #print(picture, 'shang has  ', p1)
        if (p1 > pix):
            #print(picture, 'shang is you')
            shang = 1
        #else:
            #print((picture, 'shang is wu'))
        p2 = self.imgread(picture + 'xia', imgcutxia, r, g, b)
        #print(picture, 'xia  has  ', p2)
        if (p2 > pix):
            #print(picture, 'xia is you')
            xia = 1
        #else:
            #print((picture, 'xia is wu'))
        cv2.imwrite('%s_shang.jpg' % (picture), imgcutshang)
        cv2.imwrite('%s_xia.jpg' % (picture), imgcutxia)
        #cv2.imwrite('maskshang.jpg', imgcutshang)
        #cv2.imwrite('maskxia.jpg', imgcutxia)
        return shang, xia, p1, p2,line_pix

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
        if not os.path.isdir('picture'):
            os.makedirs('picture')
            print('create dir picture')
        time.sleep(2)

    def huojia_take_pipeline(self, order, pic_time):
        pipstart = time.clock()
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
            picname = 2
            for lm in range(picname):
                cv2.imwrite('picture/%d.jpg' % (order), bg_removed)
	    nowi = datetime.datetime.now()
            if not os.path.isdir('picture1'):
                os.makedirs('picture1')
                print('create dir picture')
            cv2.imwrite('picture1/%s_%s_%s_%s_%s_%s.jpg'%(nowi.year,nowi.month,nowi.day,nowi.hour,nowi.minute,nowi.second), bg_removed)
            if time.clock() - pipstart > pic_time:
                print(order,'.jpg	has taken')
                break

    def huojia_close_pipeline(self):
        self.pipeline.stop()

    def huop(self, area, pix, r, g, b):
        # alist1=[]
        # alist2=[]
        pix1 = []
        pix2 = []
        pix3 = []
        pline=[]
        #start = time.clock()
        for kl in range(1, 7):
            print('huojia pic in picture/%d.jpg' % (kl))
            shang, xia, p1, p2,line_pix = self.huojia('picture/%d.jpg' % (kl), pix, r, g, b)
            # alist1.append([shang])
            #  alist2.append([xia])
            pix3.append(p1)
            pix3.append(p2)
            pix1.append(p1)
            pix2.append(p2)
            pline.append(line_pix)
        # alist=alist2+alist1
        pixt = pix1 + pix2
        pix3.sort()
        # print(alist)
        #print(pixt)
        print(area,(pixt))
        print(area,'line pix',pline)
        for vline in range(6):
            if pline[vline]>120 or pline[vline] == -1:
                pline[vline]=1
            elif (120-pline[vline])>12:
                pline[vline]=0              
        for xpo in pixt:
            if xpo == pix3[0] or xpo == pix3[1] or xpo == pix3[2]:
                pixt[pixt.index(xpo)] = 0
            else:
                pixt[pixt.index(xpo)] = 1

        # print(alist)
        #print(pixt)
        last_tiem = [[pixt[0], pixt[1], pixt[2], pixt[3], pixt[4], pixt[5]],
                     [pixt[6], pixt[7], pixt[8], pixt[9], pixt[10], pixt[11]
                      ]]

        # print(' last item',last_tiem)
        #end = time.clock()
        #print('cost time is', end - start)
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

        # ÏÔÊ¾ÉãÏñÍ·´ò¿ª³É¹¦ÐÅÏ¢
        if self.cap.isOpened():
            print('Open Camera Successful')

        print('start capture ', img.shape)

    def middle_take_cap(self, order_number):
        print('start take picture ')
        for l in range(self.starttimes):
            ret, frame = self.cap.read()
            time.sleep(self.sleeptime)
            cv2.imwrite(self.photo_path + '%d.jpg' % (order_number), frame)
        print(self.photo_path + '%d.jpg' % (order_number) + 'has been saves')

    def middle_close_cap(self):
        print('¹Ø±ÕÉãÏñÍ·')
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
        #print('kkkk', class_name, ' ', x1, ' ', x2, ' ', x3, ' ', x4)
        # if class_name != None :
        return x1, x2, x3, x4, class_name

    def demo(self, net, image_name):
        """Detect object classes in an image using pre-computed object proposals."""

        # Load the demo image
        im_file = os.path.join(cfg.DATA_DIR, 'demo', image_name)
        im = cv2.imread(im_file)
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
            if (x1 >= 50 and x1 < 1650):
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
        if area is "A":
			im_names = ['1.jpg']
        elif area is "B":
			im_names = ['2.jpg']
        elif area is "C":
			im_names = ['3.jpg']
        elif area is "D":
			im_names = ['4.jpg']
        #im_names = ['1.jpg', '2.jpg', '3.jpg', '4.jpg']
        self.dictf = {}
        for im_name in im_names:
            print
            '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
            #print 'Demo for data/demo/{}'.format(im_name)
            print(im_name)
            self.demo(self.net, im_name)

        itemlistp = {}
        itemlist = ('1.jpgleft', '1.jpgmid', '1.jpgright', '2.jpgleft', '2.jpgmid', '2.jpgright',
                    '3.jpgleft', '3.jpgmid', '3.jpgright', '4.jpgleft', '4.jpgmid', '4.jpgright')
        for klp in range(len(itemlist)):
            if itemlist[klp] in self.dictf:
                itemlistp[itemlist[klp]] = self.dictf[itemlist[klp]]
            else:
                itemlistp[itemlist[klp]] = 'None'
        item_list = {
            "A": {"left": itemlistp['1.jpgleft'], "mid": itemlistp['1.jpgmid'], "right": itemlistp['1.jpgright']},
            "B": {"left": itemlistp['2.jpgleft'], "mid": itemlistp['2.jpgmid'], "right": itemlistp['2.jpgright']},
            "C": {"left": itemlistp['3.jpgleft'], "mid": itemlistp['3.jpgmid'], "right": itemlistp['3.jpgright']},
            "D": {"left": itemlistp['4.jpgleft'], "mid": itemlistp['4.jpgmid'], "right": itemlistp['4.jpgright']}}

        if area is "A":
            result = {"left": itemlistp['1.jpgleft'], "mid": itemlistp['1.jpgmid'], "right": itemlistp['1.jpgright']}

            plt.savefig('ah.jpg')
            return result

        elif area is "B":
            result = {"left": itemlistp['2.jpgleft'], "mid": itemlistp['2.jpgmid'], "right": itemlistp['2.jpgright']}

            plt.savefig('bh.jpg')
            return result

        elif area is "C":
            result = {"left": itemlistp['3.jpgleft'], "mid": itemlistp['3.jpgmid'], "right": itemlistp['3.jpgright']}
            
            plt.savefig('ch.jpg')
            return result
        elif area is "D":
            result = {"left": itemlistp['4.jpgleft'], "mid": itemlistp['4.jpgmid'], "right": itemlistp['4.jpgright']}

            plt.savefig('dh.jpg')
            return result

        # plt.show()


# È«¾Ö±äÁ¿
CLASSES = ('__background__',  # always index 0
           'red cube', 'green cube', 'blue cube', 'yellow cube', 'yang le duo', 'xue bi', 'xue hua', 'shuang wai wai',
           'steel ball', 'tennis ball', 'badminton', 'apple')
NETS = {'vgg16': ('VGG16',
                  'VGG16_faster_rcnn_final.caffemodel'),
        'zf': ('ZF',
               '3318000ZF_faster_rcnn_final.caffemodel')}
if __name__ == '__main__':
    qstart=time.clock()
    tt = car_pic()
    tt.middle_load_cap()
    tt.load_model()#load fasterrcnn model

    #take middle pictur
    tt.middle_take_cap(1)#1ÎªÅÄÉãÐòºÅ
    tt.middle_take_cap(2)
    tt.middle_take_cap(3)
    tt.middle_take_cap(4)
    tt.middle_close_cap()
    #item_list¾ÍÊÇ·µ»Ø½á¹û
    result = tt.recon("A")
    print('A  :',result)
    result = tt.recon("B")
    print('B  :',result)
    result = tt.recon("C")
    print('C  :',result)
    result = tt.recon("D")
    print('D  :',result)
    tt.huojia_load_pipeline()
    for lll in range(1,7):
       tt.huojia_take_pipeline(lll,0.5)#lllÎªÅÄÉãÐòºÅ£¬0.5Îªwhile ³ÖÐøÊ±¼ätrue
    result = tt.huop("A", 500, 70, 70, 75)
    print(result)


    for lll in range(1,7):
       tt.huojia_take_pipeline(lll,0.5)
    result = tt.huop("B", 500, 70, 70, 75)
    print(result)


    for lll in range(1,7):
	   tt.huojia_take_pipeline(lll,0.5)
    result = tt.huop("C", 500, 70, 70, 75)
    print(result)


    for lll in range(1,7):
	   tt.huojia_take_pipeline(lll,0.5)
    result = tt.huop("D", 500, 70, 70, 75)
    print(result)
    tt.huojia_close_pipeline()
    """
    result = tt.huop("A", 500, 70, 70, 75)  # result ÎªÄãÒªµÄ½á¹ûµÄÐÎÊ½£¬500ÎªÅÐ¶ÏÓÐÎÞµÄÉèÖÃµã£¬70£¬70£¬85£¬ÎªÈ¥³ý»õ¼Ü°×É«µÄãÐÖµ£¬È«´óÓÚÊ±È¥³ý
    result = tt.huop("B", 500, 70, 70, 75)
    result = tt.huop("C", 500, 70, 70, 75)
    result = tt.huop("D", 500, 70, 70, 75)
    print(result)
    """
    print('cost time is    ',time.clock()-qstart)








