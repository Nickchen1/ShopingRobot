import os.path as osp
import sys

def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

this_dir = osp.dirname(__file__)
dir_base,dir = osp.split(this_dir)
dir_base = osp.join(dir_base, 'faster-rcnn.pytorch')
# Add lib to PYTHONPATH
lib_path = osp.join(dir_base, 'lib')
add_path(lib_path)

coco_path = osp.join(dir_base, 'data', 'coco', 'PythonAPI')
add_path(coco_path)

import os
import sys
import numpy as np
import argparse
import pprint
import pdb
import time
import cv2
import torch
from torch.autograd import Variable
import torch.nn as nn
import torch.optim as optim

import torchvision.transforms as transforms
import torchvision.datasets as dset
from scipy.misc import imread
from roi_data_layer.roidb import combined_roidb
from roi_data_layer.roibatchLoader import roibatchLoader
from model.utils.config import cfg, cfg_from_file, cfg_from_list, get_output_dir
from model.rpn.bbox_transform import clip_boxes
from model.nms.nms_wrapper import nms
from model.rpn.bbox_transform import bbox_transform_inv
from model.utils.net_utils import save_net, load_net, vis_detections
from model.utils.blob import im_list_to_blob
from model.faster_rcnn.vgg16 import vgg16
from model.faster_rcnn.resnet import resnet
import pdb

try:
    xrange  # Python 2
except NameError:
    xrange = range  # Python 3


class Object_Detection:
    def __init__(self):
        this_dir = osp.dirname(__file__)
        dir_base, dir = osp.split(this_dir)
        dir_base = osp.join(dir_base, 'faster-rcnn.pytorch')
        os.chdir(dir_base)
        self.dataset = 'pascal_voc'
        self.cfg_file = 'cfgs/res101.yml'
        self.net = 'res101'
        self.set_cfgs = None
        self.image_dir = "images"
        self.cuda = True
        self.mGPUs = False
        self.vis  = False
        self.class_agnostic = 0
        self.parallel_type = 0
        self.checksession = 1
        self.checkepoch = 200
        self.checkpoint = 227
        self.batch_size = 2
        self.thresh = 0.05
        self.visThresh = 0.65
        self.lr = cfg.TRAIN.LEARNING_RATE
        self.momentum = cfg.TRAIN.MOMENTUM
        self.weight_decay = cfg.TRAIN.WEIGHT_DECAY
        self.fasterRCNN = None
        self.pascal_classes = np.asarray([  '__background__', # always index 0
                                            'redbrick', 'yakult', 'tennis', 'bluebrick',
                                            'cool', 'rubikcube', 'greenbrick', 'admilk', 'lehu',
                                            'snowbeer', 'terunsu', 'redbull'])

        self.model_path = '/home/robot/pytorch_venv_pip/faster-rcnn.pytorch/' \
                          'models/res101/pascal_voc/' \
                          'faster_rcnn_%d_%d_%d.pth' % (self.checksession, self.checkepoch, self.checkpoint)


    def Load_model(self):
        if self.cfg_file is not None:
            cfg_from_file(self.cfg_file)
        if self.set_cfgs is not None:
            cfg_from_list(self.set_cfgs)

        cfg.USE_GPU_NMS = self.cuda
        print('Using config:')
        pprint.pprint(cfg)
        np.random.seed(cfg.RNG_SEED)

        # initilize the network here.
        self.fasterRCNN = resnet(self.pascal_classes, 101, pretrained=False, class_agnostic=self.class_agnostic)
        self.fasterRCNN.create_architecture()

        if self.cuda > 0:
            checkpoint = torch.load(self.model_path)
        else:
            checkpoint = torch.load(self.model_path, map_location=(lambda storage, loc: storage))
        self.fasterRCNN.load_state_dict(checkpoint['model'])

        if 'pooling_mode' in checkpoint.keys():
            cfg.POOLING_MODE = checkpoint['pooling_mode']
        print('load model successfully!')


    def _get_image_blob(self,im):
        """Converts an image into a network input.
        Arguments:
          im (ndarray): a color image in BGR order
        Returns:
          blob (ndarray): a data blob holding an image pyramid
          im_scale_factors (list): list of image scales (relative to im) used
            in the image pyramid
        """
        im_orig = im.astype(np.float32, copy=True)
        im_orig -= cfg.PIXEL_MEANS

        im_shape = im_orig.shape
        im_size_min = np.min(im_shape[0:2])
        im_size_max = np.max(im_shape[0:2])

        processed_ims = []
        im_scale_factors = []

        for target_size in cfg.TEST.SCALES:
            im_scale = float(target_size) / float(im_size_min)
            # Prevent the biggest axis from being more than MAX_SIZE
            if np.round(im_scale * im_size_max) > cfg.TEST.MAX_SIZE:
                im_scale = float(cfg.TEST.MAX_SIZE) / float(im_size_max)
            im = cv2.resize(im_orig, None, None, fx=im_scale, fy=im_scale,
                            interpolation=cv2.INTER_LINEAR)
            im_scale_factors.append(im_scale)
            processed_ims.append(im)

        # Create a blob to hold the input images
        blob = im_list_to_blob(processed_ims)

        return blob, np.array(im_scale_factors)

    def Predict(self,im_in,area):
        # initilize the tensor holder here.
        im_data = torch.FloatTensor(1)
        im_info = torch.FloatTensor(1)
        num_boxes = torch.LongTensor(1)
        gt_boxes = torch.FloatTensor(1)

        # ship to cuda
        if self.cuda > 0:
            im_data = im_data.cuda()
            im_info = im_info.cuda()
            num_boxes = num_boxes.cuda()
            gt_boxes = gt_boxes.cuda()

        # make variable
        with torch.no_grad():
            im_data = Variable(im_data)
            im_info = Variable(im_info)
            num_boxes = Variable(num_boxes)
            gt_boxes = Variable(gt_boxes)

        if self.cuda > 0:
            cfg.CUDA = True

        if self.cuda > 0:
            self.fasterRCNN.cuda()

        self.fasterRCNN.eval()


        #im_in = cv2.imread(im_file)
        if len(im_in.shape) == 2:
            im_in = im_in[:, :, np.newaxis]
            im_in = np.concatenate((im_in, im_in, im_in), axis=2)
        # rgb -> bgr
        im_in = im_in[:, :, ::-1]
        im= cv2.cvtColor(im_in, cv2.COLOR_BGR2RGB)

        blobs, im_scales = self._get_image_blob(im)
        assert len(im_scales) == 1, "Only single-image batch implemented"
        im_blob = blobs
        im_info_np = np.array([[im_blob.shape[1], im_blob.shape[2], im_scales[0]]], dtype=np.float32)

        im_data_pt = torch.from_numpy(im_blob)
        im_data_pt = im_data_pt.permute(0, 3, 1, 2)
        im_info_pt = torch.from_numpy(im_info_np)

        im_data.data.resize_(im_data_pt.size()).copy_(im_data_pt)
        im_info.data.resize_(im_info_pt.size()).copy_(im_info_pt)
        gt_boxes.data.resize_(1, 1, 5).zero_()
        num_boxes.data.resize_(1).zero_()


        rois, cls_prob, bbox_pred, \
        rpn_loss_cls, rpn_loss_box, \
        RCNN_loss_cls, RCNN_loss_bbox, \
        rois_label = self.fasterRCNN(im_data, im_info, gt_boxes, num_boxes)

        scores = cls_prob.data
        boxes = rois.data[:, :, 1:5]

        if cfg.TEST.BBOX_REG:
            # Apply bounding-box regression deltas
            box_deltas = bbox_pred.data
            if cfg.TRAIN.BBOX_NORMALIZE_TARGETS_PRECOMPUTED:
                # Optionally normalize targets by a precomputed mean and stdev
                if self.class_agnostic:
                    if self.cuda > 0:
                        box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS).cuda() \
                                     + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS).cuda()
                    else:
                        box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS) \
                                     + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS)

                    box_deltas = box_deltas.view(1, -1, 4)
                else:
                    if self.cuda > 0:
                        box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS).cuda() \
                                     + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS).cuda()
                    else:
                        box_deltas = box_deltas.view(-1, 4) * torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_STDS) \
                                     + torch.FloatTensor(cfg.TRAIN.BBOX_NORMALIZE_MEANS)
                    box_deltas = box_deltas.view(1, -1, 4 * len(self.pascal_classes))

            pred_boxes = bbox_transform_inv(boxes, box_deltas, 1)
            pred_boxes = clip_boxes(pred_boxes, im_info.data, 1)
        else:
            # Simply repeat the boxes, once for each class
            _ = torch.from_numpy(np.tile(boxes, (1, scores.shape[1])))
            pred_boxes = _.cuda() if self.cuda > 0 else _

        pred_boxes /= im_scales[0]

        scores = scores.squeeze()
        pred_boxes = pred_boxes.squeeze()
        ItemAndBoxes_all = []
        im2show = np.copy(im)
        for j in xrange(1, len(self.pascal_classes)):
            inds = torch.nonzero(scores[:, j] > self.thresh).view(-1)
            # if there is det
            if inds.numel() > 0:
                cls_scores = scores[:, j][inds]
                _, order = torch.sort(cls_scores, 0, True)
                if self.class_agnostic:
                    cls_boxes = pred_boxes[inds, :]
                else:
                    cls_boxes = pred_boxes[inds][:, j * 4:(j + 1) * 4]

                cls_dets = torch.cat((cls_boxes, cls_scores.unsqueeze(1)), 1)
                # cls_dets = torch.cat((cls_boxes, cls_scores), 1)
                cls_dets = cls_dets[order]
                keep = nms(cls_dets, cfg.TEST.NMS, force_cpu=not cfg.USE_GPU_NMS)
                cls_dets = cls_dets[keep.view(-1).long()]
                im2show,ItemAndBoxes = vis_detections(im2show, self.pascal_classes[j], cls_dets.cpu().numpy(), self.visThresh)
                ItemAndBoxes_all.append(ItemAndBoxes)

        ItemAndBoxes_all  = sorted(ItemAndBoxes_all,key =lambda x:x[2],reverse=True)
        ItemAndBoxes_all = ItemAndBoxes_all[0:3]
        ItemAndBoxes_all = sorted(ItemAndBoxes_all, key=lambda x: x[1][0])

        if self.vis == 1:
            cv2.namedWindow("result", 0);
            cv2.resizeWindow("result", 1080, 720);
            cv2.imshow('result', im2show)
            cv2.waitKey(0)
            result_path = os.path.join(self.image_dir, str(area)+".jpg")
            cv2.imwrite(result_path, im2show)

        return {"Left": ItemAndBoxes_all[0][0], "Mid": ItemAndBoxes_all[1][0], "Right": ItemAndBoxes_all[2][0]}


if __name__ == '__main__':
    od = Object_Detection()
    od.Load_model()
    imglist = os.listdir(od.image_dir)
    for filename in imglist:
        image = cv2.imread('images/' + filename)
        print(od.Predict(image,'A'),filename)
