# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

""" Demo of using VoteNet 3D object detector to detect objects from a point cloud.

"""

import os
import sys
import numpy as np
import argparse
import importlib
import time

import torch
import torch.optim as optim

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = BASE_DIR
#NUM_OF_POINTS = 20000
NUM_OF_POINTS = 40000
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'models'))
from pc_util import random_sampling, read_ply, write_ply
from ap_helper import parse_predictions, flip_axis_to_depth
from sunrgbd_detection_dataset import DC  # dataset config
import sunrgbd_utils

def preprocess_point_cloud(point_cloud):
    ''' Prepare the numpy point cloud (N,3) for forward pass '''
    point_cloud = point_cloud[:, 0:3]  # do not use color for now
    floor_height = np.percentile(point_cloud[:, 2], 0.99)  # S: estimate the floor height 1% of all points height
    height = point_cloud[:, 2] - floor_height  # S: height of current Z point from floor
    point_cloud = np.concatenate([point_cloud, np.expand_dims(height, 1)], 1)  # (N,4) or (N,7) # S: XYZ+ height or XYZ + RGB+ Height
    point_cloud = random_sampling(point_cloud, NUM_OF_POINTS)  # S : take out random sample points 20K or 50K ( Generally) (20000,4) (XYZ + Height)
    pc = np.expand_dims(point_cloud.astype(np.float32), 0)  # (1,40000,4)    # S : Need this for forward pass to point net (1,40000(points),4)
    return pc


# Set file paths and dataset config
demo_dir = os.path.join(BASE_DIR, 'votenet_model')
sys.path.append(os.path.join(ROOT_DIR, 'sunrgbd'))

#checkpoint_path = os.path.join(os.path.join(demo_dir,"trained_model"), 'pretrained_votenet_on_sunrgbd.tar')
#checkpoint_path = os.path.join(os.path.join(demo_dir, "trained_model"), 'checkpoint.tar')
checkpoint_path = os.path.join(demo_dir, 'checkpoint_40K.tar')
#pc_path = os.path.join(os.path.join(demo_dir,"input_data"), 'sample4.ply')

# configuration parameters for votenet
eval_config_dict = {'remove_empty_box': True, 'use_3d_nms': True, 'nms_iou': 0.25,
                    'use_old_type_nms': False, 'cls_nms': True, 'per_class_proposal': False,           # S: TODO: change cls_nms to True
                    'conf_thresh': 0.5, 'dataset_config': DC}

# Init the model and optimzier
MODEL = importlib.import_module('votenet')  # import network module
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
net = MODEL.VoteNet(num_proposal=256, input_feature_dim=1, vote_factor=1,
                    sampling='vote_fps', num_class=DC.num_class,                #S: changed from seed_fps to vote_fps for my trained model
                    num_heading_bin=DC.num_heading_bin,
                    num_size_cluster=DC.num_size_cluster,
                    mean_size_arr=DC.mean_size_arr).to(device)
print('Constructed model.')
# Load checkpoint
optimizer = optim.Adam(net.parameters(), lr=0.001)
checkpoint = torch.load(checkpoint_path)
net.load_state_dict(checkpoint['model_state_dict'])
optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
epoch = checkpoint['epoch']
print("Loaded checkpoint %s (epoch: %d)" % (checkpoint_path, epoch))
net.eval()  # set model to eval mode (for bn and dp)



def run_votenet(point_cloud):
    # Load and preprocess input point cloud
    #point_cloud = read_ply(pc_path)
    #point_cloud = sio.loadmat("/home/vcr/UBC/Research/votenet/sunrgbd/sunrgbd_trainval/depth/000074.mat")['instance']
    #point_cloud = read_ply("/home/vcr/Desktop/test3.ply")

    pc = preprocess_point_cloud(point_cloud)
    print('Loaded point cloud data: ',pc.shape)

    # Model inference
    inputs = {'point_clouds': torch.from_numpy(pc).to(device)}
    tic = time.time()
    with torch.no_grad():
        end_points = net(inputs)
    toc = time.time()
    print('Inference time: %f' % (toc - tic))
    end_points['point_clouds'] = inputs['point_clouds']
    pred_map_cls = parse_predictions(end_points, eval_config_dict)
    print('Finished detection. %d object detected.' % (len(pred_map_cls[0])))

    """
    #S : Here pred_map_cls is a list of list. [[0],[1],[2],[3]....[BS-1]].
    Each List ie [1] is [(pred_sem_cls, box_params, box_score)_j]
    Note In this demo BS = 1 
     Returns:
        batch_pred_map_cls: a list of len == batch size (BS)
            [pred_list_i], i = 0, 1, ..., BS-1
            where pred_list_i = [(pred_sem_cls, box_params, box_score)_j]
            where j = 0, ..., num of valid detections - 1 from sample input i
    """
    '''
    dump_dir = os.path.join(demo_dir, 'results')
    if not os.path.exists(dump_dir): os.mkdir(dump_dir)
    MODEL.dump_results(end_points, dump_dir, DC, True)
    print('Dumped detection results to folder %s' % (dump_dir))
    """
    # S : Extraction code starts here...
    """
    '''
    print('-' * 30)
    '''
    Returns : [(class_name, objectness score, bbox_params in world, cropped_pcd)_i]
    '''
    all_detections = []
    input_pc = end_points['point_clouds'].cpu().numpy()[0,:,:]  # S : [0] since only one pcd in the batch (N,3)
    for i in range(len(pred_map_cls[0])):
        pred_list = pred_map_cls[0][i]
        semantic_class = DC.class2type[pred_list[0]]
        box3d_pts_3d = pred_list[1]
        box3d_pts_3d = flip_axis_to_depth(box3d_pts_3d)   # the 3d points are in the camera frame so need to convert it to world frame
        obj_score = pred_list[2]
        print("Semantic Class: " , semantic_class, " Object Score : ", obj_score)
        pc_in_box3d, inds = sunrgbd_utils.extract_pc_in_box3d(input_pc, box3d_pts_3d)
        detection = [semantic_class,obj_score,box3d_pts_3d,pc_in_box3d]
        all_detections.append(detection)                 # return all detections from here filter in ROS code
    #     if semantic_class == 'table':
    #         detection = [semantic_class,obj_score,box3d_pts_3d,pc_in_box3d]
    #         all_detections.append(detection)
    print('-' * 30)
    return all_detections

        #write_ply(pc_in_box3d, os.path.join(dump_dir, '%s_%f.ply' % (semantic_class,obj_score)))






