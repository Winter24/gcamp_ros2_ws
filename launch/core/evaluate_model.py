from core.datasets.DATASET import *
from core.models.backbones.PIXORMobileNet import *
from shapely.geometry import Polygon
from kitti_config import *
from scipy.linalg import block_diag
from tqdm import tqdm
from core.datasets.utils_1.kitti_utils import compute_box_3d


#######################
# reshape predictions #
#######################

def process_regression_target(valid_reg_predictions, validity_mask):
    """
        Convert the raw regression prediction into camera coordinates of a bounding box.
        :param valid_reg_predictions: raw regression output | shape: [N_VALID, OUTPUT_DIM_REG]
        :param validity_mask: mask of predictions indices with confidence score higher than threshold. Required for the
                              calculation of the camera coordinates of the predicted bounding box | shape: [OUTPUT_DIM_0 * OUTPUT_DIM_1, ]
        :return: point_cloud_box_predictions: predicted box coordinates | shape: [N_VALID, 8]
        """

    # get camera coordinates of the pixel corner for all valid predictions
    x_lin = np.linspace(VOX_Y_MIN, VOX_Y_MAX - 0.4, OUTPUT_DIM_1) # W
    y_lin = np.linspace(VOX_X_MAX, VOX_X_MIN + 0.4, OUTPUT_DIM_0) # H
    px_x, px_y = np.meshgrid(x_lin, y_lin)
    px_x = px_x.reshape((-1,))[validity_mask]
    px_y = px_y.reshape((-1,))[validity_mask]
    
    # get the camera coordinates of the center of the predicted bounding boxes
    center_x = np.broadcast_to((px_x - valid_reg_predictions[:, 2]).reshape(-1, 1), (valid_reg_predictions.shape[0], 4))# [a, b,c,...]->[[a], [b], [c], ...]
    center_y = np.broadcast_to((px_y - valid_reg_predictions[:, 3]).reshape(-1, 1), (valid_reg_predictions.shape[0], 4))
    centers = np.hstack((center_x, center_y)) # hang gan voi hang 
    
    # get the predicted angles for all bounding boxes
    prediction_angles = np.arctan2(valid_reg_predictions[:, 1], valid_reg_predictions[:, 0]) # LAy goc theta dai dien chinh xac ma model tra ve thong qua cos_pred va sin_pred
    prediction_cos = np.cos(prediction_angles)
    prediction_sin = np.sin(prediction_angles)

    # build a block diagonal rotation matrix to rotate all predicted bounding boxes
    rot_matrices = np.stack((prediction_cos, prediction_sin, -prediction_sin, prediction_cos), axis=1).reshape((-1, 2))
    rot_matrices = [rot_matrices[i:i + 2, :] for i in range(0, rot_matrices.shape[0], 2)]
    block_rot_matrix = block_diag(*rot_matrices) # * :cac phan tu trong list se la 1 tham so rieng le cho HAM( tung cai mot)

    # get predicted width and length for all bounding boxes
    width = np.exp(valid_reg_predictions[:, 4])
    length = np.exp(valid_reg_predictions[:, 5])
    
    # get all bounding box corners
    x_corners = np.stack((length / 2, length / 2, -length / 2, -length / 2), axis=1)
    y_corners = np.stack((-width / 2, width / 2, width / 2, -width / 2), axis=1)
    corners = np.stack((x_corners, y_corners), axis=1).reshape((-1, 4))

    # rotate the bounding boxes
    corners = np.dot(block_rot_matrix, corners).reshape(-1, 8)
    # translate the bounding boxes
    point_cloud_box_predictions = corners + centers

    return point_cloud_box_predictions


##########################
# non-maximum suppression # this def maybe work 
##########################

def perform_nms(valid_class_predictions, valid_box_predictions, nms_threshold):
    """
        Perform Non-Maximum Suppression to eliminate overlapping predictions.
        :param valid_class_predictions: all confidence scores higher than the threshold | shape: [N_VALID, ]
        :param valid_box_predictions: all corresponding bounding box predictions | shape: [N_VALID, 8]
        :param nms_threshold: threshold for maximum overlap between two bounding boxes
        :return: sorted_class_predictions: remaining confidence scores | shape: [N_FINAL, ]
                 sorted_box_predictions: remaining box predictions | shape: [N_FINAL, 8]
        """
    
    # sort the detections such that the entry with the maximum confidence score is at the top
    sorted_indices = np.argsort(valid_class_predictions)[::-1] # [start: stop: step] , voi step = -1 -> dao nguoc mang
    sorted_box_predictions = valid_box_predictions[sorted_indices]
    sorted_class_predictions = valid_class_predictions[sorted_indices]

    for i in range(sorted_box_predictions.shape[0]):
        # get the IOUs of all boxes with the currently most certain bounding box
        try:
            ious = np.zeros((sorted_box_predictions.shape[0]))
            ious[i + 1:] = bbox_iou(sorted_box_predictions[i, :], sorted_box_predictions[i + 1:, :])
        except ValueError:
            break
        except IndexError:
            break

        # eliminate all detections which have IoU > threshold
        overlap_mask = np.where(ious < nms_threshold, True, False)
        sorted_box_predictions = sorted_box_predictions[overlap_mask]
        sorted_class_predictions = sorted_class_predictions[overlap_mask]

    return sorted_class_predictions, sorted_box_predictions


####################
# bounding box IoU #
####################

def bbox_iou(box1, boxes):
    """
    Compute the bounding box IoUs between the given bounding box "box1" and a number of bounding boxes in "boxes"
    :param box1: given bounding box | shape: [8, ]
    :param boxes: bounding boxes for which IoU with box1 is to be computed | shape: [N, 8]
    :return: IoUs for each box in "boxes" with "box1" | shape: [N, ]
    """
    
    # currently inspected box
    box1 = box1.reshape((2, 4)).T
    rect_1 = Polygon([(box1[0, 0], box1[0, 1]), 
                      (box1[1, 0], box1[1, 1]), 
                      (box1[2, 0], box1[2, 1]),
                      (box1[3, 0], box1[3, 1])])
    area_1 = rect_1.area
    
    # IoU of box1 with each of the boxes in "boxes"
    ious = np.zeros(boxes.shape[0])
    for box_id in range(boxes.shape[0]):
        box2 = boxes[box_id]
        box2 = box2.reshape((2, 4)).T
        rect_2 = Polygon([(box2[0, 0], box2[0, 1]), 
                          (box2[1, 0], box2[1, 1]), 
                          (box2[2, 0], box2[2, 1]),
                          (box2[3, 0], box2[3, 1])])
        area_2 = rect_2.area

        # get intersection of both bounding boxes
        inter_area = rect_1.intersection(rect_2).area

        # compute IoU of the two bounding boxes
        iou = inter_area / (area_1 + area_2 - inter_area)
        ious[box_id] = iou
    return ious


#######################
# process predictions #
#######################

def process_predictions(batch_predictions, confidence_threshold=0.2, nms_threshold=0.05):

    batch_predictions[:, :, :, :-1] = (batch_predictions[:, :, :, :-1] * REG_STD) + REG_MEAN

    final_batch_predictions = None 
    for point_cloud_id in range(batch_predictions.shape[0]):
        
        point_cloud_predictions = batch_predictions[point_cloud_id]
        print('Raw predictions shape:', point_cloud_predictions.shape) 
        exit()
        point_cloud_predictions = point_cloud_predictions.reshape(
            (OUTPUT_DIM_0 * OUTPUT_DIM_1, OUTPUT_DIM_CLA + OUTPUT_DIM_REG))
    
        point_cloud_class_predictions = point_cloud_predictions[:, -1]
        point_cloud_reg_predictions = point_cloud_predictions[:, :-1]

        validity_mask = np.where(point_cloud_class_predictions > confidence_threshold, True, False)
        valid_reg_predictions = point_cloud_reg_predictions[validity_mask]
        valid_class_predictions = point_cloud_class_predictions[validity_mask]

        if valid_reg_predictions.shape[0]:
            valid_box_predictions = process_regression_target(valid_reg_predictions, validity_mask)
        else:
            continue
        # print(f"Point Cloud {point_cloud_id}: {valid_box_predictions.shape[0]} boxes before NMS")
        final_class_predictions, final_box_predictions = perform_nms(valid_class_predictions, valid_box_predictions,
                                                                     nms_threshold)

        # print(f"Point Cloud {point_cloud_id}: {final_box_predictions.shape[0]} boxes after NMS")
        point_cloud_ids = np.ones((final_box_predictions.shape[0], 1)) * point_cloud_id
        final_point_cloud_predictions = np.hstack((point_cloud_ids, 
                                                   final_class_predictions[:, np.newaxis],
                                                   final_box_predictions))
        # format of invalid bbox : [point_cloud_id, confidence_score, x1, y1, x2, y2, x3, y3, x4, y4]
        
        if final_batch_predictions is None:
            final_batch_predictions = final_point_cloud_predictions
        else:
            final_batch_predictions = np.vstack((final_batch_predictions, final_point_cloud_predictions))
    # print('Final :{} \n shape: {}'.format(final_batch_predictions, final_batch_predictions.shape))
    
    return final_batch_predictions





import matplotlib.pyplot as plt

def evaluate_model(model, data_loader, distance_ranges, iou_thresholds):
  
    model.eval()  
    
    eval_dict = {distance_range: {'targets': {threshold: [] for threshold in iou_thresholds}, 'scores': [], 'n_labels': 0}
                 for distance_range in distance_ranges}

    for batch_id, (batch_data, batch_labels, batch_calib) in tqdm(enumerate(data_loader),
                                                                  total = len(data_loader),
                                                                  desc='Evaluating'):
        batch_data = batch_data.to(device)

        with torch.set_grad_enabled(False):
            batch_predictions = model(batch_data)
            batch_predictions = np.transpose(batch_predictions.detach().cpu().numpy(), (0, 2, 3, 1))
            final_box_predictions = process_predictions(batch_predictions)

            for point_cloud_id in range(batch_data.size(0)):
                if final_box_predictions is not None:
                    point_cloud_predictions = np.vstack(
                        [predictions for predictions in final_box_predictions if predictions[0] == point_cloud_id])
                else:
                    point_cloud_predictions = None

                # Debugging: Display number of predictions and labels
                # print(f"Batch ID {batch_id} -- Point Cloud {point_cloud_id}: Predictions: {point_cloud_predictions.shape[0] if point_cloud_predictions is not None else 0}")


                # Get ground truth box corners
                ground_truth_box_corners = None
                for label in batch_labels[point_cloud_id]:
                    if label.type == 'Car':
                        _, bbox_corners_camera_coord = compute_box_3d(label, batch_calib[point_cloud_id].P) # return _, (8,3)
                        bbox_corners_camera_coord = np.hstack((bbox_corners_camera_coord[:4, 0], bbox_corners_camera_coord[:4, 2])) # 4 x roi toi 4 z
                        if ground_truth_box_corners is None:  
                            ground_truth_box_corners = bbox_corners_camera_coord
                        else:
                            ground_truth_box_corners = np.vstack((ground_truth_box_corners, bbox_corners_camera_coord))
                
                # Debugging: Check ground truth box corner dimensions
                # if ground_truth_box_corners is not None:
                #     print(f"Ground Truth Boxes Shape: {ground_truth_box_corners.shape}") # (n,8)
                
                # Ensure 2D structure for ground truth box corners
                if ground_truth_box_corners is not None:
                    ground_truth_box_corners = np.atleast_2d(ground_truth_box_corners)
                    if ground_truth_box_corners.ndim != 2 or ground_truth_box_corners.shape[1] < 8:
                        print(f"Skipping point cloud {point_cloud_id} due to invalid ground truth box shape: {ground_truth_box_corners.shape}")
                        continue

                # Evaluate for each distance range
                for distance_range in distance_ranges:
                    if point_cloud_predictions is not None and ground_truth_box_corners is not None:
        
                        max_distance_predictions = np.max(point_cloud_predictions[:, 6:], axis=1)
                        max_distance_ground_truth = np.max(ground_truth_box_corners[:, 4:], axis=1) # moi hang la 1 bbox, lay 4 cot cuoi va tim max
                    
                        distance_mask_predictions = np.where(max_distance_predictions <= distance_range, True, False)
                        distance_mask_ground_truth = np.where(max_distance_ground_truth <= distance_range, True, False)

                        point_cloud_predictions = point_cloud_predictions[distance_mask_predictions]
                        ground_truth_box_corners = ground_truth_box_corners[distance_mask_ground_truth]

                        # Debugging: Display filtered prediction and ground truth counts
                        # print(f"Distance {distance_range}m:\n \
                        #         Filtered Predictions: {point_cloud_predictions.shape[0]}\n \
                        #         Filtered GT: {ground_truth_box_corners.shape[0]}")
                        
                        if point_cloud_predictions.shape[0] and ground_truth_box_corners.shape[0]:
                            ious              = np.zeros((point_cloud_predictions.shape[0], ground_truth_box_corners.shape[0]))
                            confidence_scores = np.zeros((point_cloud_predictions.shape[0], ground_truth_box_corners.shape[0]))
                            for pid, prediction in enumerate(point_cloud_predictions):
                                ious[pid, :] = bbox_iou(prediction[2:], ground_truth_box_corners)
                                confidence_scores[pid, np.argmax(ious[pid, :])] = prediction[1]

                            for threshold in iou_thresholds:
                                eval_dict[distance_range]['targets'][threshold].extend(
                                    np.where(np.max(ious, axis=1) > threshold, 1, 0) # tung hang trong iou va lay max
                                )

                            eval_dict[distance_range]['scores'].extend(np.max(confidence_scores, axis=1))
                            eval_dict[distance_range]['n_labels'] += ground_truth_box_corners.shape[0]
                        elif point_cloud_predictions.shape[0] and not ground_truth_box_corners.shape[0]:
                            for threshold in iou_thresholds:
                                eval_dict[distance_range]['targets'][threshold].extend(np.zeros((point_cloud_predictions.shape[0],)))
                            eval_dict[distance_range]['scores'].extend(point_cloud_predictions[:, 1])
                        elif not point_cloud_predictions.shape[0] and ground_truth_box_corners.shape[0]:
                            eval_dict[distance_range]['n_labels'] += ground_truth_box_corners.shape[0]
    
    # Compute performance metrics
    for distance_range in distance_ranges:
        sorted_indices = np.argsort(eval_dict[distance_range]['scores'])[::-1]
        eval_dict[distance_range]['scores'] = np.array(eval_dict[distance_range]['scores'])[sorted_indices]

        for threshold in iou_thresholds:
            eval_dict[distance_range]['targets'][threshold] = np.array(eval_dict[distance_range]['targets'][threshold])[sorted_indices]
            
            # print(f"Distance {distance_range}m IoU {threshold}: Sorted Targets = {eval_dict[distance_range]['targets'][threshold]}")
            # print(f"Distance {distance_range}m IoU {threshold}: Sorted Scores = {eval_dict[distance_range]['scores']}")
            # print(f"Distance {distance_range}m: Number of Labels = {eval_dict[distance_range]['n_labels']}")

            performance_dict = {}
            recall = list(np.cumsum(eval_dict[distance_range]['targets'][threshold]) / eval_dict[distance_range]['n_labels'])
            recall.insert(0, 0.) # vi tri thu 0 , append phan tu 0.
            recall = np.array(recall)
            performance_dict['recall'] = recall

            precision = [np.sum(eval_dict[distance_range]['targets'][threshold][:i + 1]) / (i + 1) for i in range(len(eval_dict[distance_range]['targets'][threshold]))]
            precision.insert(0, 0.) 
            precision = np.array(precision)
            performance_dict['precision'] = precision

            indices = np.where(recall[:-1] != recall[1:])[0] + 1
            average_precision = np.sum((recall[indices] - recall[indices - 1]) * precision[indices])
            performance_dict['AP'] = average_precision
            eval_dict[distance_range][threshold] = performance_dict
        
        average_precisions = [eval_dict[distance_range][key]['AP'] for key in eval_dict[distance_range] if not isinstance(key, str)]
        eval_dict[distance_range]['mAP'] = sum(average_precisions) / len(average_precisions)


    # # Debug IoU values
    # print(f"Point Cloud {point_cloud_id}: IoUs = {ious}")

    # # Debug confidence scores
    # print(f"Point Cloud {point_cloud_id}: Confidence Scores = {confidence_scores}")

    for distance_range in distance_ranges:
        print(f"Distance Range {distance_range}m: mAP = {eval_dict[distance_range]['mAP']:.4f}")

    return eval_dict

def param_model(model):
    name_model = model.__class__.__name__
    print('===================================================')
    print(f'Model: {name_model}')
    print(f'Total number of parameters: {sum(p.numel() for p in model.parameters())}')
    print(f'Total number of trainable parameters: {sum(p.numel() for p in model.parameters() if p.requires_grad)}')
    print('===================================================') 
    return name_model

def create_dir(verison):
    if not os.path.exists(f'Models/{verison}') :
        os.makedirs(f'Models/{verison}', exist_ok=True)
        print(f'Created directory: Models/{verison}')
    if not os.path.exists(f'Metrics/{verison}') :
        os.makedirs(f'Metrics/{verison}', exist_ok=True)
        print(f'Created directory: Metrics/{verison}')
    if not os.path.exists(f'RESULTS/{verison}') :
        os.makedirs(f'RESULTS/{verison}', exist_ok=True)
        print(f'Created directory: RESULTS/{verison}')

def get_backbone_name(path):
    dirs = path.split('/')
    idx_base = dirs.index('Models')
    dir_model = dirs[idx_base + 1]
    backbone_name = dir_model.split('_')[0]
    epoch_n = dirs[-1].split('_')[0]
    return dir_model, backbone_name, epoch_n



if __name__ == '__main__':

    device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
    print(f"The device is located on: {device}")
   
    batch_size = 1
    model_p = './Models/PIXORMobileNet_lr001_4-36_400_400-voxel_noVKI/13_epoch.pt'
    root_dir = './Data_7'
    data_loader = load_dataset(root=root_dir, batch_size=batch_size, device=device, test_set=True, used_vki= False, used_voxel= False)
    dir_model, _, epoch = get_backbone_name(model_p)
    # create_dir(dir_model)

    pixor = PIXORMobileNet(input_channels=((VOX_Z_MAX - VOX_Z_MIN) // VOX_Z_DIVISION) + 1)
    _ = param_model(pixor)
    
    pixor.load_state_dict(torch.load(model_p, map_location=device))
    pixor = pixor.to(device)

    eval_dict = evaluate_model(pixor, data_loader, distance_ranges=[70, 50, 30], iou_thresholds=[0.5, 0.6, 0.7, 0.8, 0.9])

    eval_dict['epoch'] = epoch
    np.savez(f'Evals/{dir_model}/eval_dict_epoch_{epoch}.npz', eval_dict=eval_dict)







### Results on test set PIXORMobileNet_lr0005_16_320_640
'''
Distance Range 70m: mAP = 0.3388
Distance Range 50m: mAP = 0.3754
Distance Range 30m: mAP = 0.4926
'''

### Results on test set PIXORMobileNet_lr0005_16_700_800
'''
Distance Range 70m: mAP = 0.3742
Distance Range 50m: mAP = 0.4011
Distance Range 30m: mAP = 0.4715
'''

### Results on test set PIXORMobileNet_lr001_16_700_800
'''
Distance Range 70m: mAP = 0.3513
Distance Range 50m: mAP = 0.3746
Distance Range 30m: mAP = 0.4430
'''

### Results on test set PIXORMobileNet_lr0.001_36_700_800_7_9
'''
Distance Range 70m: mAP = 0.5034
Distance Range 50m: mAP = 0.5278
Distance Range 30m: mAP = 0.5712
'''

### Results on test set PIXORMobileNet_lr0005_36_700_800
'''
Distance Range 70m: mAP = 0.5422
Distance Range 50m: mAP = 0.5673
Distance Range 30m: mAP = 0.6101
'''

