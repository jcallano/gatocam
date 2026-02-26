#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
import cv2
import numpy as np
from rknnlite.api import RKNNLite

class JetAutoYoloRKNN(Node):
    def __init__(self):
        super().__init__('jetauto_yolo_rknn')
        
        # ROS 2 Interfaces
        self.bridge = CvBridge()
        self.sub_image = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.pub_detections = self.create_publisher(Detection2DArray, '/yolo/detections', 10)
        
        # RKNN NPU Configuration
        self.model_path = '/home/jcallano/ros2_ws/src/jetauto_description/scripts/yolov8n.rknn'
        self.rknn_lite = RKNNLite()
        
        self.get_logger().info('Cargando Modelo RKNN YOLOv8 en la NPU...')
        ret = self.rknn_lite.load_rknn(self.model_path)
        if ret != 0:
            self.get_logger().error('Fallo al cargar el modelo RKNN. Verifique la ruta.')
            return
            
        ret = self.rknn_lite.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().error('Fallo al inicializar el NPU Runtime. ¿Drivers librknn instalados?')
            return
            
        self.get_logger().info('✅ NPU Lista. Esperando imágenes de la cámara Astra...')

        # YOLOv8 target shape
        self.img_size = 640
        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
                   'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
                   'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
                   'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
                   'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
                   'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
                   'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
                   'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
                   'hair drier', 'toothbrush']

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV CV2 Format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
            
        # Preprocess: Resize to 640x640 and convert BGR -> RGB for YOLO
        img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img_rgb, (self.img_size, self.img_size))
        
        # Inference on NPU
        # YOLOv8 expects list of inputs, outputs a list of tensors
        outputs = self.rknn_lite.inference(inputs=[np.expand_dims(img_resized, 0)])
        
        # Postprocess YOLOv8 Tensors into Bounding Boxes
        detections_msg = self.post_process(outputs[0], cv_image.shape, msg.header)
        
        if detections_msg.detections:
            self.pub_detections.publish(detections_msg)

    def post_process(self, output_tensor, original_shape, header):
        # YOLOv8 output shape is usually (1, 84, 8400)
        # 84 = 4 bbox coords (cx, cy, w, h) + 80 class probabilities
        # 8400 = total number of anchor boxes proposed
        
        predictions = np.squeeze(output_tensor).T # Reshape to (8400, 84)
        
        boxes = []
        scores = []
        class_ids = []
        
        # Confidence threshold
        conf_thresh = 0.50
        
        orig_h, orig_w = original_shape[:2]
        x_factor = orig_w / self.img_size
        y_factor = orig_h / self.img_size

        for pred in predictions:
            classes_scores = pred[4:]
            class_id = np.argmax(classes_scores)
            score = classes_scores[class_id]
            
            if score > conf_thresh:
                cx, cy, w, h = pred[0], pred[1], pred[2], pred[3]
                
                # Scale back to original image resolution
                cx_orig = int(cx * x_factor)
                cy_orig = int(cy * y_factor)
                w_orig = int(w * x_factor)
                h_orig = int(h * y_factor)
                
                boxes.append([cx_orig, cy_orig, w_orig, h_orig])
                scores.append(float(score))
                class_ids.append(class_id)

        # Apply Non-Maximum Suppression (NMS) to remove duplicates
        indices = cv2.dnn.NMSBoxes(
            bboxes=[[b[0]-b[2]/2, b[1]-b[3]/2, b[2], b[3]] for b in boxes], 
            scores=scores, 
            score_threshold=conf_thresh, 
            nms_threshold=0.45
        )
        
        det_msg = Detection2DArray()
        det_msg.header = header
        
        if len(indices) > 0:
            for i in indices.flatten():
                det = Detection2D()
                det.header = header
                
                box = boxes[i]
                det.bbox.center.position.x = float(box[0])
                det.bbox.center.position.y = float(box[1])
                det.bbox.size_x = float(box[2])
                det.bbox.size_y = float(box[3])
                
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(self.classes[class_ids[i]])
                hyp.hypothesis.score = scores[i]
                
                det.results.append(hyp)
                det_msg.detections.append(det)
                
        return det_msg

def main(args=None):
    rclpy.init(args=args)
    node = JetAutoYoloRKNN()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.rknn_lite.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
