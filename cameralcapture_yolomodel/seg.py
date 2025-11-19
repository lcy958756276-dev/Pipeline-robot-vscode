#!/usr/bin/python3
import rospy
import cv2
import numpy as np
import onnxruntime as ort
import yaml
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from pipeline_robot.msg import detection_new, compressedRGBD
from geometry_msgs.msg import PointStamped

# 配置文件和模型路径1
yaml_path = '/home/ralab/pipeline-inspection-robot-topnx/src/pipeline_robot/scripts/yoloseg/sewer_seg.yaml'
model_path = '/home/ralab/pipeline-inspection-robot-topnx/src/pipeline_robot/scripts/yoloseg/best.onnx'

# 图像中心和目标点
image_center = None
target_coordinates = None

class YOLOv8Seg:
    """YOLOv8 ONNX Segmentation ROS 节点

    作用：
        使用 ONNX 模型进行图像分割和目标检测，并通过 ROS 发布检测结果和可视化图像。

    参数：
        onnx_model (str): ONNX 模型文件路径。

    返回：
        无
    """

    def __init__(self, onnx_model):
        """初始化 YOLOv8Seg 节点

        作用：
            初始化 ONNXRuntime，加载类别配置，初始化 ROS 节点和订阅/发布器。

        参数：
            onnx_model (str): ONNX 模型路径

        返回：
            无
        """
        # ONNXRuntime Session
        self.session = ort.InferenceSession(
            onnx_model,
            providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            if ort.get_device() == 'GPU' else ['CPUExecutionProvider']
        )
        self.ndtype = np.half if self.session.get_inputs()[0].type == 'tensor(float16)' else np.float32
        self.model_height, self.model_width = [x.shape for x in self.session.get_inputs()][0][-2:]

        # 类名加载
        with open(yaml_path, 'r') as f:
            cfg = yaml.safe_load(f)
        self.classes = cfg['names']

        # ROS 初始化
        rospy.init_node('yolo_segmentation', anonymous=True)
        self.image_pub = rospy.Publisher("/yolov8_detection", Image, queue_size=1)
        rospy.Subscriber('/camera/rgb_depth/compressed', compressedRGBD, self.image_callback, queue_size=1)
        self.detection_pub = rospy.Publisher("/_detection_info", detection_new, queue_size=1)

        self.bridge = CvBridge()
        self.count = 0
        rospy.loginfo("YOLOv8Seg ONNX Node initialized")

    def __call__(self, im0, conf_threshold=0.45, iou_threshold=0.45, nm=32):
        """执行模型推理

        作用：
            对输入图像进行预处理、推理和后处理，返回检测框、分割区域和掩码。

        参数：
            im0 (np.ndarray): 原始图像
            conf_threshold (float): 置信度阈值
            iou_threshold (float): NMS IoU 阈值
            nm (int): 掩码通道数量

        返回：
            boxes (np.ndarray): 检测框坐标和置信度
            segments (list): 分割轮廓点集
            masks (np.ndarray): 分割掩码
        """
        im, ratio, (pad_w, pad_h) = self.preprocess(im0)
        preds = self.session.run(None, {self.session.get_inputs()[0].name: im})
        boxes, segments, masks = self.postprocess(preds, im0, ratio, pad_w, pad_h, conf_threshold, iou_threshold, nm)
        return boxes, segments, masks

    def preprocess(self, img):
        """图像预处理

        作用：
            对图像进行缩放、填充和格式转换以适应 ONNX 模型输入。

        参数：
            img (np.ndarray): 原始图像

        返回：
            img (np.ndarray): 处理后的图像
            ratio (tuple): 缩放比例
            pad_w, pad_h (float): 填充宽度和高度
        """
        shape = img.shape[:2]
        new_shape = (self.model_height, self.model_width)
        r = min(new_shape[0]/shape[0], new_shape[1]/shape[1])
        ratio = r, r
        new_unpad = int(round(shape[1]*r)), int(round(shape[0]*r))
        pad_w, pad_h = (new_shape[1]-new_unpad[0])/2, (new_shape[0]-new_unpad[1])/2

        if shape[::-1] != new_unpad:
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        top, bottom = int(round(pad_h - 0.1)), int(round(pad_h + 0.1))
        left, right = int(round(pad_w - 0.1)), int(round(pad_w + 0.1))
        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114,114,114))

        img = np.ascontiguousarray(np.einsum('HWC->CHW', img)[::-1], dtype=self.ndtype)/255.0
        return img[None], ratio, (pad_w, pad_h)

    def postprocess(self, preds, im0, ratio, pad_w, pad_h, conf_threshold, iou_threshold, nm=32):
        """模型后处理

        作用：
            对模型输出进行 NMS、掩码处理和轮廓提取。

        参数：
            preds (list): 模型输出
            im0 (np.ndarray): 原始图像
            ratio (tuple): 缩放比例
            pad_w, pad_h (float): 填充宽度和高度
            conf_threshold (float): 置信度阈值
            iou_threshold (float): NMS 阈值
            nm (int): 掩码通道数量

        返回：
            boxes (np.ndarray): 检测框坐标和置信度
            segments (list): 分割轮廓
            masks (np.ndarray): 分割掩码
        """
        x, protos = preds[0], preds[1]
        x = np.einsum('bcn->bnc', x)
        x = x[np.amax(x[..., 4:-nm], axis=-1) > conf_threshold]
        if len(x) == 0:
            return [], [], []

        x = np.c_[x[..., :4], np.amax(x[..., 4:-nm], axis=-1), np.argmax(x[..., 4:-nm], axis=-1), x[..., -nm:]]
        indices = cv2.dnn.NMSBoxes(x[:, :4].tolist(), x[:, 4].tolist(), conf_threshold, iou_threshold)
        if len(indices) == 0:
            return [], [], []

        x = x[indices.flatten()]
        x[..., [0,1]] -= x[..., [2,3]]/2
        x[..., [2,3]] += x[..., [0,1]]
        x[..., :4] -= [pad_w, pad_h, pad_w, pad_h]
        x[..., :4] /= min(ratio)
        x[..., [0,2]] = x[:, [0,2]].clip(0, im0.shape[1])
        x[..., [1,3]] = x[:, [1,3]].clip(0, im0.shape[0])

        masks = self.process_mask(protos[0], x[:,6:], x[:,:4], im0.shape)
        segments = self.masks2segments(masks)
        return x[..., :6], segments, masks

    @staticmethod
    def masks2segments(masks):
        """掩码转换为轮廓

        作用：
            将二值掩码转换为轮廓点集

        参数：
            masks (np.ndarray): 分割掩码

        返回：
            segments (list): 每个掩码对应的轮廓点
        """
        segments = []
        for x in masks.astype('uint8'):
            c = cv2.findContours(x, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
            if c:
                c = np.array(c[np.array([len(xx) for xx in c]).argmax()]).reshape(-1,2)
            else:
                c = np.zeros((0,2))
            segments.append(c.astype('float32'))
        return segments

    @staticmethod
    def crop_mask(masks, boxes):
        """裁剪掩码到检测框区域

        作用：
            将掩码裁剪到对应的检测框区域

        参数：
            masks (np.ndarray): 掩码
            boxes (np.ndarray): 检测框

        返回：
            masks (np.ndarray): 裁剪后的掩码
        """
        n, h, w = masks.shape
        x1, y1, x2, y2 = np.split(boxes[:,:,None], 4, 1)
        r = np.arange(w, dtype=x1.dtype)[None,None,:]
        c = np.arange(h, dtype=x1.dtype)[None,:,None]
        return masks * ((r>=x1)*(r<x2)*(c>=y1)*(c<y2))

    def process_mask(self, protos, masks_in, bboxes, im0_shape):
        """处理掩码并缩放

        作用：
            根据原始图像尺寸缩放掩码，并裁剪到检测框区域

        参数：
            protos (np.ndarray): 原始掩码原型
            masks_in (np.ndarray): 模型输出掩码
            bboxes (np.ndarray): 检测框
            im0_shape (tuple): 原图形状

        返回：
            masks (np.ndarray): 处理后的掩码
        """
        c, mh, mw = protos.shape
        masks = np.matmul(masks_in, protos.reshape((c,-1))).reshape((-1,mh,mw)).transpose(1,2,0)
        masks = np.ascontiguousarray(masks)
        masks = self.scale_mask(masks, im0_shape)
        masks = np.einsum('HWN->NHW', masks)
        masks = self.crop_mask(masks, bboxes)
        return np.greater(masks, 0.5)

    @staticmethod
    def scale_mask(masks, im0_shape, ratio_pad=None):
        """掩码缩放到原图大小

        作用：
            将掩码缩放到输入图像大小

        参数：
            masks (np.ndarray): 原始掩码
            im0_shape (tuple): 原图尺寸
            ratio_pad (tuple): 缩放填充比例（可选）

        返回：
            masks (np.ndarray): 缩放后的掩码
        """
        im1_shape = masks.shape[:2]
        if ratio_pad is None:
            gain = min(im1_shape[0]/im0_shape[0], im1_shape[1]/im0_shape[1])
            pad = (im1_shape[1]-im0_shape[1]*gain)/2, (im1_shape[0]-im0_shape[0]*gain)/2
        else:
            pad = ratio_pad[1]
        top, left = int(round(pad[1]-0.1)), int(round(pad[0]-0.1))
        bottom, right = int(round(im1_shape[0]-pad[1]+0.1)), int(round(im1_shape[1]-pad[0]+0.1))
        masks = masks[top:bottom, left:right]
        masks = cv2.resize(masks, (im0_shape[1], im0_shape[0]), interpolation=cv2.INTER_LINEAR)
        if len(masks.shape)==2:
            masks = masks[:,:,None]
        return masks

    def draw_and_visualize(self, im, bboxes, segments):
        """绘制检测框、分割轮廓并可视化

        作用：
            在图像上绘制检测框、轮廓、多边形，并标注目标中心点。

        参数：
            im (np.ndarray): 原始图像
            bboxes (np.ndarray): 检测框
            segments (list): 分割轮廓

        返回：
            im (np.ndarray): 可视化后的图像
        """
        global image_center, target_coordinates
        im_canvas = im.copy()
        for i, segment in enumerate(segments):
            box_conf_cls = bboxes[i]
            box = box_conf_cls[:4]
            conf = box_conf_cls[4]
            cls_ = int(box_conf_cls[5])

            cv2.polylines(im, np.int32([segment]), True, (255,255,255), 2)
            cv2.fillPoly(im_canvas, np.int32([segment]), (0,255,0))

            # 画框和文字
            cv2.rectangle(im, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0,255,0), 1)
            cv2.putText(im, f'{self.classes[cls_]}: {conf:.3f}', (int(box[0]), int(box[1]-9)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            if image_center is None:
                height, width, _ = im.shape
                image_center = (width//2, height//2)

            center_x = int((box[0]+box[2])/2)
            center_y = int((box[1]+box[3])/2)
            target_coordinates = (center_x, center_y)
            for point in segment:
                if point[0]>=center_x and point[1]>=center_y:
                    target_coordinates = (int(point[0]), int(point[1]))
                    break

            cv2.circle(im, target_coordinates, 5, (255,0,0), -1)
            cv2.putText(im, f"Target: {target_coordinates}", (target_coordinates[0]+10, target_coordinates[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0),2)
            if image_center is not None:
                cv2.circle(im, image_center, 5, (0,0,255), -1)
        im = cv2.addWeighted(im_canvas, 0.3, im, 0.7, 0)
        return im

    def publish_image(self, cv_image):
        """发布 ROS 图像消息

        作用：
            将 OpenCV 图像发布到 ROS 话题

        参数：
            cv_image (np.ndarray): 要发布的图像

        返回：
            无
        """
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def image_callback(self, msg):
        """ROS 图像订阅回调函数

        作用：
            接收摄像头图像消息，解码后进行 YOLOv8 检测，绘制可视化结果，并发布检测结果和图像。

        参数：
            msg (compressedRGBD): 摄像头压缩 RGBD 消息

        返回：
            无
        """
        start_time = time.time()  # 接收到消息时间

        self.count += 1
        if self.count <= 2:  # 减少帧数，降低延迟
            return
        else:
            self.count = 0

        # 解码压缩图像
        try:
            color_encoded = np.frombuffer(msg.rgb.data, np.uint8)
            color_frame = cv2.imdecode(color_encoded, cv2.IMREAD_COLOR)
        except Exception as e:
            rospy.logerr(f"error in decompressing images: {e}")
            return
        decode_time = time.time()

        # YOLOv8 处理
        boxes, segments, _ = self(color_frame)
        process_time = time.time()

        # 绘制和发布
        if len(boxes) > 0:
            rgb = self.draw_and_visualize(color_frame, boxes, segments)
            self.publish_image(rgb)
            for (*box, conf, cls_), segment in zip(boxes, segments):
                det_msg = detection_new()
                det_msg.class_name = self.classes[cls_]
                det_msg.score = conf

                det_msg.point_stamped = PointStamped()
                det_msg.point_stamped.header.frame_id = "camera_link"
                det_msg.point_stamped.header.stamp = rospy.Time.now()
                det_msg.size="0.1"

                try:
                    det_msg.image_data = self.bridge.cv2_to_imgmsg(rgb, "bgr8")
                except CvBridgeError as e:
                    rospy.logerr("CvBridge Error: {0}".format(e))

                self.detection_pub.publish(det_msg)
        else:
            self.publish_image(color_frame)
        end_time = time.time()


if __name__ == '__main__':
    model = YOLOv8Seg(model_path)
    rospy.spin()
