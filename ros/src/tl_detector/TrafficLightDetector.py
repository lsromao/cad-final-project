from styx_msgs.msg import TrafficLight
import cv2
import rospy
import tensorflow as tf

class TrafficLightDetector():
    def __init__(self):
        rospy.logwarn('Loading the model...')

        self.detect_fn = tf.saved_model.load('tl_model/')

        rospy.logwarn('Model loaded.')


    def detect_state(self, camera_frame):

        image = cv2.imread(camera_frame)
        image = cv2.resize(image, (512,512))
        image = image / 255.0

        input_tensor = tf.convert_to_tensor(image)
        input_tensor = input_tensor[tf.newaxis, ...]

        detections = detect_fn(input_tensor)

        num_detections = int(detections.pop('num_detections'))
        detections = {key: value[0, :num_detections].numpy()
                    for key, value in detections.items()}
        detections['num_detections'] = num_detections

        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

        boxes = detections['detection_boxes']
        classes = detections['detection_classes']
        scores = detections['detection_scores']

        for box, score, class_label in zip(boxes, scores, classes):
            if score > 0.5:
                class_label = int(class_label)
                if class_label == 1:
                    return TrafficLight.GREEN
                elif class_label == 2:
                    return TrafficLight.RED
                elif class_label == 3:
                    return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
