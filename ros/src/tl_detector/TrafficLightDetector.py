from styx_msgs.msg import TrafficLight
import cv2

class TrafficLightDetector():
    def __init__(self):
        self.model_loaded = False

    def detect_state(self, camera_frame):
        if not self.model_loaded:
            rospy.logwarn("Loading the model...")
            self.model = load_model('tl_model/saved_model.pb')
            self.model._make_predict_function()
            self.model_loaded = True
            #return TrafficLight.UNKNOWN

        image = cv2.resize(camera_frame,(512,512))
        
        image_array = img_preprocessing.img_to_array(image)
        image_array = np.expand_dims(image_array, axis=0).astype('float32')/255

        signal_prediction = np.argmax(self.model.predict(image_array))
        return TrafficLight.UNKNOWN
