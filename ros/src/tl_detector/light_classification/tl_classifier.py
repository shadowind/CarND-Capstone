from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        is_simulation = True
        if is_simulation:
            file_name = "light_classification/frozen_inference_graph.pb"
        else:
            file_name = "light_classification/real_graph.pb"
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(file_name, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess = tf.Session(graph=self.detection_graph)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_expanded = np.expand_dims(image, axis=0)

        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
                                    [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                                    feed_dict={self.image_tensor: image_expanded})

        if scores[0][0] > 0.5:
            if classes[0][0] == 1:
                return TrafficLight.GREEN
            elif classes[0][0] == 2:
                return TrafficLight.RED
            elif classes[0][0] == 3:
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
