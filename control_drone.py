import tensorflow as tf
#from keras import backend as K
import numpy as np
import cv2
import rospy
import message_filters
import std_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from universal_teleop.msg import Control
from mav_msgs.msg import RateThrust
image_ = "/home/mohamedisse/Documents/Imitation_Learning/data_full/images/left2.jpg"
HEIGHT = 384
WIDTH = 512

pub_control = rospy.Publisher('/control_nodes/universal_teleop/controls', Control, queue_size=100)
pub_velocity = rospy.Publisher('/uav/input/rateThrust', RateThrust, queue_size=100)
#/uav/input/rateThrust
config = tf.ConfigProto()
config.gpu_options.allow_growth = True
with tf.Session(config=config) as sess:

    class ControlInference:
        def __init__(self, model, sess):
            self.model = model
            self.sess = sess
            img = cv2.imread(image_, 1)
            img = img / 255.
            #img = np.resize(img, [HEIGHT, WIDTH])
            img = np.expand_dims(img, 0)
            #with self.sess as sess:
            logits = self.model.predict(img, steps=1)
            #print("B {}".format(logits))
            # list_ = self.sess.graph.get_operations()
            # for l in list_:
            #     if l.name.startswith("output/Tanh"):
            #         print(l)

        def infer(self, img):
            #img = cv2.imread(image_, 1)
            #cv2.imwrite("/home/mohamedisse/catkin_ws/one.jpg", img)
            img = img / 255.
            #img = np.resize(img, [HEIGHT, WIDTH])
            img = np.expand_dims(img, 0)
            #print(img.shape)
            #img = tf.convert_to_tensor(img, dtype=tf.float32)
            #x = np.arange(10, dtype='int32')
            #y = x.view('float32')
            #y[:] = x

            #img_tensor = tf.convert_to_tensor(img, tf.uint8)
            #img_tensor = tf.expand_dims(img_tensor, 0)
            #image = tf.image.resize_images(img_tensor, size=[HEIGHT, WIDTH])
            #val = sess.run(img)


            tf.keras.backend.set_session(sess)
            logits = self.model.predict(img, steps=1)

            #print("val shape {}".format(img.shape))
            #print("img shape {}".format(img_tensor.shape))
            #logits = self.model.predict(img, steps=1)

            return logits
            #new_model.summary()

    def callback(image_data_l, image_data_r):
        global inference
        global sess
        #print("callback hit")
        # process image data #
        bridge = CvBridge()
        try:
            cv_image_l = bridge.imgmsg_to_cv2(image_data_l, "bgr8")
            cv_image_r = bridge.imgmsg_to_cv2(image_data_r, "bgr8")
        except CvBridgeError as e:
          print(e)


        #cv_image_l = cv_image_l / 255.
        #cv_image_r = cv_image_r / 255.

        #cv_image_l = np.expand_dims(cv_image_l, 0)
        #cv_image_r = np.expand_dims(cv_image_r, 0)
        #print(cv_image_l.shape)
        l = np.asarray(cv_image_l)
        r = np.asarray(cv_image_r)
        logits_l = inference.infer(l)
        logits_r = inference.infer(r)

        #print("Al {}".format(logits_l))
        #print("Ar {}".format(logits_r))
        logits_mean = np.mean([logits_l, logits_r], 0 )

        #print(logits_l)
        #print(logits_r)
        # std_msgs/Header header
        #   uint32 seq
        #   time stamp
        #   string frame_id
        # geometry_msgs/Vector3 angular_rates
        #   float64 x
        #   float64 y
        #   float64 z
        # geometry_msgs/Vector3 thrust
        #   float64 x
        #   float64 y
        #   float64 z

        c = Control()
        r = RateThrust()
        idleThrust = 9.81
        pitch_multi = 1.0
        roll_multi = 1.0
        yaw_multi = 1.0
        vertical_multi = 2.0 * idleThrust

        scale_factor = 4.0
        yaw = logits_mean[0][2] * scale_factor
        pitch = logits_mean[0][1] * scale_factor
        roll = -logits_mean[0][0] * scale_factor
        vertical = logits_mean[0][3] * scale_factor
        r.thrust.z = idleThrust

        r.angular_rates.y = (pitch ** 3.0) * pitch_multi
        r.angular_rates.x = (roll ** 3.0) * roll_multi
        r.angular_rates.z = (yaw ** 3.0) * yaw_multi
        r.thrust.z = ((vertical ** 3.0) * vertical_multi) + idleThrust
        scale = 30.0
        for i in range(4):

            if i == 0:
                c.control = "roll"
                c.value = logits_mean[0][0] * scale

            elif i == 1:
                c.control = "pitch"
                c.value = logits_mean[0][1] * scale

            elif i == 2:
                c.control = "yaw"
                c.value = logits_mean[0][2] * scale

            elif i == 3:
                c.control = "vertical"
                c.value = logits_mean[0][3] * scale
            c.header = std_msgs.msg.Header()
            c.header.stamp = rospy.Time.now()
            pub_control.publish(c)

        r.header = std_msgs.msg.Header()
        r.header.stamp = rospy.Time.now()
        pub_velocity.publish(r)
        print("published")

    def main():
        rospy.init_node('control_drone')
        rate = rospy.Rate(10)

        image_sub_l = message_filters.Subscriber('/colored_gates_left', Image)
        image_sub_r = message_filters.Subscriber('/colored_gates_right', Image)

        # print("created subscribers")
        ts = message_filters.ApproximateTimeSynchronizer([image_sub_l, image_sub_r], 10, 0.3)
        ts.registerCallback(callback)

        rospy.spin()

    model_ = "/home/mohamedisse/Documents/Imitation_Learning/graphs/fresh_models/model_model.207-0.0475998.h5"
    model = tf.keras.models.load_model(model_)
    inference = ControlInference(model, sess)
    main()
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# string control
# float32 value
