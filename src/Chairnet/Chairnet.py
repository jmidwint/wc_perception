#!/usr/bin/env python
import rospy
from wc_perception.msg import CNN_out
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty, String
import utils

# <-- JKM

from wc_msgs.msg  import DriveCmd, Chair
import numpy as np
JKM=False

# -- JKM -->


from keras import backend as K

TEST_PHASE=0

class Chairnet (object):
    def __init__(self,
                 json_model_path,
                 weights_path, target_size=(200, 200),
                 crop_size=(150, 150),
                 imgs_rootpath="../models"):

        self.pub = rospy.Publisher("cnn_predictions", CNN_out, queue_size=5)
        self.feedthrough_sub = rospy.Subscriber("state_change", Bool, self.callback_feedthrough, queue_size=1)

        # Turn off publishing drive commands
        self.use_network_out = False
        print(" NOT Publishing  drive commands!") # TODO ros log this

        # JKM
        # Want separate toggle to save images separate from being able to publish drive commands
        self.save_img_sub = rospy.Subscriber("save_img", Bool, self.callback_save_img, queue_size=1)
        self.save_img = False
        print(" NOT saving NN produced images & Data to CSV!") # TODO ros log this


        self.imgs_rootpath = imgs_rootpath

        # JKM - Set up to publish to wc_msg format(s) DriveCmd & Chair
        self.pub_drive_cmd= rospy.Publisher("drive_wc", DriveCmd, queue_size=5)
        self.pub_drive_chair = rospy.Publisher("drive", Chair, queue_size=5)

        #JKM - to publish visualize results
        self.pub_visualize = rospy.Publisher("perception", Image, queue_size=5)

        # Set keras utils
        K.set_learning_phase(TEST_PHASE)

        # Load json and create model
        model = utils.jsonToModel(json_model_path)
        # Load weights
        model.load_weights(weights_path)
        print("Loaded model from {}".format(weights_path))

        model.compile(loss='mse', optimizer='sgd')
        self.model = model
        self.target_size = target_size
        self.crop_size = crop_size

    def callback_feedthrough(self, data):
        self.use_network_out = data.data
        if self.use_network_out == True:  print("Publish to drive:  ON")
        if self.use_network_out == False: print("Publish to drive:  OFF")


    # JKM - add ability to just save image separate from published drive commands 
    def callback_save_img(self, data):
        self.save_img = data.data
        if self.save_img == True:  print("Save Images:  ON")
        if self.save_img == False: print("Save Images:  OFF")


    def run(self):
        while not rospy.is_shutdown():
            msg = CNN_out()
            msg.header.stamp = rospy.Time.now()

            # <-- JKM
            # Define the message format & fill in the header.
            # Legacy message
            drive_msg = DriveCmd()
            drive_msg.header.stamp = rospy.Time.now()

            # Chair format message
            drive_chair_msg = Chair()
            drive_chair_msg.header.stamp = rospy.Time.now() 
            # JKM --> 

            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message("camera", Image, timeout=10)
                except:
                    pass
            ''' # JKM - comment this out for now 
            if self.use_network_out:
                print("Publishing commands!")
            else:
                print("NOT Publishing commands!")
            '''

            cv_img, fn = utils.callback_img(data, self.target_size, self.crop_size,
                self.imgs_rootpath, self.save_img)
            img = np.asarray(cv_img, dtype=np.float32) * np.float32(1.0/255.0)
            #if JKM: print("JKM2: cv_image.shape" , cv_image.shape)
            outs = self.model.predict_on_batch(img[None])
            steer, coll = outs[0][0], outs[1][0]
            msg.steering_angle = steer
            msg.collision_prob = coll
            self.pub.publish(msg)

            # JKM
            # fill in the rest of the drive msg , & the drive chair msg
            drive_msg, drive_chair_msg = utils.convert_to_drive_msg(
                                                 drive_msg, drive_chair_msg,
                                                 steer, coll)
            self.pub_drive_cmd.publish(drive_msg)      
            # Klugy - this whole part about drive control needs to be moved 
            #    to the  drive control package            
            if self.use_network_out:
                # print("Publishing Drive  commands!")
                self.pub_drive_chair.publish(drive_chair_msg)
            else:
                # print("NOT Publishing Drive std/String commands!")
                pass

            # jkm -print data to csv file
            if JKM: print("CSV: ",   fn,  steer[0], coll[0], drive_msg_str.data)
            if self.imgs_rootpath and self.save_img and fn != "":
                 if JKM: print ("save csv file")
                 utils.jkm_write_csv([fn, steer[0], coll[0], 
                             drive_chair.msg.data.data], self.imgs_rootpath)

            # jkm - visualize output of NN overlayed on input image
            self.pub_visualize.publish(utils.jkm_visualize_results(
                                         cv_img, 
                                         steer[0], coll[0],
                                         drive_msg.vel_fb.data,
                                         drive_chair_msg.data.data ))

 
