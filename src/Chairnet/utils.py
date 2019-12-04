from cv_bridge import CvBridge, CvBridgeError
from keras.models import model_from_json
import cv2
import numpy as np
import rospy

bridge = CvBridge()

# JKM
import csv
from wc_msgs.msg  import DriveCmd
from std_msgs.msg import String
import math
JKM=False # debug , TODO turn into ros logging later

'''
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
std_msgs/String dir_fb
  string data
std_msgs/String vel_fb
  string data
std_msgs/String dir_lr
  string data
std_msgs/String vel_lr
  string data
'''

def callback_img(data, target_size, crop_size, rootpath, save_img):
    try:
        image_type = data.encoding
        img = bridge.imgmsg_to_cv2(data, image_type)
    except CvBridgeError, e:
        print e
    # jkm capture full image
    #
    fn=""
    if JKM: print("JKM img size before re-size: ",  img.shape)
    img = cv2.resize(img, target_size)
    if JKM: print("JKM img size after re-size: ",  img.shape)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = central_image_crop(img, crop_size[0], crop_size[1])
    if JKM: print("JKM img size after crop: ",  img.shape)
    # jkm - they originally just captured the final processed image
    if rootpath and save_img:
        temp = rospy.Time.now()
        fn="{}.jpg".format(temp)
        cv2.imwrite("{}/{}.jpg".format(rootpath, temp), img)
    
    # jkm - return visible image for now
    return img, fn
    # return np.asarray(img, dtype=np.float32) * np.float32(1.0/255.0), fn


def central_image_crop(img, crop_width, crop_heigth):
    """
    Crops the input PILLOW image centered in width and starting from the bottom
    in height.
    Arguments:
        crop_width: Width of the crop
        crop_heigth: Height of the crop
    Returns:
        Cropped image
    """
    half_the_width = img.shape[1] / 2
    img = img[(img.shape[0] - crop_heigth): img.shape[0],
              (half_the_width - (crop_width / 2)): (half_the_width + (crop_width / 2))]
    img = img.reshape(img.shape[0], img.shape[1], 1)
    return img

def jsonToModel(json_model_path):
    with open(json_model_path, 'r') as json_file:
        loaded_model_json = json_file.read()

    model = model_from_json(loaded_model_json)

    return model


def convert_to_drive_msg(drive_msg, drive_chair_msg, steer, coll):
    ''' Takes a partially filled in DriveCmd message with sequence number and time stamp.
        Converts the steer angle in radians to a direction left or right  
        And converts the probability of collision to command to a velocity 
        wheelchair.
        Command is a 4 ASCII character which is stored in the message as 4 separate Strings

        Format is: F/B 0-70 L/R 0-70
           F = Forward
           B = Back
           L = Left
           R = Right
     
    '''
    MAX_RANGE=70.0
    MAX_SPEED=50.0
    drive_msg.dir_fb.data='F' # For now we always move forward in autonomous mode
    drive_msg.vel_fb.data='20' # For now we always move at a fixed speed unless we have to stop
    drive_msg.dir_lr.data='L' # Calculate this
    drive_msg.vel_lr.data='00' # Calculate this
    #
    # Determine which direction 
    # steer > 0 , means counter-clockwise , so go towards the LEFT
    # steer < 0 , means clockwise, so go to the RIGHT
    #
    if (steer >= 0 ):
        drive_msg.dir_lr.data='L'
    else: 
       # steer < 0
       drive_msg.dir_lr.data='R'
    # 
    # Fill in the speed, already done above at the beginning
    # This should be a ROS param but for now just hard code it to 6
    # if we detect a collision, if so make a stop
    # TODO: This is pretty rudimentary & clugy, make the speed & stopping more graceful
    #        This should be moved to drive control package later. Not here. 
    THRESH=0.95 # 
    # JKM - JUNE 18  Comment out this part for now, setting velocity constant
    #
    #if (coll>THRESH):
    #    # Just stop, and do not go left or right
    #    new_speed = 0
    #    vel_lr=0
    #else:
    #    new_speed=int(round(MAX_SPEED -(MAX_SPEED*coll)))
    #    # left or right amount
    #    amount=(abs(steer) * 2*MAX_RANGE)/math.pi
    #    vel_lr=int(round(amount))
    #
    # JKM - JUNE 18
    new_speed = 20
    # left or right amount
    amount=(abs(steer) * 2*MAX_RANGE)/math.pi
    vel_lr=int(round(amount))
    
    #
    # Convert to string
    drive_msg.vel_fb.data= '{:02d}'.format(new_speed)
    drive_msg.vel_lr.data='{:02d}'.format(vel_lr)
    # Convert to one string of type standard message
    # drive_msg_str = String()
    drive_chair_msg.data.data = drive_msg.dir_fb.data + drive_msg.vel_fb.data + \
              drive_msg.dir_lr.data + drive_msg.vel_lr.data 
    if JKM: 
        print('DriveStr: ', drive_chair_msg.data.data)  
    return drive_msg, drive_chair_msg


def jkm_write_csv(row, rootpath):
    # write a row to the end of the csv file
    # print(" JKM - in jkm_write_csv")
    fn_path= rootpath + '/results.csv'
    if JKM: print (" JKM about to write to ", fn_path)
    with open(fn_path, 'a') as csvFile:
        w = csv.writer(csvFile)
        w.writerow(row)

    csvFile.close()


def jkm_add_data_to_image(img, steer=0, coll=0, vel='00', drive='_NONE_'):
    # Add overlay of the data to a BGR colour image
    #
    # TODO: This is hardcoded now for a 200x200x3 BGR image but should used 
    #   the read in constants for the end size. Also perhaps these should also be params?
    #
    # 1. Draw the outline of the half steering wheel
    # ============================================
    # Constants: Half Circle 
    axes5040=(50,40)   # top circle
    #axes3020=(30, 20) # smaller half circle
    axes4030=(40,30)   # lower half circle
    angle = 0
    startAngle = 180
    endAngle = 360
    center = (100, 198)
    #
    # Constants: Color and thickness of lines
    color=(255,255,255) # 
    thickness, filled = 1, 1 # 1, -1
    linetype = cv2.LINE_AA
    #
    # Draw 2 half circles
    cv2.ellipse(img, center, axes5040, angle, startAngle, endAngle,color,thickness,linetype)
    cv2.ellipse(img, center, axes4030, angle, startAngle, endAngle,color,filled),linetype
    #cv2.ellipse(img, center, axes3020, angle, startAngle, endAngle, color, thickness)
    #
    # Constants: Lines on Half Circle, x1y1 -> x2y2
    leftline_start=(47,198)
    leftline_end=(60, 198)
    rightline_start=(140, 198)
    rightline_end=(153, 198)
    centerline_start=(100, 155)
    centerline_end=(100, 170)
    #
    # Draw 3 lines
    cv2.line(img,rightline_start, rightline_end,color,thickness,linetype)
    cv2.line(img,leftline_start, leftline_end,color,thickness,linetype)
    cv2.line(img,centerline_start, centerline_end,color,thickness,linetype)
    #
    # 2. Add the data as text
    # ======================== 
    # Constants - Text
    # font = cv2.FONT_HERSHEY_SIMPLEX
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 0.30 # 0.25
    textcolor= (0,0,0)
    # Draw Text Fixed
    cv2.putText(img,'L',(40,197), font, font_scale, textcolor, thickness,linetype)
    cv2.putText(img,'R',(160,197), font, font_scale, textcolor, thickness,linetype)
    #cv2.putText(img,'STEERING',(86,197), font, font_scale, textcolor, thickness,linetype) 
    #
    # Draw Text Data to display variables inside the steering wheel
    #   Add the text for variables: 
    #         steer, probability of collision, velocity, drive command
    cv2.putText(img,'{0:.2f}'.format(steer),(90,178), font,font_scale,
                 textcolor, thickness,linetype)
    cv2.putText(img,'COL: {}%  VEL: {}'.format(int(coll*100), vel),(55,197), font,font_scale,
                 textcolor, thickness,linetype)
    #cv2.putText(img, vel ,(115,197),font,font_scale, textcolor, thickness,linetype)
    cv2.putText(img, drive,(86,188), font, font_scale, textcolor, thickness,linetype)     
    #
    # 3. Add the graphical steering angle
    # ====================================
    #
    axes4545=(45,45)   #
    steerAngleStraight=270  
    steerAngleStart = steerAngleStraight - (steer * 180 / math.pi)
    steerthick=4
    steerAngleEnd=steerAngleStart+steerthick
    cv2.ellipse(img, center, axes5040, angle, steerAngleStart, steerAngleEnd, 
                  textcolor, steerthick, linetype)
    # cv2.circle(img,(100,100), 2, textcolor, -1)
    return img 

def jkm_visualize_results(img, steer, coll, vel, drive):
    # Take the image and overlay the data,
    # convert to ros msg
    # for now , just return what we get
    # Inputs: 
    #   img: cv grayscale image
    #   steer: float point steering angle
    #   coll: float point probability of collision
    #   vel:  string of scaled velocity 0-70
    #   drive: string of command sent to drive the chair
    if JKM: print("in visualize")
    color_img=cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    data_img=jkm_add_data_to_image(color_img, steer=steer, coll=coll, vel=vel, drive=drive)
    try:
        imgmsg = bridge.cv2_to_imgmsg(data_img, encoding="bgr8")
    except CvBridgeError, e:
        print e
    return imgmsg
