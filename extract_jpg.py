import roslib;  
import rosbag
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from tqdm import tqdm
path='/media/oliver/Elements SE/dataset/Lingongdata/20230426/pic/' #存放图片的位置
class ImageCreator():


    def __init__(self):
        self.bridge = CvBridge()
        with rosbag.Bag('/media/oliver/Elements SE/dataset/Lingongdata/20230426/2023-04-26.bag', 'r') as bag:   #要读取的bag文件名称，需要修改；
            with tqdm(total=int(bag.get_message_count())) as pbar:
                pbar.set_description('Processing:')
                for topic,msg,t in bag.read_messages():
                    pbar.update(1)
                    if topic == "/usb_cam/image_raw":  #图像的topic；
                            try:
                                cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                            except CvBridgeError as e:
                                print (e)
                            timestr = "%.6f" %  msg.header.stamp.to_sec()
                            #%.6f表示小数点后带有6位，可根据精确度需要修改；
                            image_name = timestr+ ".jpg" #图像命名：时间戳.jpg
                            cv2.imwrite(path+image_name, cv_image)  #保存；
                    elif topic == "/usb_cam1/image_raw":  #图像的topic；
                            try:
                                cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                            except CvBridgeError as e:
                                print (e)
                            timestr = "%.6f" %  msg.header.stamp.to_sec()
                            #%.6f表示小数点后带有6位，可根据精确度需要修改；
                            image_name = timestr+ "1.jpg" #图像命名：时间戳.jpg
                            cv2.imwrite(path+image_name, cv_image)  #保存；


if __name__ == '__main__':

    #rospy.init_node(PKG)

    try:
        image_creator = ImageCreator()
    except rospy.ROSInterruptException:
        pass
