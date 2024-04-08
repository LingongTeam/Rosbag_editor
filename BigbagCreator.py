import os
from rosbag import Bag
import rospy
from tqdm import tqdm

class BigbagCreator():
    def __init__(self):

        inputbagpath = '/media/oliver/Elements SE/Lingongdata/20230103/temp/' # 'input bag files path'
        outputbag = '/media/oliver/Elements SE/Lingongdata/20230103/202301032.bag'# output bag file'
        files= os.listdir(inputbagpath)
        files.sort(key=lambda x:int(x[:-4]))
        with Bag(outputbag, 'w') as o:
            for file in files:
                with Bag(inputbagpath + file, 'r') as bag:
                    with tqdm(total=int(bag.get_message_count())) as pbar:
                        pbar.set_description('Processing '+ file + ':')
                        for i,[topic,msg,t] in enumerate(bag.read_messages()):
                            o.write(topic, msg, t)
                            pbar.update(1)
            


if __name__ == '__main__':


    try:
        image_creator = BigbagCreator()
    except rospy.ROSInterruptException:
        pass
