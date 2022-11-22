import os
from rosbag import Bag
import rospy
from tqdm import tqdm

class smallbagsCreator():
    def __init__(self):
        inputbag = '/media/oliver/Elements SE/2022-08-30_out1_camlidar.bag' # 'input big bag files' 
        outputbagpath = '/media/oliver/Elements SE/bags/'# output bag files'
        file=1
        outputbagpath = outputbagpath + inputbag.split("/")[-1][:-4] + '/'
        if not os.path.exists(outputbagpath):
            os.makedirs(outputbagpath)
        with Bag(inputbag, 'r') as bag:
            with tqdm(total=int(bag.get_message_count())) as pbar:
                pbar.set_description('Processing:')
                filename = str(file) + ".bag"
                print("Write file ", filename)
                o = Bag(outputbagpath + filename, 'w')
                # flag = True
                for i,[topic,msg,t] in enumerate(bag.read_messages()):
                    if i % 500 == 0 and i!=0 :
                        file_size = os.path.getsize(outputbagpath + filename)
                        if file_size*1e-9 >5:   # 根据大小确定是否写入下一个bags,这里设为5GB
                            o.close()
                            file = file + 1
                            filename = str(file) + ".bag"
                            print("Write file ", filename)
                            o = Bag(outputbagpath + filename, 'w')

                        
                    o.write(topic, msg, t)

                    pbar.update(1)
            


if __name__ == '__main__':

    try:
        image_creator = smallbagsCreator()
    except rospy.ROSInterruptException:
        pass
