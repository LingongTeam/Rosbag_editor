import numpy as np
import os
import tqdm
from tqdm import tqdm

def merge_pc(front, back, mid360):
    back[:, 5] += 6
    mid360[:, 5] += 12
    merge_data = np.concatenate((front, back, mid360), axis=0)
    return merge_data

path='/media/oliver/Elements SE/rosbag2_2024_04_03-16_32_39/point_cloud'
# Get a list of all files in the directory
front_files = os.listdir(path + '/front')
front_files.sort()
back_files = os.listdir(path + '/back')
back_files.sort()
mid360_files = os.listdir(path + '/mid360')
mid360_files.sort()
front_times = []
back_times = []
mid360_times = []
# Print the list of files
for file in front_files:
    sec, nsec, _ = file.split('.')
    time = float(sec) + float(nsec) / 1e9
    front_times.append(time)
for file in back_files:
    sec, nsec, _ = file.split('.')
    time = float(sec) + float(nsec) / 1e9
    back_times.append(time)
for file in mid360_files:
    sec, nsec, _ = file.split('.')
    time = float(sec) + float(nsec) / 1e9
    mid360_times.append(time)
front_times = np.array(front_times)
back_times = np.array(back_times)
mid360_times = np.array(mid360_times)
for f_id, f_time in tqdm(enumerate(front_times), total=front_times.shape[0], desc='Merging point cloud data', leave=False):
    back_time_idx = np.argmin(np.abs(back_times - f_time))
    back_time = np.abs(back_times[back_time_idx] - f_time)
    if back_time > 0.01:
        continue
    mid360_time_idx = np.argmin(np.abs(mid360_times - f_time))
    mid360_time = np.abs(mid360_times[mid360_time_idx] - f_time)
    if mid360_time > 0.01:
        continue
    # Read point cloud data
    front_data = np.fromfile(path + '/front/' + front_files[f_id], dtype=np.float64).reshape(-1, 7)
    back_data = np.fromfile(path + '/back/' + back_files[back_time_idx], dtype=np.float64).reshape(-1, 7)
    mid360_data = np.fromfile(path + '/mid360/' +  mid360_files[mid360_time_idx], dtype=np.float64).reshape(-1, 7)
    # Merge point cloud data
    merge_data = merge_pc(front_data, back_data, mid360_data)
    # Save the merged point cloud data
    merge_data.tofile(path + '/merge/' + front_files[f_id])