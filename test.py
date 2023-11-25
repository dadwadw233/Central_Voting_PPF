import os
import subprocess

root = '../../PPF_Dataset/test_data/synthetic'  # or real

dirs = os.listdir(root)

for dir_name in dirs:
    path = root + '/' + dir_name
    src_path = path + '/src.pcd'
    tgt_path = path + '/tgt.pcd'

    result = subprocess.run(['./build/central_voting', src_path, tgt_path], stdout=subprocess.PIPE, encoding='utf-8')
    print(result.stdout)
