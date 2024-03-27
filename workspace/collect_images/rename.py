import os
import shutil

def rename_images_in_folder(folder_path, new_name_prefix):
    # 获取文件夹中的所有文件
    files = os.listdir(folder_path)
    
    # 遍历所有文件
    for i, file in enumerate(files, start=986):
        # 构造新的文件名
        new_name = f"{new_name_prefix}_{i}.jpg"
        
        # 获取旧文件的完整路径
        old_file_path = os.path.join(folder_path, file)
        
        # 获取新文件的完整路径
        new_file_path = os.path.join(folder_path, new_name)
        
        # 重命名文件
        shutil.move(old_file_path, new_file_path)

# 使用函数
rename_images_in_folder('./cable_0317', 'cable_image')