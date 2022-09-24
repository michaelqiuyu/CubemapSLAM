# _*_ coding: utf-8 _*_
# @Time: 2022/9/15 下午6:00
# @Author: xiongchao
# @software: Pycharm2021.2.2

import os


def create_image_list(image_dir: str, image_list_path: str):
    image_names = os.listdir(image_dir)
    image_names.sort()
    image_names = [str(image_name) + "\n" for image_name in image_names]

    with open(image_list_path, 'w') as file:
        file.writelines(image_names)


if __name__ == '__main__':
    image_dir = "/home/xiongchao/视频/data/loop2_front"
    image_list_path = "/home/xiongchao/视频/data/loop2_front.txt"
    create_image_list(image_dir, image_list_path)