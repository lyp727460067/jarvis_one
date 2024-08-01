import cv2
import numpy as np
img = cv2.imread('/home/lyp/data/0607/data/2024_4_29_15_8/image/179431939293000.png')
# 把图片转换为单通道的灰度图
def ifdack(img):
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 获取形状以及长宽
    img_shape = gray_img.shape
    height, width = img_shape[0], img_shape[1]
    size = gray_img.size
    # 灰度图的直方图
    hist = cv2.calcHist([gray_img], [0], None, [256], [0, 256])
    # 计算灰度图像素点偏离均值(128)程序
    a = 0
    ma = 0
    #np.full 构造一个数组，用指定值填充其元素
    reduce_matrix = np.full((height, width), 128)
    shift_value = gray_img - reduce_matrix
    shift_sum = np.sum(shift_value)
    da = shift_sum / size
    # 计算偏离128的平均偏差
    for i in range(256):
        ma += (abs(i-128-da) * hist[i])
    m = abs(ma / size)
    # 亮度系数
    k = abs(da) / m
    print(k)
    if k[0] > 1:
        # 过亮
        if da > 0:
            print("过亮","da:", da)
            black = 0
        else:
            print("过暗","da:",da)
            black = 1
    else:
        print("亮度正常","da:", da)
        black = 2

    return da

print(ifdack(img))