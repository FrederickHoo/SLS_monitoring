import numpy as np
import cv2
import time
import threading
import msvcrt

from MvCameraControl_class import *

g_bExit = False


# 需要显示的图像数据转换，获取视频流并显示
def image_control_imshow(data, stFrameInfo, fps):
    data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
    frame = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_YUYV)
    frame = cv2.resize(frame, None, fx=0.2, fy=0.2, interpolation=cv2.INTER_CUBIC)
    # 灰度话
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 设置预定义的字典
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    # 使用默认值初始化检测器参数
    parameters = cv2.aruco.DetectorParameters_create()
    # 使用aruco.detectMarkers()函数可以检测到marker，返回ID和标志板的4个角点坐标
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # 画出标志位置
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    print(ids)

    if np.all(ids != None):
        cv2.aruco.drawDetectedMarkers(frame, corners)

    frame = cv2.resize(frame, (1080, 720))
    cv2.imshow('aruco_detect', frame)
    key = cv2.waitKey(1) & 0xff
    return key
    # if stFrameInfo.enPixelType == 17301505:
    #     image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
    #     image_show(image=image)
    # # elif stFrameInfo.enPixelType == 17301514:
    #
    # elif stFrameInfo.enPixelType == 35127316:
    #     data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
    #     image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
    #     image_show(image=image)
    # elif stFrameInfo.enPixelType == 34603039:
    #     data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
    #     image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
    #     image_show(image=image)


# 为线程定义一个函数
def work_thread(cam, pData):
    stOutFrame = MV_FRAME_OUT()
    memset(byref(stOutFrame), 0, sizeof(stOutFrame))
    fps = 0.0
    while True:
        t1 = time.time()
        ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)
        if None != stOutFrame.pBufAddr and 0 == ret:
            # print(stOutFrame.stFrameInfo.enPixelType)
            print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
            pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)()
            cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                               stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)
            data = np.frombuffer(pData,
                                 count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2),
                                 dtype=np.uint8)
            key = image_control_imshow(data=data, stFrameInfo=stOutFrame.stFrameInfo, fps=fps)
            fps = (fps + (1. / (time.time() - t1))) / 2
            if key == 27:
                break

        else:
            print("no data[0x%x]" % ret)
        nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)
        if g_bExit == True:
            break


stDevInfo = MV_CC_DEVICE_INFO()
stGigEDev = MV_GIGE_DEVICE_INFO()

# if sys.version >= '3':
# input the remote camera ip address
deviceIp = "10.16.10.168"  # input("please input current camera ip : ")
netIp = "10.16.31.203"  # input("please input net export ip : ")
# else:
#     deviceIp = raw_input("please input current camera ip : ")
#     netIp = raw_input("please input net export ip : ")
# 切片为list
deviceIpList = deviceIp.split('.')
# 对常规ip 做位运算，得到int类型的ip地址
stGigEDev.nCurrentIp = (int(deviceIpList[0]) << 24) | (int(deviceIpList[1]) << 16) | (
        int(deviceIpList[2]) << 8) | int(deviceIpList[3])
# print(stGigEDev.nCurrentIp)

netIpList = netIp.split('.')
stGigEDev.nNetExport = (int(netIpList[0]) << 24) | (int(netIpList[1]) << 16) | (int(netIpList[2]) << 8) | int(
    netIpList[3])

stDevInfo.nTLayerType = MV_GIGE_DEVICE
stDevInfo.SpecialInfo.stGigEInfo = stGigEDev

# ch:创建相机实例 | en:Creat Camera Object
cam = MvCamera()

# ch:选择设备并创建句柄 | en:Select device and create handle
ret = cam.MV_CC_CreateHandle(stDevInfo)
if ret != 0:
    print("create handle fail! ret[0x%x]" % ret)
    sys.exit()

# ch:打开设备 | en:Open device
ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
if ret != 0:
    print("open device fail! ret[0x%x]" % ret)
    sys.exit()

# ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
if stDevInfo.nTLayerType == MV_GIGE_DEVICE:
    nPacketSize = cam.MV_CC_GetOptimalPacketSize()
    if int(nPacketSize) > 0:
        ret = cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
        if ret != 0:
            print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
    else:
        print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)

# ch:设置触发模式为off | en:Set trigger mode as off
ret = cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
if ret != 0:
    print("set trigger mode fail! ret[0x%x]" % ret)
    sys.exit()

# ch:开始取流 | en:Start grab image
ret = cam.MV_CC_StartGrabbing()
print(ret)

if ret != 0:
    print("start grabbing fail! ret[0x%x]" % ret)
    sys.exit()
try:
    hThreadHandle = threading.Thread(target=work_thread, args=(cam, None))
    hThreadHandle.start()
except:
    print("error: unable to start thread")

print("press a key to stop grabbing.")
msvcrt.getch()

g_bExit = True
hThreadHandle.join()

# ch:停止取流 | en:Stop grab image
ret = cam.MV_CC_StopGrabbing()
if ret != 0:
    print("stop grabbing fail! ret[0x%x]" % ret)
    sys.exit()

# ch:关闭设备 | Close device
ret = cam.MV_CC_CloseDevice()
if ret != 0:
    print("close deivce fail! ret[0x%x]" % ret)
    sys.exit()

# ch:销毁句柄 | Destroy handle
ret = cam.MV_CC_DestroyHandle()
if ret != 0:
    print("destroy handle fail! ret[0x%x]" % ret)
    sys.exit()
