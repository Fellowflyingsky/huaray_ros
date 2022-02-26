#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include "VideoRender.h"
#include "IMVApi.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iostream"
#include "mutex"

using namespace std;

//枚举触发方式
enum ETrigType
{
  trigContinous = 0,	//连续拉流
  trigSoftware = 1,	//软件触发
  trigLine = 2,		//外部触发
};

string m_currentCameraKey;// 当前相机key | current camera key
IMV_HANDLE							m_devHandle;				// 相机句柄 | camera handle
image_transport::Publisher image_pub;
std::mutex pubimage_mutex;
std::vector<cv::Mat> image_list;//存放图像

void CameraCheck();
bool CameraOpen();
bool CameraStart();
void CameraChangeTrig(ETrigType trigType);
static void FrameCallback(IMV_Frame* pFrame, void* pUser);//回调函数
void publish_img();//发布ROS消息

//add by hushenghui
//设置曝光
bool SetExposeTime(double exposureTime);
//设置增益
bool SetAdjustPlus(double gainRaw);

//设置伽马

bool SetGamma(double gama);

bool SetExposeTime(double exposureTime)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "ExposureTime", exposureTime);
  if (IMV_OK != ret)
  {
    printf("set ExposureTime value = %0.2f fail, ErrorCode[%d]\n", exposureTime, ret);
    return false;
  }

  return true;
}

bool SetAdjustPlus(double gainRaw)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "GainRaw", gainRaw);
  if (IMV_OK != ret)
  {
    printf("set GainRaw value = %0.2f fail, ErrorCode[%d]\n", gainRaw, ret);
    return false;
  }

  return true;
}

bool SetGamma(double gama)
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  ret = IMV_SetDoubleFeatureValue(m_devHandle, "Gamma", gama);
  if (IMV_OK != ret)
  {
    printf("set GainRaw value = %0.2f fail, ErrorCode[%d]\n", gama, ret);
    return false;
  }

  return true;
}


void CameraCheck()
{
  IMV_DeviceList deviceInfoList;
  if (IMV_OK != IMV_EnumDevices(&deviceInfoList, interfaceTypeAll))
  {
    printf("Enumeration devices failed!\n");
    return;
  }

  // 打印相机基本信息（key, 制造商信息, 型号, 序列号）
  for (unsigned int i = 0; i < deviceInfoList.nDevNum; i++)
  {
    printf("Camera[%d] Info :\n", i);
    printf("    key           = [%s]\n", deviceInfoList.pDevInfo[i].cameraKey);
    printf("    vendor name   = [%s]\n", deviceInfoList.pDevInfo[i].vendorName);
    printf("    model         = [%s]\n", deviceInfoList.pDevInfo[i].modelName);
    printf("    serial number = [%s]\n", deviceInfoList.pDevInfo[i].serialNumber);
  }

  if (deviceInfoList.nDevNum < 1)
  {
    printf("no camera.\n");
  //	msgBoxWarn(tr("Device Disconnected."));
  }
  else
  {
    //默认设置列表中的第一个相机为当前相机，其他操作比如打开、关闭、修改曝光都是针对这个相机。
    m_currentCameraKey = deviceInfoList.pDevInfo[0].cameraKey;
  }
}

/**
 * @brief CameraOpen
 * @return
 *
 */
bool CameraOpen()
{
  int ret = IMV_OK;

  if (m_currentCameraKey.length() == 0)
  {
    printf("open camera fail. No camera.\n");
    return false;
  }

  if (m_devHandle)
  {
    printf("m_devHandle is already been create!\n");
    return false;
  }

  string cameraKeyArray = m_currentCameraKey;
  const char* cameraKey = cameraKeyArray.data();

  ret = IMV_CreateHandle(&m_devHandle, modeByCameraKey, (void*)cameraKey);
  if (IMV_OK != ret)
  {
    printf("create devHandle failed! cameraKey[%s], ErrorCode[%d]\n", cameraKey, ret);
    return false;
  }

  // 打开相机
  // Open camera
  ret = IMV_Open(m_devHandle);
  if (IMV_OK != ret)
  {
    printf("open camera failed! ErrorCode[%d]\n", ret);
    return false;
  }

  return true;
}

/**
 * @brief CameraStart
 * @return
 */
bool CameraStart()
{
  if (!m_devHandle)
  {
    return false;
  }

  int ret = IMV_OK;

  if (IMV_IsGrabbing(m_devHandle))
  {
    printf("camera is already grebbing.\n");
    return false;
  }


  ret = IMV_AttachGrabbing(m_devHandle, FrameCallback, nullptr);
  if (IMV_OK != ret)
  {
    printf("Attach grabbing failed! ErrorCode[%d]\n", ret);
    return false;
  }

  ret = IMV_StartGrabbing(m_devHandle);
  if (IMV_OK != ret)
  {
    printf("start grabbing failed! ErrorCode[%d]\n", ret);
    return false;
  }

  return true;
}

/**
 * @brief CameraChangeTrig
 * @param trigType
 */
void CameraChangeTrig(ETrigType trigType)
{
  if (!m_devHandle)
  {
    return;
  }

  int ret = IMV_OK;

  if (trigContinous == trigType)
  {
    // 设置触发模式
    // set trigger mode
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "Off");
    if (IMV_OK != ret)
    {
      printf("set TriggerMode value = Off fail, ErrorCode[%d]\n", ret);
      return;
    }
  }
  else if (trigSoftware == trigType)
  {
    // 设置触发器
    // set trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
    if (IMV_OK != ret)
    {
      printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发模式
    // set trigger mode
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
    if (IMV_OK != ret)
    {
      printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发源为软触发
    // set triggerSource as software trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Software");
    if (IMV_OK != ret)
    {
      printf("set TriggerSource value = Software fail, ErrorCode[%d]\n", ret);
      return;
    }
  }
  else if (trigLine == trigType)
  {
    // 设置触发器
    // set trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
    if (IMV_OK != ret)
    {
      printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发模式
    // set trigger mode
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
    if (IMV_OK != ret)
    {
      printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
      return;
    }

    // 设置触发源为Line1触发
    // set trigggerSource as Line1 trigger
    ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Line1");
    if (IMV_OK != ret)
    {
      printf("set TriggerSource value = Line1 fail, ErrorCode[%d]\n", ret);
      return;
    }
  }
}

/**
 * @brief The CFrameInfo class
 */
class CFrameInfo
{
public:
  CFrameInfo()
  {
    m_pImageBuf = NULL;
    m_nBufferSize = 0;
    m_nWidth = 0;
    m_nHeight = 0;
    m_ePixelType = gvspPixelMono8;
    m_nPaddingX = 0;
    m_nPaddingY = 0;
    m_nTimeStamp = 0;
  }

  ~CFrameInfo()
  {
  }

public:
  unsigned char*	m_pImageBuf;
  int				m_nBufferSize;
  int				m_nWidth;
  int				m_nHeight;
  IMV_EPixelType	m_ePixelType;
  int				m_nPaddingX;
  int				m_nPaddingY;
  uint64_t		m_nTimeStamp;
};

//回调函数
/**
 * @brief FrameCallback
 * @param pFrame
 * @param pUser
 */
double time1,time2,time3;
cv::Mat g_BGRImage;
static void FrameCallback(IMV_Frame* pFrame, void* pUser)
{
//  time1 = ros::Time::now().toSec();
//  cout<<"time1: "<<time1<<endl;
  CFrameInfo frameInfo;
  frameInfo.m_nWidth = (int)pFrame->frameInfo.width;
  frameInfo.m_nHeight = (int)pFrame->frameInfo.height;
  frameInfo.m_nBufferSize = (int)pFrame->frameInfo.size;
  frameInfo.m_nPaddingX = (int)pFrame->frameInfo.paddingX;
  frameInfo.m_nPaddingY = (int)pFrame->frameInfo.paddingY;
  frameInfo.m_ePixelType = pFrame->frameInfo.pixelFormat;
  frameInfo.m_pImageBuf = (unsigned char *)malloc(sizeof(unsigned char) * frameInfo.m_nBufferSize);
  frameInfo.m_nTimeStamp = pFrame->frameInfo.timeStamp;

//  cout<<"type: "<<frameInfo.m_ePixelType <<endl;

  // 内存申请失败，直接返回
  // memory application failed, return directly
  if (frameInfo.m_pImageBuf != nullptr)
  {
    /* 释放内存 */
    free(frameInfo.m_pImageBuf);

    if (g_BGRImage.empty())
    {
      g_BGRImage.create(frameInfo.m_nHeight, frameInfo.m_nWidth, CV_8UC3);
    }
    memcpy(frameInfo.m_pImageBuf, pFrame->pData, frameInfo.m_nBufferSize);
    cv::Mat BayGB8_Image(frameInfo.m_nHeight, frameInfo.m_nWidth, CV_8UC1, (unsigned char *)frameInfo.m_pImageBuf);
    pubimage_mutex.lock();
    cvtColor(BayGB8_Image, g_BGRImage, cv::COLOR_BayerGR2BGR);  //COLOR_BayerGR2BGR
    image_list.push_back(g_BGRImage);
//    time2 = ros::Time::now().toSec();
//    cout.precision(18);
//    cout<<"time2: "<<time1<<endl;
//    cout<<"time2-1: "<<time2 - time1<<endl;
    pubimage_mutex.unlock();
  }
}

/**
 * @brief publish_img
 * 发布ROS消息
 */
void publish_img()
{
  pubimage_mutex.lock();
  if(image_list.size() > 0)
  {
    cv::Mat image_temp = image_list.front();
    std::vector<cv::Mat>::iterator k = image_list.begin();
    image_list.erase(k);
    ros::Time time_c = ros::Time::now();
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_temp).toImageMsg();
    img_msg->header.stamp = time_c;
//    time3 = ros::Time::now().toSec();
    image_pub.publish(img_msg);
//    cout.precision(18);
//    cout<<"time3: "<<time3<<endl;
//    cout<<"time3 - time1: "<<time3 - time1<<endl;
    if(image_list.size()>20)
    {
      image_list.clear();
    }

  }
  pubimage_mutex.unlock();
//  cout<<"image_list.size(): "<<image_list.size()<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "huaray_ros");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_pub = it.advertise("/huaray/image_raw",1);

    CameraCheck();
    CameraOpen();
    CameraStart();
    SetAdjustPlus(2.01);
    SetExposeTime(32680.70);
    SetGamma(0.86);

    CameraChangeTrig(ETrigType::trigContinous);


    ros::Rate(150);
    while(ros::ok())
    {
      publish_img();
      ros::spinOnce();
    }
    return 0;
}
