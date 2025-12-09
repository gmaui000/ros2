#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include <memory>
#include <vector>
#include <string>

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};

bool is_undistorted = true;
bool exit_flag = false;
int width, height;
std::vector<PixelFormat> PIXEL_FORMAT = { RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8 };
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;

image_transport::Publisher pub;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
  if (NULL == pstMVDevInfo)
  {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
  {
    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
  {
    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  else
  {
    printf("Not support.\n");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file, std::shared_ptr<rclcpp::Node> node) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    RCLCPP_ERROR(node->get_logger(), msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];   
  if (image_scale < 0.1) image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int nRet;

  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
  std::string msg = "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    RCLCPP_INFO(node->get_logger(), msg.c_str());
  } else {
    if(ExposureAutoMode == 2) {
      RCLCPP_WARN(node->get_logger(), "Fail to set Exposure Auto Mode to Continues");
    }
    else {
      RCLCPP_INFO(node->get_logger(), msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) + "us";
      RCLCPP_INFO(node->get_logger(), msg.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) + "us";
      RCLCPP_INFO(node->get_logger(), msg.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg = "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      RCLCPP_INFO(node->get_logger(), msg.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    RCLCPP_INFO(node->get_logger(), msg.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      RCLCPP_INFO(node->get_logger(), msg.c_str());
    } else {
      RCLCPP_ERROR(node->get_logger(), "Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    RCLCPP_INFO(node->get_logger(), msg.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    RCLCPP_INFO(node->get_logger(), msg.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Fail to set Gamma");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) {
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true;
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {};
  
  unsigned char* pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue);
  unsigned char* pDataForBGR = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);

  if (pData == nullptr || pDataForBGR == nullptr) {
    printf("Memory allocation failed!\n");
    if (pData) free(pData);
    if (pDataForBGR) free(pDataForBGR);
    return nullptr;
  }

  auto node_ptr = rclcpp::Node::make_shared("mvs_trigger_worker");
  auto clock = node_ptr->get_clock();

  RCLCPP_INFO(node_ptr->get_logger(), "Worker thread started, waiting for camera frames...");
  int frame_count = 0;

  while (!exit_flag && rclcpp::ok()) {
    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, stParam.nCurValue, &stImageInfo, 1000);

    if (nRet == MV_OK) {
      frame_count++;

      rclcpp::Time rcv_time;
      
      // 简化共享内存访问逻辑
      if (trigger_enable && pointt != MAP_FAILED && pointt != nullptr && pointt->low != 0) {
        // 使用共享内存时间戳
        double time_pc = pointt->low / 1000000000.0;
        rcv_time = rclcpp::Time(static_cast<int64_t>(time_pc * 1e9));
      } else {
        // 使用当前时间
        rcv_time = clock->now();
      }
      
      stConvertParam.nWidth = stImageInfo.nWidth;
      stConvertParam.nHeight = stImageInfo.nHeight;
      stConvertParam.pSrcData = pData;
      stConvertParam.nSrcDataLen = stParam.nCurValue; 
      stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      stConvertParam.pDstBuffer = pDataForBGR;
      stConvertParam.nDstBufferSize = stParam.nCurValue * 3;
      nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
      if (MV_OK != nRet)
      {
        RCLCPP_WARN(node_ptr->get_logger(), "MV_CC_ConvertPixelType failed! nRet [%x], skipping this frame", nRet);
        continue;
      }
      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);
      if (image_scale > 0.0) {
        cv::resize(srcImage, srcImage, cv::Size(srcImage.cols * image_scale, srcImage.rows * image_scale), cv::INTER_LINEAR);
      } else {
        RCLCPP_WARN(node_ptr->get_logger(), "Invalid image_scale: %f. Skipping resize.", image_scale);
      }

      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", srcImage).toImageMsg();
      msg->header.stamp = rcv_time;
      pub.publish(*msg);
      if (frame_count % 20 == 1) {  // 每30帧打印一次
        RCLCPP_INFO(node_ptr->get_logger(), "Published frame %d to topic", frame_count);
      }
    } else if (nRet == (int)MV_E_GC_TIMEOUT) {
      RCLCPP_DEBUG(node_ptr->get_logger(), "Frame timeout (normal)");
    } else {
      RCLCPP_WARN(node_ptr->get_logger(), "MV_CC_GetOneFrameTimeout failed with error: 0x%x", nRet);
    }
  }

  if (pData) {
    free(pData);
    pData = nullptr;
  }

  if (pDataForBGR)
  {
    free(pDataForBGR);
    pDataForBGR = nullptr;
  }

  return 0;
}

class MvsTriggerNode : public rclcpp::Node
{
public:
  MvsTriggerNode() : Node("mvs_trigger")
  {
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    RCLCPP_ERROR(rclcpp::get_logger("mvs_trigger"), "Usage: %s <config_file>", argv[0]);
    return -1;
  }
  
  std::string params_file = std::string(argv[1]);
  auto node = std::make_shared<MvsTriggerNode>();
  auto it = std::make_shared<image_transport::ImageTransport>(node);
  int nRet = MV_OK;
  void *handle = NULL;
  rclcpp::Rate loop_rate(10);
  
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    RCLCPP_ERROR(node->get_logger(), msg.c_str());
    exit(-1);
  }
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  pub = it->advertise(pub_topic, 1);
  RCLCPP_INFO(node->get_logger(), "Image publisher created successfully");
  
  std::string path_for_time_stamp = "/tmp/timeshare";
  const char* shared_file_name = path_for_time_stamp.c_str();

  RCLCPP_INFO(node->get_logger(), "Setting up shared memory for timestamp at: %s", shared_file_name);


  int fd = open(shared_file_name, O_RDWR | O_CREAT, 0644);
  if (fd == -1) {
    RCLCPP_ERROR(node->get_logger(), "Failed to open or create shared memory file: %s", shared_file_name);
    return -1;
  }
  
  // 确保文件大小足够
  if (ftruncate(fd, sizeof(time_stamp)) == -1) {
    RCLCPP_ERROR(node->get_logger(), "Failed to set shared memory file size");
    close(fd);
    return -1;
  }
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, 0);
  if (pointt == MAP_FAILED) {
    RCLCPP_ERROR(node->get_logger(), "Failed to map shared memory");
    close(fd);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Shared memory mapped successfully");

  SetupSignalHandler();
 
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  RCLCPP_INFO(node->get_logger(), "Enumerating MVS devices...");
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(node->get_logger(), "MV_CC_EnumDevices fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Found %d MVS devices", stDeviceList.nDeviceNum);

  if (stDeviceList.nDeviceNum > 0)
  {
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL)
      {
        printf("Device info is NULL for device %d\n", i);
        return -1;
      } 
      PrintDeviceInfo(pDeviceInfo);            
    }  
  } 
  else
  {
    printf("Find No Devices!\n");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) 
  {
    RCLCPP_INFO(node->get_logger(), "Multiple devices found, searching for serial number: %s", expect_serial_number.c_str());
    if (expect_serial_number.empty()) 
    {
      RCLCPP_ERROR(node->get_logger(), "Expected serial number is empty!");
      return -1;
    }
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) 
    {
      if (stDeviceList.pDeviceInfo[i] == NULL) 
      {
        RCLCPP_WARN(node->get_logger(), "Device info is NULL for device %d", i);
        continue;
      }
      
      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE)
      {
        serial_number = std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber);
        RCLCPP_INFO(node->get_logger(), "Device %d: USB device, serial: %s", i, serial_number.c_str());
      }
      else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
      {
        serial_number = std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber);
        RCLCPP_INFO(node->get_logger(), "Device %d: GigE device, serial: %s", i, serial_number.c_str());
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "Device %d: Unknown device type", i);
        continue;
      }
      if (serial_number.empty()) 
      {
        RCLCPP_WARN(node->get_logger(), "Device %d: Serial number is empty", i);
        continue;
      }
      if (expect_serial_number == serial_number) 
      {
        find_expect_camera = true;
        nIndex = i;
        RCLCPP_INFO(node->get_logger(), "Found expected camera at index %d", i);
        break;
      }
    }
    if (!find_expect_camera) 
    {
      std::string msg = "Can not find the camera with serial number " + expect_serial_number;
      RCLCPP_ERROR(node->get_logger(), msg.c_str());
      return -1;
    }
  }
  else
  {
    RCLCPP_INFO(node->get_logger(), "Single device found, using index 0");
    nIndex = 0;
  }
  
  // select device and create handle
  RCLCPP_INFO(node->get_logger(), "Creating handle for device at index %d", nIndex);
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet)
  {
    RCLCPP_ERROR(node->get_logger(), "MV_CC_CreateHandle fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Handle created successfully");

  // open device
  RCLCPP_INFO(node->get_logger(), "Opening device...");
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet)
  {
    RCLCPP_ERROR(node->get_logger(), "MV_CC_OpenDevice fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Device opened successfully");

  RCLCPP_INFO(node->get_logger(), "Setting AcquisitionFrameRateEnable to false");
  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(node->get_logger(), "set AcquisitionFrameRateEnable fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "AcquisitionFrameRateEnable set successfully");

  RCLCPP_INFO(node->get_logger(), "Setting PixelFormat to %d", PixelFormat);
  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[PixelFormat]);
  if (nRet != MV_OK) {
    RCLCPP_ERROR(node->get_logger(), "Pixel setting can't work. nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "PixelFormat set successfully");

  RCLCPP_INFO(node->get_logger(), "Setting camera parameters...");
  setParams(handle, params_file, node);
  RCLCPP_INFO(node->get_logger(), "Camera parameters set successfully");

  // set trigger mode as on
  RCLCPP_INFO(node->get_logger(), "Setting TriggerMode to %d", trigger_enable);
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(node->get_logger(), "MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "TriggerMode set successfully");

  // set trigger source
  RCLCPP_INFO(node->get_logger(), "Setting TriggerSource to LINE0");
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(node->get_logger(), "MV_CC_SetTriggerSource fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "TriggerSource set successfully");

  // 检查触发模式设置
  MVCC_ENUMVALUE trigger_mode_enum;
  memset(&trigger_mode_enum, 0, sizeof(MVCC_ENUMVALUE));
  nRet = MV_CC_GetEnumValue(handle, "TriggerMode", &trigger_mode_enum);
  if (MV_OK == nRet) {
    RCLCPP_INFO(node->get_logger(), "Current TriggerMode value: %d", trigger_mode_enum.nCurValue);
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to get TriggerMode value, nRet: 0x%x", nRet);
  }

  MVCC_ENUMVALUE trigger_source_enum;
  memset(&trigger_source_enum, 0, sizeof(MVCC_ENUMVALUE));
  nRet = MV_CC_GetEnumValue(handle, "TriggerSource", &trigger_source_enum);
  if (MV_OK == nRet) {
    RCLCPP_INFO(node->get_logger(), "Current TriggerSource value: %d", trigger_source_enum.nCurValue);
  } else {
    RCLCPP_WARN(node->get_logger(), "Failed to get TriggerSource value, nRet: 0x%x", nRet);
  }

  // 如果处于触发模式，添加警告信息
  if (trigger_enable == 1) {
    RCLCPP_WARN(node->get_logger(), "Camera is in TRIGGER MODE. Make sure external trigger signal is provided!");
    RCLCPP_WARN(node->get_logger(), "If no trigger signal is available, set TriggerEnable: 0 in config file.");
  }

  RCLCPP_INFO(node->get_logger(), "Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    RCLCPP_ERROR(node->get_logger(), "Start Grabbing fail! nRet [%x]", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Grabbing started successfully");

  RCLCPP_INFO(node->get_logger(), "Creating worker thread...");
  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    RCLCPP_ERROR(node->get_logger(), "thread create failed. ret = %d", nRet);
    return -1;
  }
  RCLCPP_INFO(node->get_logger(), "Worker thread created successfully");

  while (!exit_flag && rclcpp::ok()) {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  
  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    RCLCPP_INFO(node->get_logger(), "Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  munmap(pointt, sizeof(time_stamp));
  rclcpp::shutdown();

  return 0;
}
