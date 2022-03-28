#ifndef __GPNETSDK_H__
#define __GPNETSDK_H__
#include <stdlib.h>
#include "GpTypes.h"
#include "ErrorCode.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* 设备类型 */
enum
{
    /* 光珀网络协议设备类型 */
    /* Optical Fiber Network Protocol Device Type */
    GP_DEVICE_TYPE_NET,
};

/* 帧图像数据类型 */
/* Frame image data type */
enum 
{
    /* 8bit 深度数据类型 单位为厘米*/
    /* 8bit depth data type in cm*/
    GP_FRAME_TYPE_DEPTH_CM8,
    /* 16bit 深度数据类型 单位为毫米 */
    /* 16bit depth data type in mm */
    GP_FRAME_TYPE_DEPTH16,
    /* 16bit 灰度数据类型  */
    /* 16bit gray data type */
    GP_FRAME_TYPE_GRAY16,
    /* RGB888彩色图像类型 */
    /* RGB888 color image type */
    GP_FRAME_TYPE_RGB888,
};

/* 数据流类型 */
/* Data stream type */
typedef enum
{
    /* 深度流 */
    /* Depth Stream */
    GP_STREAM_TYPE_DEPTH,
    /* 灰度流 */
    /* Grayscale stream */
    GP_STREAM_TYPE_GRAY,
}GpStreamType;

enum
{
    /* 无效设备句柄 */
    /* Invalid device handle */
    GP_INVALID_DEVICE_HANDLE = 0,
};

/* 设备状态 */
/* device status */
enum
{
    /* 设备处于关闭状态 */
    /* The device is off */
    GP_DEVICE_STATE_CLOSE,
    /* 正在打开设备 */
    /* Opening the device */
    GP_DEVICE_STATE_OPENING,
    /* 设备处于以打开状态 */
    /* The device is in an open state */
    GP_DEVICE_STATE_OPENED,
};

/* 流状态 */
enum
{
    /* 处于关闭状态 */
    GP_STREAM_STATE_CLOSE,
    /* 正在打开设备流 */
    GP_STREAM_STATE_OPENING,
    /* 处于以打开状态 */
    GP_STREAM_STATE_OPENED,
};

/* 图像帧结构 */
typedef struct GpFrame
{
    /*帧类型 值参考GP_FRAME_TYPE_*系列枚举 */
    GpInt32_t frameType;
    /* 图像宽 */
    GpInt32_t w;
    /* 图像高 */
    GpInt32_t h;
    /* 帧索引 */
    GpUInt32_t idx;
    /* 时间戳 单位毫秒 */
    GpUInt32_t timestamp;
    /* 帧数据长度 */
    GpUInt32_t frameLen;
    /* 帧数据 */
    void *data;
    
    /* -----私有成员用户不可访问以及修改------ */
    /* ----- Private member users cannot access and modify ------ */
    GpUInt32_t bufSize;
    GpInt32_t ref;
    void *mtx;
}GpFrame;

typedef struct ParamTRigidTrans3D
{
	double matR[9];
	double X;
	double Y;
	double Z;
}ParamTRigidTrans3D;

typedef struct Point3D 
{
	float x;
	float y;
	float z;
}Point3D;

/* 设备参数 */
/* Device parameters */
typedef struct GpDeviceParam
{
    /* 不同类型设备uri含义不同
     * 目前只支持网络设备
     * 网络设备为设备ip地址 字符串类型
     * */
    /* Different types of equipment uri have different meanings
      * Currently only supports network devices
      * The network device is the device ip address string type
      * */
    char uri[32];
}GpDeviceParam;

/* 设备版本信息 */
typedef struct GpDeviceInfo
{
    /* 嵌入式软件版本 */
    char softwareVersion[16];
    /* 系统版本 */
    char systemVersion[16];
    /* fpga程序版本 */
    char fpgaVersion[16];
    /* mac地址 */
    char mac[32];
    /* sn序列号 */
    char sn[32];
}GpDeviceInfo;

/* 客户私有数据存储 */
/* Customer private data storage */
typedef struct GpDevicePrivateData
{
    /* 客户私有数据 只能是字符串！！！ */
    /* Customer private data can only be a string! ! ! */
    char data[128];
}GpDevicePrivateData;


/**
 *@brief The parameters of camera lens
 * 相机镜头内参
 */
typedef struct __CAMERA_LENS_PARAM_T
{
	float fFocalLengthX;
	float fFocalLengthY;
	float fPrincipalPointX;
	float fPrincipalPointY;
        float fRadialDistortionK1;
	float fRadialDistortionK2;
	float fRadialDistortionK3;
	float fTangentialDistortionP1;
	float fTangentialDistortionP2;
}GpCameraLensParam;

/* 设备句柄 */
typedef void * GpDeviceHandle;

/* sdk初始化接口 */
extern void gpSdkInit();

/* sdk去初始化 */
void gpSdkUninit();

/**
 * @brief 创建设备句柄
 * @deviceType 设备类型 值参考GP_DEVICE_TYPE_*系列枚举
 * @return 设备句柄 如果返回GP_INVALID_DEVICE_HANDLE表示创建失败
 */
GpDeviceHandle gpCreateDeviceHandle(GpInt32_t deviceType);

/**
 * @brief 释放设备句柄以及资源
 * @param handle 设备句柄
 */
void gpReleaseDeviceHandle(GpDeviceHandle handle);

/**
 * @brief 打开设备
 * @param handle 设备句柄
 * @param param 设备参数
 * @return GP_EC_*
 * 该接口为异步接口！！！ 返回GP_EC_OK 表示接口调用成功
 * 表示sdk接受了该调用请求进入设备打开阶段 
 * 设备最终是否打开需要通过gpGetDeviceState接口查询设备状态是否为GP_DEVICE_STATE_OPENED
 * 接口允许重复调用 但不会重复发起打开设备操作
 */
GpInt32_t gpOpenDevice(GpDeviceHandle handle, const GpDeviceParam *param);

/**
 * @brief 关闭设备
 * @param handle 设备句柄
 * 该接口为异步接口！！！ 返回GP_EC_OK 表示接口调用成功 sdk进入关闭设备流程
 * 设备最终是否关闭需要通过gpGetDeviceState接口查询设备状态是否为GP_DEVICE_STATE_CLOSE
 */
void gpCloseDevice(GpDeviceHandle handle);

/**
 * @brief 获取设备状态
 * @param handle 设备句柄
 * @return GP_DEVICE_STATE_*
 */
GpInt32_t gpGetDeviceState(GpDeviceHandle handle);

/**
 * @brief 获取设备最后一次出错时的错误码 值含义参考GP_EC_系列
 * @param handle 设备句柄
 */
GpInt32_t gpGetDeviceLastErrorCode(GpDeviceHandle handle);

/**
 * @brief 清除设备错误码 清除后为GP_EC_OK
 * @param handle 设备句柄
 * @return 返回值为清除前的错误码值 值含义参考GP_EC_系列
 */
GpInt32_t gpClearDeviceLastErrorCode(GpDeviceHandle handle);

/**
 * @brief 打开设备流
 * @param handle 设备句柄
 * @param streamType 流类型  值参考GP_STREAM_TYPE_系列
 * @return GP_EC_*
 */
GpInt32_t gpStartStream(GpDeviceHandle handle, GpInt32_t streamType);

/**
 * @brief 关闭设备流
 * @param handle 设备句柄
 * @param streamType 流类型 值参考GP_STREAM_TYPE_系列
 */
void gpStopStream(GpDeviceHandle handle, GpInt32_t streamType);


/**
 * @brief 获取设备流状态 Get device stream status
 * @param handle 设备句柄 Device handle
 * @param streamType 流类型 值参考GP_STREAM_TYPE_*系列 Stream type value refer to GP_STREAM_TYPE_* series
 * @return 参考GP_STREAM_STATE_*系列 Refer to GP_STREAM_STATE_* series
 */
GpInt32_t gpGetStreamState(GpDeviceHandle handle,  GpInt32_t streamType);

/**
 * @brief 读设备流
 * @param handle 设备句柄
 * @param frameList 保存帧列表
 * @param size 列表大小
 * @return 读取到的帧数
 */
GpInt32_t gpReadStream(GpDeviceHandle handle, GpFrame **frame, GpInt32_t size, GpUInt32_t timeout, GpInt32_t type);

/**
 * @brief 获取设备信息
 * @param handle 设备句柄
 * @param info 输出设备信息
 * @return GP_EC_*
 */
GpInt32_t gpGetDeviceInfo(GpDeviceHandle handle, GpDeviceInfo *info);

/**
 * @brief 获取客户端存储在设备上的少量私有数据
 * @param handle 设备句柄
 * @param data 私有数据
 */
GpInt32_t gpGetDevicePrivateData(GpDeviceHandle handle, GpDevicePrivateData *data);

/**
 * @brief 存储少量私有数据到设备
 * @param handle 设备句柄
 * @param data 私有数据
 */
GpInt32_t gpSetDevicePrivateData(GpDeviceHandle handle, const GpDevicePrivateData *data);

/**
 * @brief 获取积分时间
 * @param handle 设备句柄
 * @param integraionTime 积分时间
 */
GpInt32_t gpGetIntegrationTime(GpDeviceHandle handle, GpUInt32_t *integraionTime);

/**
 * @brief 设置积分时间
 * @param handle 设备句柄
 * @param integraionTime 积分时间
 */
GpInt32_t gpSetIntegrationTime(GpDeviceHandle handle, GpUInt32_t integraionTime);

/**
 * @brief 获取镜头内部参数
 * @param handle 设备句柄
 * @param cameraLens 镜头内部参数
 */
GpInt32_t gpGetCameraLensParam(GpDeviceHandle handle, GpCameraLensParam *cameraLens);

/**
 * @brief 申请一个帧
 * @param size 帧大小
 */
GpFrame *gpAllocFrame(GpInt32_t size);

/**
 * @brief 释放帧 用于释放通过gpReadStream接口获取到的帧数据
 * @param frame
 */
void gpReleaseFrame(GpFrame *frame);

/**
 * @brief 获取镜头内部参数
 * @param handle 设备句柄
 * @param cameraLens 镜头内部参数
 */
 GpInt32_t gpRatingDevice(GpDeviceHandle handle, Point3D *robotCoor, ParamTRigidTrans3D *TRigidTrans3D);

/**
 * @brief 获取SDK版本信息
 * @param handle 设备句柄
 * @param pVer 版本信息
 */
 GpInt32_t gpSdkVersion(GpDeviceHandle handle, char* pVer);

#if defined(__cplusplus)
}
#endif
#endif
