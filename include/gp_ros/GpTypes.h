#ifndef __GP_TYPES_H__
#define __GP_TYPES_H__

#if defined(__cplusplus)
extern "C" {
#endif

typedef signed char GpInt8_t;
typedef unsigned char GpUInt8_t;

typedef short GpInt16_t;
typedef unsigned short GpUInt16_t;

typedef int GpInt32_t;
typedef unsigned int GpUInt32_t;

typedef long long GpInt64_t;
typedef unsigned long long GpUInt64_t;

typedef enum GpBool
{
    FALSE,
    TRUE,
}GpBool;

#if defined(__cplusplus)
}
#endif

#endif