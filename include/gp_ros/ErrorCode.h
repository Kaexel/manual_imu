#ifndef __ERROR_CODE_H__
#define __ERROR_CODE_H__
#if defined(__cplusplus)
extern "C" {
#endif

enum
{
    GP_EC_OK = 0,
    GP_EC_ERROR = -1,
    GP_EC_TIMEOUT = -2,
    GP_EC_DISCONNECTED = -3,
    GP_EC_MALLOC_ERROR = -4,
    GP_EC_STARTSTREAM_ERROR = -5,
    GP_EC_FILE_NOT_EXIT = -6,
    GP_EC_RATING_FAILED = -7,
};
#if defined(__cplusplus)
}
#endif

#endif