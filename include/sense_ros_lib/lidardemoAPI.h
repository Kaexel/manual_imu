

#define SP_INVALID_LIDAR_ID         -101
#define SP_INVALID_STREAM_ID        -102
#define SP_2D_DATA_NOT_REQUESTED    -103
#define SP_3D_DATA_NOT_REQUESTED    -104
#define SP_RGB_DATA_NOT_REQUESTED   -105
#define SP_RAW_DATA_NOT_REQUESTED   -106
#define SP_EVENT_DATA_NOT_REQUESTED -107
#define SP_NO_DATA_REQUESTED        -108
#define SP_IP_ADDR_ALREADY_IN_USE   -109
#define SP_INVALID_IP_ADDR          -110
#define SP_STREAM_STILL_ACTIVE      -111
#define SP_UNKNOWN_ERROR            -112



/**************************************************************************/
/* SP_initLidar                                                           */
/*                                                                        */
/* Input Param:  ip address of the lidar in the form "192.0.10.162"       */
/*               if ip address is the same as an already initted lidar,   */
/*               it is an error                                           */
/* Output Param: lidar id value to use in subsequent calls                */
/* Return value: 0 on success, < 0 if failure                             */
/*                                                                        */
/**************************************************************************/
extern int SP_initLidar (char *ipAddress, int *lidarId);


/**************************************************************************/
/* SP_startStream                                                         */
/*                                                                        */
/* Input params: lidar id value from SP_initLidar                         */
/*               4 data request params (1 or 0) indicating what type of   */
/*               data that you want. More than one can be 1, at least     */
/*               one must be 1.                                           */
/* Output Param: stream id value to use in subsequent calls               */
/* Return value: 0 on success, < 0 if failure                             */
/*                                                                        */
/**************************************************************************/
extern int SP_startStream (int lidarId, int request2dData, int request3dData, int requestRgbData,
			   int requestEventData, int requestRawData, int *streamId);


/**************************************************************************/
/* SP_get2dData                                                           */
/*                                                                        */
/* Input params: stream id value from SP_startStream                      */
/*               pointer to your buffer to hold a SP_LIDAR_2D_DATA record */
/* Return value: success == 0, failure < 0                                */
/*                                                                        */
/* If this call is made on a stream where 2d data was not requested when  */
/* the stream was started, or the stream id is invalid, then it returns   */
/* failure status immediately, otherwise it blocks until a 2d data record */
/* is available.                                                          */
/*                                                                        */
/**************************************************************************/
extern int SP_get2dData (int streamId, SP_LIDAR_2D_DATA *dataFrame2d);


/**************************************************************************/
/* SP_get3dData                                                           */
/*                                                                        */
/* Input params: stream id value from SP_startStream                      */
/*               pointer to your buffer to hold a SP_LIDAR_3D_DATA record */
/* Return value: success == 0, failure < 0                                */
/*                                                                        */
/* If this call is made on a stream where 3d data was not requested when  */
/* the stream was started, or the stream id is invalid, then it returns   */
/* failure status immediately, otherwise it blocks until a 3d data record */
/* is available.                                                          */
/*                                                                        */
/**************************************************************************/
extern int SP_get3dData (int streamId, SP_LIDAR_3D_DATA *dataFrame3d);


/**************************************************************************/
/* SP_stopStream                                                          */
/*                                                                        */
/* Input params: stream id value from SP_initStream                       */
/* Return value: success == 0, failure < 0                                */
/*                                                                        */
/* Stops stream and invalidates the stream id, but does not invalidate    */
/* the associated lidar id. Another stream can be started later with the  */
/* same lidar id using the same or different data request values.         */
/*                                                                        */
/**************************************************************************/
extern int SP_stopStream (int streamId);


/**************************************************************************/
/* SP_terminateLidar                                                      */
/*                                                                        */
/* Input params: lidar id value from SP_initLidar                         */
/* Return value: success == 0, failure < 0                                */
/*                                                                        */
/* Terminates lidar session, releases resources and invalidates lidar id, */
/* and any outstanding associated stream ids.                             */
/*                                                                        */
/**************************************************************************/
extern int SP_terminateLidar (int lidarId);
