

#define SP_LIDAR_FRAME_HEIGHT        287
#define SP_LIDAR_FRAME_WIDTH         352
#define SP_LIDAR_FRAME_NUM_POINTS 101376



typedef struct
{
  float x;
  float y;
  float z;
  unsigned int color;   //RGBA8 can be all zero when not needed
} SP_LIDAR_POINT;


typedef struct
{
  long long      timeStamp;
  int            frameNum;
  SP_LIDAR_POINT points[SP_LIDAR_FRAME_NUM_POINTS];
} SP_LIDAR_3D_DATA;


typedef struct
{
  long long timeStamp;
  int       frameNum;
  float     distance[SP_LIDAR_FRAME_NUM_POINTS];
  float     intensity[SP_LIDAR_FRAME_NUM_POINTS];
  float     amplitude[SP_LIDAR_FRAME_NUM_POINTS];
} SP_LIDAR_2D_DATA;



// Sense Photonics private below here

#define SP_LIDAR_RAW_MAX_SUBFRAME_COUNT   25
#define SP_LIDAR_RAW_DATA_PER_FRAME   101024
#define SP_LIDAR_RAW_MAX_DATA        (SP_LIDAR_RAW_MAX_SUBFRAME_COUNT * SP_LIDAR_RAW_DATA_PER_FRAME)

#define SP_LIDAR_RAW_MAX_MOD_FREQUENCIES  10
#define SP_LIDAR_RAW_MAX_EXPOSURE_TIMES   10
#define SP_LIDAR_RAW_MAX_PHASE_ANGLES     10
