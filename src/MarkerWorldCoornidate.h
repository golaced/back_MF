#ifndef __MarkerWorldCoornidate_h__
#define __MarkerWorldCoornidate_h__

#include <opencv2/opencv.hpp>
#include <vector>
<<<<<<< HEAD
#include "inifile.h"
// #define MARKER_NEW 1
// #if MARKER_NEW //new for camera paper
// #define MARKER_ROW_NUM 10//hang
// #define MARKER_COL_NUM 10//lie
// #define MARKERS_ROW_DISTANCE 36.2//11.8	//cm
// #define MARKERS_COL_DISTANCE MARKERS_ROW_DISTANCE//12.2	//cm
// #define MARKER_SIZE 0.178 //0.052//m 20cm
// #define MARKER_SIZE_DRAW MARKER_SIZE*100//cm 20
// #else
// #define MARKER_ROW_NUM 6
// #define MARKER_COL_NUM 6
// #define MARKERS_ROW_DISTANCE 60.0	//cm
// #define MARKERS_COL_DISTANCE 50.0	//cm
// #define MARKER_SIZE 20//cm
// #endif

#define MARKER_CONFIG_FILENAME std::string("/home/pi/QT/MF/markerconfig.ini")

extern int marker_row_num;
extern int marker_col_num;
extern double marker_size;
extern double markers_row_distance;
extern double markers_col_distance;
extern int show_traj;
extern int camera_sel;
extern string cal_file;
#define MARKER_ROW_NUM marker_row_num
#define MARKER_COL_NUM marker_col_num
#define MARKERS_ROW_DISTANCE markers_row_distance
#define MARKERS_COL_DISTANCE markers_col_distance
#define MARKER_SIZE marker_size
void init_sys_para(void);
=======
#define MARKER_NEW 1
#if MARKER_NEW //new for camera paper
#define MARKER_ROW_NUM 10//hang
#define MARKER_COL_NUM 10//lie
#define MARKERS_ROW_DISTANCE 36.2//11.8	//cm
#define MARKERS_COL_DISTANCE MARKERS_ROW_DISTANCE//12.2	//cm
#define MARKER_SIZE 0.178 //0.052//m 20cm
#define MARKER_SIZE_DRAW MARKER_SIZE*100//cm 20
#else
#define MARKER_ROW_NUM 6
#define MARKER_COL_NUM 6
#define MARKERS_ROW_DISTANCE 60.0	//cm
#define MARKERS_COL_DISTANCE 50.0	//cm
#define MARKER_SIZE 20//cm
#endif

>>>>>>> origin/master
extern float mark_map[6][5];
class MarkerWorld
{
  public:
    MarkerWorld();
    MarkerWorld(int _id, cv::Point3f _coordinate);
    MarkerWorld(int _id, float _x, float _y, float _z);
    ~MarkerWorld();

    cv::Point3f coordinate;

    int id;

  private:
};

class MarkerWorldCoordinate
{
  public:
    MarkerWorldCoordinate();
    MarkerWorldCoordinate(size_t _size);
    ~MarkerWorldCoordinate();

    size_t size();
    bool setCoordinate(MarkerWorld mw);
    cv::Point3f getCoordinate(int _id);

  private:
    size_t m_size;
    std::vector<MarkerWorld> coorTable;
};

void initCoordinateTable(std::vector<MarkerWorld> &CoordinateTable);

#endif // __MarkerWorldCoornidate_h__
