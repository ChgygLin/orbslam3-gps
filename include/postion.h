
/*
1. 3d坐标与ECEF坐标转换矩阵
    a)GPS坐标与ECEF坐标互转

    b)参数拟合（s、R、T）


2. 定位

    a)有效关键帧、地图点查找

    b)地图点重投影，三角化

    c)确定目标所在三角形，插值计算3d坐标

    d)转ECEF、转GPS
*/

#ifndef POSTION_H
#define POSTION_H

#include<opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc_c.h>

#include <vector>

#define USE_LBA_GPS         // 局部建图中LBA融合GPS
#define USE_POSE_GPS        // 当前帧位姿估计融合GPS

typedef struct GeoTransform
{
    /* data */
    double s;
    Eigen::Matrix3d R;
    Eigen::Vector3d T;
}GeoTransform;

#define PI 3.1415926
//#define RAD2DEG(x) (x)*180.0/PI
//#define DEG2RAD(x) (x)*PI/180.0
static const double WGS84_A = 6378137.0;      // major axis
static const double WGS84_E = 0.0818191908;   // first eccentricity


static cv::Point3d ECEF2LLA(cv::Point3d cur_ecef) {
    double x = cur_ecef.x;
    double y = cur_ecef.y;
    double z = cur_ecef.z;
    const double b = sqrt(WGS84_A * WGS84_A * (1 - WGS84_E * WGS84_E));
    const double ep = sqrt((WGS84_A * WGS84_A - b * b) / (b * b));
    const double p = hypot(x, y);
    const double th = atan2(WGS84_A * z, b * p);
    const double lon = atan2(y, x);
    const double lat = atan2((z + ep * ep * b * pow(sin(th), 3)), (p - WGS84_E * WGS84_E * WGS84_A * pow(cos(th), 3)));
    const double N = WGS84_A / sqrt(1 - WGS84_E * WGS84_E * sin(lat) * sin(lat));
    const double alt = p / cos(lat) - N;

    return cv::Point3d(RAD2DEG(lat), RAD2DEG(lon) ,alt);
}


static cv::Point3d LLA2ECEF(cv::Point3d cur_gps) {
    double lat = cur_gps.x;
    double lon = cur_gps.y;
    double alt = cur_gps.z;

    double WGS84_f = 1 / 298.257223565;
    double WGS84_E2 = WGS84_f * (2 - WGS84_f);
    double deg2rad = M_PI / 180.0;
    //double rad2deg = 180.0 / M_PI;
    lat *= deg2rad;
    lon *= deg2rad;
    double N = WGS84_A / (sqrt(1 - WGS84_E2 * sin(lat) * sin(lat)));
    double x = (N + alt) * cos(lat) * cos(lon);
    double y = (N + alt) * cos(lat) * sin(lon);
    double z = (N * (1 - WGS84_f) * (1 - WGS84_f) + alt) * sin(lat);
    return cv::Point3d(x, y, z);
}


// 地球半径
static const double EARTH_RADIUS = 6371000;

// 1、计算两个经纬度之间的距离(m)
// double GetDistanceCpp(double lat1, double lng1, double lat2, double lng2)
static double GetDistance(cv::Point3d gps1, cv::Point3d gps2)
{
    double lat1 = gps1.x;
    double lng1 = gps1.y;

    double lat2 = gps2.x;
    double lng2 = gps2.y;

    double radLat1 = DEG2RAD(lat1);   // 角度转弧度
    double radLat2 = DEG2RAD(lat2);
    double a = radLat1 - radLat2;
    double b = DEG2RAD(lng1) - DEG2RAD(lng2);
    double s = 2 * asin(sqrt(pow(sin(a/2),2) +cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s * EARTH_RADIUS;
    return s;
}

static cv::Point3d GetDeltaGps(cv::Point3d gps1, cv::Point3d gps2)
{

    double lat1 = gps1.x;
    double lon1 = gps1.y;
    double alt1 = gps1.z;

    double lat2 = gps2.x;
    double lon2 = gps2.y;
    double alt2 = gps2.z;

    // 经度1秒 约等于 30.87*cos纬度（米）
    double delta_lon = abs(lon2-lon1)*3600*30.87*cos(lat1);

    // 纬度的1秒 = 30.8米
    double delta_lat = abs(lat2-lat1)*3600*30.8;

    double delta_alt = abs(alt2-alt1);

    return cv::Point3d(delta_lat, delta_lon, delta_alt);
}


template<typename ... Args>
static std::string str_format(const std::string &format, Args ... args)
{
	auto size_buf = std::snprintf(nullptr, 0, format.c_str(), args ...) + 1; 
	std::unique_ptr<char[]> buf(new(std::nothrow) char[size_buf]);

	if (!buf)
		return std::string("");

	std::snprintf(buf.get(), size_buf, format.c_str(), args ...);
	return std::string(buf.get(), buf.get() + size_buf - 1); 
}

#if 1
//画出剖分
static void drawSubdiv(cv::Mat &img, cv::Subdiv2D &subdiv, cv::Scalar delaunay_color)
{
    vector<cv::Vec6f> triangleList;
    subdiv.getTriangleList(triangleList);
    vector<cv::Point> pt(3);

    for (size_t i = 0; i < triangleList.size(); i++)
    {
        cv::Vec6f t = triangleList[i];
        pt[0] = cv::Point(cvRound(t[0]), cvRound(t[1]));
        pt[1] = cv::Point(cvRound(t[2]), cvRound(t[3]));
        pt[2] = cv::Point(cvRound(t[4]), cvRound(t[5]));
        cv::line(img, pt[0], pt[1], delaunay_color, 1, CV_AA, 0);
        cv::line(img, pt[1], pt[2], delaunay_color, 1, CV_AA, 0);
        cv::line(img, pt[2], pt[0], delaunay_color, 1, CV_AA, 0);
    }
}
#endif

/* -----------------------------------------------
    帧添加gps坐标成员变量

    
    所有的帧(pose, gps)， 现有或新建(buffer M=5F,  Jetson NX=50)
    所有的关键帧(pose, gps, 3d)

    1. 关键帧>3时，计算ecef_matrix      初步定为在localmapping主线程中
        相机pose参考pRefKF->GetCameraCenter()

    2. buffer满 且 ecef_matrix有效时，依次计算buffer帧的目标gps，读取ecef时需要加锁     初步定为tracking主线程中
        a.遍历所有关键帧，取相机gps较近的帧

        b.遍历相应帧的3d点，并投影到当前图像中
            参考SearchInNeighbors中的投影, 
            投影后是否在当前帧中，pKF->IsInImage(u,v)

        c.三角剖分
        d.确定目标所在三角形，插值计算3d坐标
        e.转ECEF、转GPS

ORBmatcher::SearchByProjection
// Project into Image
const Eigen::Vector2f uv = pKF->mpCamera->project(p3Dc);

// Point must be inside the image
if(!pKF->IsInImage(uv(0),uv(1)))
    continue;


 ----------------------------------------------- */





// -------------------------------------
// 与yolov5对接，航空数据集



#endif
