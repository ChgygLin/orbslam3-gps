/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h> 
#include "gps-data.h"

namespace ORB_SLAM3
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

double crossArea(const cv::Point2d &pt1, const cv::Point2d &pt2, const cv::Point2d &pt3)
{
    cv::Vec3d a(pt1.x, pt1.y, 0);
    cv::Vec3d b(pt2.x, pt2.y, 0);
    cv::Vec3d c(pt3.x, pt3.y, 0);
    cv::Vec3d df0 = b-a;
    cv::Vec3d df1 = c-a;
    double tri_area = cv::norm(df0.cross(df1))/2;//叉积是个向量，二范数的模就是围成的平行四边形的面积
    return tri_area;
}

cv::Vec3d calcWeightByArea(const cv::Point2d &pt, const cv::Point2d &pt1, const cv::Point2d &pt2, const cv::Point2d &pt3)
{
    double S = crossArea(pt1, pt2, pt3);
	double S1 = crossArea(pt2, pt3, pt);
	double S2 = crossArea(pt1, pt3, pt);
	double S3 = crossArea(pt1, pt2, pt);

    cv::Vec3d weight(S1/S, S2/S, S3/S);
    return weight;
}

//判断点在三角形中
double Crossproduct(double a,double b,double c,double d)
{  //向量叉乘
    return a*d-b*c;
}

bool isPtInTriangle(const cv::Point2d &pt, const cv::Point2d &pt1, const cv::Point2d &pt2, const cv::Point2d &pt3)
{
    double x = pt.x;
    double x1 = pt1.x;
    double x2 = pt2.x;
    double x3 = pt3.x;
    double y = pt.y;
    double y1 = pt1.y;
    double y2 = pt2.y;
    double y3 = pt3.y;

    //保证点是逆时针输入
    if(Crossproduct(x3-x1,y3-y1,x2-x1,y2-y1)>=0){
        double t=x2;
        double t2=y2;//将其调整为逆时针
        x2=x3;
        y2=y3;
        x3=t;
        y3=t2;
    }
    if(Crossproduct(x2-x1,y2-y1,x-x1,y-y1)<0){
        return false;
    }
    if(Crossproduct(x1-x3,y1-y3,x-x3,y-y3)<0){
        return false;
    }
    if(Crossproduct(x3-x2,y3-y2,x-x2,y-y2)<0){
        return false;
    }
    return true;
}

cv::Point3d gps_cur, gps_last;
int gpos = 0;
string showPname_cur = "";
string showPname_last = "";
std::mutex mfileMutex;

static string path;
static string HotelPic;

void getReferFromFile(vector<vector<int>> &vve, string fname)
{
    std::unique_lock<std::mutex> lock(mfileMutex);
    ifstream f;
    f.open(fname.c_str());
    if (f.is_open())
    {
        while(!f.eof())
        {
            vector<int> ve;
            string line, tmp;
            getline(f,line);
            if(!line.empty())
            {
                stringstream ss;
                ss << line;
                while (getline(ss, tmp, ',')) {
                    ve.push_back(stoi(tmp));
                    //cout << tmp << endl;
                }
                ss.clear();
                vve.push_back(ve);
            }
        }
        f.close();
    }
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
//用于在两个图中相同的点双击，记录像素点位置和对应的gps序号.
void MouseEvent(int event, int x, int y, int flags, void *data)
{
	double value;
    cv::Point2d pt(x, y);

    //查找三角形并插值计算
    vector<cv::Vec6f> triangleList;

    FrameDrawer* frameDrawer = (FrameDrawer*)data;
	switch (event)
	{
        case cv::EVENT_LBUTTONDBLCLK:
        {
            std::unique_lock<std::mutex> lock(frameDrawer->mSubdivMutex);
            cout << "left-dclick, x=" << x << ", y=" << y << endl;   

            frameDrawer->mSubdiv.getTriangleList(triangleList);

            //如果在三角中则计算3d坐标
            for (size_t i = 0; i < triangleList.size(); i++)
            {
                cv::Vec6f t = triangleList[i];
                cv::Point2f pt1(t[0],t[1]);
                cv::Point2f pt2(t[2],t[3]);
                cv::Point2f pt3(t[4],t[5]);

                cv::Point2f tmp(0, 0);

                //确定目标是否在三角形中
                cv::Vec3d weight;
                if (isPtInTriangle(pt, pt1, pt2, pt3) == true) 
                {
                    int id1 = frameDrawer->mSubdiv.findNearest(pt1, &tmp) - 4;
                    int id2 = frameDrawer->mSubdiv.findNearest(pt2, &tmp) - 4;
                    int id3 = frameDrawer->mSubdiv.findNearest(pt3, &tmp) - 4;

                    //插值计算3d坐标,pt1,pt2,pt3的3d坐标
                    //weight = calcWeightByCoordinate(pt, pt1, pt2, pt3);
                    weight = calcWeightByArea(pt, pt1, pt2, pt3);
                    //pt_3d = weight[0] * pt1_3d + weight[1] * pt2_3d + weight[2] * pt3_3d;
                    
                    Eigen::Vector3d p1 = frameDrawer->mvCurrent3DP[id1];
                    Eigen::Vector3d p2 = frameDrawer->mvCurrent3DP[id2];
                    Eigen::Vector3d p3 = frameDrawer->mvCurrent3DP[id3];
                    Eigen::Vector3d p = weight[0]*p1 + weight[1]*p2 + weight[2]*p3;
                    cv::Point3d cgps = ECEF2LLA(cv::Point3d(p(0), p(1), p(2)));
                    //cout<< "cgps=" << setprecision(10) << cgps << ",gps_cur=" << gps_cur << ",gps_last=" << gps_last << endl;
                    cout<< "cgps=" << setprecision(10) << cgps << ",gps_cur=" << gps_cur << endl;
                    //cout<< "showPname_cur=" << showPname_cur << ",showPname_last=" << showPname_last << endl;
                    if (gps_cur.x > 0 && gps_cur.y > 0 && gps_cur.z > 0) {
                        double dist = GetDistance(gps_cur, cgps);
                        cout << "Distance=" << dist << "m" << endl;
                        //if (flags == cv::EVENT_FLAG_CTRLKEY && gps_cur != gps_last && showPname_cur != showPname_last) {
                        if (flags == cv::EVENT_FLAG_CTRLKEY) {
                            string fname = path + frameDrawer->mCurrentFrame.mNameFile + ".txt";
                            //cout << "fname=" << fname << endl;
                            vector<vector<int>> vve;
                            getReferFromFile(vve, fname);
                            std::unique_lock<std::mutex> lock(mfileMutex);
                            ofstream f;
                            //cout << "fname=" << fname << endl;
                            
                            f.open(fname.c_str(), ios_base::out|ios_base::trunc);
                            if (f.is_open()) {
                                //cout << "opened+++" << endl;
                                if (vve.size() > 0) {
                                    //cout << "vve.size()=" << vve.size() << endl;
                                    for (int i = 0; i < vve.size(); i++) {
                                        vector<int> ve = vve[i];
                                        if (gpos+1 == ve[0]) {
                                            ve[1] = x;
                                            ve[2] = y;
                                            x = y = -1;
                                            cout << "override!!!!!!!" << endl;
                                            cout << "Save: " << to_string(ve[0])+","+to_string(ve[1])+","+to_string(ve[2])<< endl;
                                        }
                                        f << to_string(ve[0])+","+to_string(ve[1])+","+to_string(ve[2])<< endl;
                                    }
                                    if (x >= 0 && y >= 0) {
                                        cout << "Save: " << to_string(gpos+1)+","+to_string(x)+","+to_string(y)<< endl;
                                        f << to_string(gpos+1)+","+to_string(x)+","+to_string(y)<< endl;
                                    }
                                } else {
                                    cout << "Save: " << to_string(gpos+1)+","+to_string(x)+","+to_string(y)<< endl;
                                    f << to_string(gpos+1)+","+to_string(x)+","+to_string(y)<< endl;
                                }
                                f.close();
                            }
                            //gps_last = gps_cur;
                        }
                    }
                    break;
                }
            }
        }
        break;
	}
}

int findGpsIndex(int x, int y)
{
	for (int i=0; i<67; i++)
	{
		if (x >= gps_d[i][0]-delta && x < gps_d[i][0]+delta && y >=gps_d[i][1]-delta && y < gps_d[i][1]+delta && i != 26)
			return i;
	}
	return -1;
}

void MouseEvent2(int event, int x, int y, int flags, void *data)
{
	int pos;
	switch (event)
	{
		case cv::EVENT_LBUTTONDBLCLK:
			//cout << "left dclick, x=" << x << ", y=" << y << endl;
			pos = findGpsIndex(x,y);
			if (pos < 0) {
				cout << "Invalid gps pos" << endl;
			} else {
                gpos = pos;
				cout << "gps pos: " << pos+1 << ", (lat, lon, alt)=(" << gps_d[pos][2] << gps_d[pos][3] << gps_d[pos][4] << ")" << endl;
                gps_cur.x = (double)gps_d[pos][2];
                gps_cur.y = (double)gps_d[pos][3];
                gps_cur.z = (double)gps_d[pos][4];
			}
			break;
	}
}

double calcDistance(FrameDrawer* frameDrawer, int gps_index, int piexl_x, int piexl_y)
{
    cv::Point2d pt(piexl_x, piexl_y);
    //查找三角形并插值计算
    vector<cv::Vec6f> triangleList;
    std::unique_lock<std::mutex> lock(frameDrawer->mSubdivMutex);
    cv::Point3d gps_true;
    gps_true.x = (double)gps_d[gps_index][2];
    gps_true.y = (double)gps_d[gps_index][3];
    gps_true.z = (double)gps_d[gps_index][4];

    frameDrawer->mSubdiv.getTriangleList(triangleList);

    //如果在三角中则计算3d坐标
    for (size_t i = 0; i < triangleList.size(); i++)
    {
        cv::Vec6f t = triangleList[i];
        cv::Point2f pt1(t[0],t[1]);
        cv::Point2f pt2(t[2],t[3]);
        cv::Point2f pt3(t[4],t[5]);

        cv::Point2f tmp(0, 0);

        //确定目标是否在三角形中
        cv::Vec3d weight;
        if (isPtInTriangle(pt, pt1, pt2, pt3) == true) 
        {
            int id1 = frameDrawer->mSubdiv.findNearest(pt1, &tmp) - 4;
            int id2 = frameDrawer->mSubdiv.findNearest(pt2, &tmp) - 4;
            int id3 = frameDrawer->mSubdiv.findNearest(pt3, &tmp) - 4;

            //插值计算3d坐标,pt1,pt2,pt3的3d坐标
            //weight = calcWeightByCoordinate(pt, pt1, pt2, pt3);
            weight = calcWeightByArea(pt, pt1, pt2, pt3);
            //pt_3d = weight[0] * pt1_3d + weight[1] * pt2_3d + weight[2] * pt3_3d;
            
            Eigen::Vector3d p1 = frameDrawer->mvCurrent3DP[id1];
            Eigen::Vector3d p2 = frameDrawer->mvCurrent3DP[id2];
            Eigen::Vector3d p3 = frameDrawer->mvCurrent3DP[id3];
            Eigen::Vector3d p = weight[0]*p1 + weight[1]*p2 + weight[2]*p3;
            cv::Point3d cgps = ECEF2LLA(cv::Point3d(p(0), p(1), p(2)));
            //cout<< setprecision(10) << cgps <<endl;
            if (gps_true.x > 0 && gps_true.y > 0 && gps_true.z > 0) {
                double dist = GetDistance(gps_true, cgps);
                //cout << "Distance=" << dist << "m" << endl;
                return dist;
            }
        }
    }
    return -1;
}
//用于将指定像素点对应的gps误差显示出来。读文件得到指定像素点和对应的gps序号，算gps以及误差，显示。
cv::Mat DrawFrame3(FrameDrawer* frameDrawer, cv::Mat &im)
{
    vector<vector<int>> vve;
    string fname = path + frameDrawer->mCurrentFrame.mNameFile + ".txt";
    getReferFromFile(vve, fname);

    for ( int i = 0; i < vve.size(); i++ ) {
        vector<int> ve = vve[i];
        double distance = calcDistance(frameDrawer, ve[0]-1, ve[1], ve[2]);
        cv::Point2d pt(ve[1], ve[2]);
        cv::circle(im, pt, 2, cv::Scalar(0,0,255), -1);
        cv::putText(im, to_string(distance), cv::Point(ve[1], ve[2]), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0,0,255), 1, 8);
    }
    return im;
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStop("menu.Stop",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);

    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    cv::namedWindow("ORB-SLAM3: Current Frame");
    cv::namedWindow("3D Point");

    cv::setMouseCallback("3D Point", MouseEvent, mpFrameDrawer);

    if(NULL == opendir(path.c_str()))
		mkdir(path.c_str(), 0755);
    
    path = mpSystem->mImgRootPath + "/reference/";
    HotelPic = mpSystem->mRootPath + "/jd.png";

    cv::namedWindow("Hotel");
    // cout<<"############# HotelPic "<<HotelPic<<endl;
    // cout<<"############# path "<<path<<endl;
    cv::Mat frame = cv::imread(HotelPic, cv::IMREAD_UNCHANGED);
    cv::setMouseCallback("Hotel", MouseEvent2, NULL);

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->GPS_MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    float trackedImageScale = mpTracker->GetImageScale();

    cout << "Starting the Viewer" << endl;
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);

        if(mbStopTrack)
        {
            menuStepByStep = true;
            mbStopTrack = false;
        }

        if(menuFollowCamera && bFollow)
        {
            if(bCameraView)
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if(bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
        }

        if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
            s_cam.Follow(Ow);
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(menuStepByStep && !bStepByStep)
        {
            //cout << "Viewer: step by step" << endl;
            mpTracker->SetStepByStep(true);
            bStepByStep = true;
        }
        else if(!menuStepByStep && bStepByStep)
        {
            mpTracker->SetStepByStep(false);
            bStepByStep = false;
        }

        if(menuStep)
        {
            mpTracker->mbStep = true;
            menuStep = false;
        }


        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc); 
        if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph, menuShowOptLba);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat toShow;
        cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

        // 添加一个自定义图,显示所有的关键帧投影的特征点
        cv::Mat toShow2;
        
        cv::Mat im2 = mpFrameDrawer->DrawFrame2(trackedImageScale);
        im2 = DrawFrame3(mpFrameDrawer, im2);
        if (showPname_cur != mpFrameDrawer->mCurrentFrame.mNameFile) {
            showPname_last = showPname_cur;
            showPname_cur = mpFrameDrawer->mCurrentFrame.mNameFile;
            gps_cur = cv::Point3d(0,0,0);
            //gps_last = cv::Point3d(0,0,0);
        }

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }

        if(mImageViewerScale != 1.f)
        {
            int width = toShow.cols * mImageViewerScale;
            int height = toShow.rows * mImageViewerScale;
            cv::resize(toShow, toShow, cv::Size(width, height));

            cv::resize(im2, im2, cv::Size(width, height));
        }

        cv::imshow("ORB-SLAM3: Current Frame",toShow);
        cv::imshow("3D Point", im2);
        cv::imshow("Hotel", frame);

        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->ResetActiveMap();
            menuReset = false;
        }

        if(menuStop)
        {
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();

            // Stop all threads
            mpSystem->Shutdown();

            // Save camera trajectory
            mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
            mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
            menuStop = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
