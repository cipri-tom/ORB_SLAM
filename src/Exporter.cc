#include <string>
using namespace std;

#include <opencv2/core/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Exporter.h"
#include "Converter.h"


namespace ORB_SLAM {

class KeyFrame;
class MapPoint;
class Converter;

Exporter::Exporter(Map *map): map_view(map)
{
}

void Exporter::WriteKeyFrames(const char *p_) const
{
    string path;
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    if (!p_)
        path = ros::package::getPath("ORB_SLAM") + "/"
             + "KeyFrameTrajectory_"
             + boost::posix_time::to_simple_string(now)
             + ".txt";
    else
        path = p_;

    string msg = "Writing Key Frames to file " + path;
    ROS_INFO("%s", msg.c_str());  // not recorded if ! ros.ok()
    cout << msg << '\n';
    ofstream f(path.c_str());

    vector<KeyFrame*> vpKFs = map_view->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    f << fixed;
    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
}


void Exporter::WriteMapPointsTXT(const char *p_) const
{
    string path;
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    if (!p_)
        path = ros::package::getPath("ORB_SLAM") + "/"
             + "MapPoints_"
             + boost::posix_time::to_simple_string(now)
             + ".txt";
    else
        path = p_;

    string msg = "Writing Key Frames to file " + path;
    ROS_INFO("%s", msg.c_str());  // not recorded if ! ros.ok()
    cout << msg << '\n';
    ofstream f(path.c_str());

    f << fixed;
    vector<MapPoint*> vMPs = map_view->GetAllMapPoints();
    for (vector<MapPoint*>::iterator point = vMPs.begin(); point != vMPs.end(); ++point) {
        if ((*point)->isBad())
            continue;
        cv::Mat pos = (*point)->GetWorldPos();
        cv::Mat nor = (*point)->GetNormal();
        cv::Vec3b colour = (*point)->GetColorInRefKF();

          // POSITION
        f << setprecision(6) << pos.at<float>(0) << ' '
          << setprecision(6) << pos.at<float>(1) << ' '
          << setprecision(6) << pos.at<float>(2) << ' '
          // NORMAL
          << setprecision(6) << nor.at<float>(0) << ' '
          << setprecision(6) << nor.at<float>(1) << ' '
          << setprecision(6) << nor.at<float>(2) << ' '
          // RGB
          << (int)colour[2] << ' '
          << (int)colour[1] << ' '
          << (int)colour[0] << ' '
          // REFLECTANCE (not relevant but needed for MeshLab import)
          << 1 << '\n';
    }
    f.close();
}

} // namespace ORB_SLAM
