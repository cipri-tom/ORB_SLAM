#include <string>
using namespace std;

#include <opencv2/core/core.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <ros/assert.h>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

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

string Exporter::GetPath(const char *p, const char *base, const char *ext) const
{
    ROS_ASSERT_MSG(!p && !base && string(base).size() > 0, "Cannot infer path");

    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    string path = p ? p : ros::package::getPath("ORB_SLAM")
                            + "/" + base + "_"
                            + boost::posix_time::to_simple_string(now)
                            + "." + ext;
    ROS_INFO("%s", ("Writing file " + path).c_str());
    return path;
}

void Exporter::WriteKeyFrames(const char *p) const
{
    ofstream f(GetPath(p, "KeyFrameTrajectory").c_str());
    f << fixed;

    vector<KeyFrame*> vpKFs = map_view->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    for(size_t i=0; i<vpKFs.size(); i++) {
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


void Exporter::WriteMapPointsTXT(const char *p) const
{
    ofstream f(GetPath(p, "MapPoints").c_str());
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
          << 255 << '\n';
    }
    f.close();
}

void Exporter::WriteMapPointsPLY(const char *p) const
{
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointXYZRGBNormal pt;

    vector<MapPoint*> vMPs = map_view->GetAllMapPoints();
    for (vector<MapPoint*>::iterator point = vMPs.begin(); point != vMPs.end(); ++point) {
        if ((*point)->isBad())
            continue;
        cv::Mat pos = (*point)->GetWorldPos();
        cv::Mat nor = (*point)->GetNormal();
        cv::Vec3b colour = (*point)->GetColorInRefKF();

        // POSITION
        pt.x = pos.at<float>(0);
        pt.y = pos.at<float>(1);
        pt.z = pos.at<float>(2);
        // NORMAL
        pt.normal[0] = nor.at<float>(0);
        pt.normal[1] = nor.at<float>(1);
        pt.normal[2] = nor.at<float>(2);
        // RGB
        pt.r = colour[2];
        pt.g = colour[1];
        pt.b = colour[0];
        // CURVATURE -- not relevant, interpreted as REFLECTANCE
        pt.curvature = 1.0;

        cloud->points.push_back(pt);
    }

    string path = GetPath(p, "MapPoints", "ply");
    pcl::PLYWriter writer;
    writer.write(path, *cloud);
}


} // namespace ORB_SLAM
