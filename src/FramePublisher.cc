/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FramePublisher.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <boost/thread.hpp>


bool IsCloseTo(const cv::Point2f &p, const int click[2])
{
    int r = 5;
    return click[0]-r <= p.x && p.x <= click[0]+r
       &&  click[1]-r <= p.y && p.y <= click[1]+r;
}

namespace ORB_SLAM
{

FramePublisher::FramePublisher()
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
    mbUpdated = true;
    mLeftClick[0] = mLeftClick[1] = mRightClick[0] = mRightClick[1] = -1;

    mImagePub = mNH.advertise<sensor_msgs::Image>("ORB_SLAM/Frame",10,true);
    mClickSub = mNH.subscribe("/clicks", 10, &FramePublisher::ProcessClick, this);

    PublishFrame();
}

void FramePublisher::ProcessClick(const std_msgs::String &msg)
{
    istringstream msg_data(msg.data), new_dist_text;
    int xleft, yleft, xright, yright;
    msg_data >> xleft >> yleft >> xright >> yright;
    if (xleft < 0 && yleft < 0 && xright < 0 && yright < 0) // RESET
        mLeftClick[0] = mLeftClick[1] = mRightClick[0] = mRightClick[1] = -1;
    else if (xleft >= 0 && yleft >= 0) {  // only left
        mLeftClick[0] = xleft;
        mLeftClick[1] = yleft;
    }
    else if (xright >= 0 && yright >= 0) { // only right
        mRightClick[0] = xright;
        mRightClick[1] = yright;
    }
    else                                  // can't be both
        ROS_WARN("INVALID CLICK: %s", msg.data.c_str());
    cout << "Current clicks: " << mLeftClick[0]  << ' ' << mLeftClick[1]  << "  "
                               << mRightClick[0] << ' ' << mRightClick[0] << '\n';
}

void FramePublisher::SetMap(Map *pMap)
{
    mpMap = pMap;
}

void FramePublisher::Refresh()
{
    if(mbUpdated)
    {
        PublishFrame();
        mbUpdated = false;
    }
}

cv::Mat FramePublisher::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<MapPoint*> vMatchedMapPoints; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variable to be used within scoped mutex
    {
        boost::mutex::scoped_lock lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vIniKeys = mvIniKeys;
        }
        else if(mState==Tracking::INITIALIZING)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::WORKING)
        {
            vCurrentKeys = mvCurrentKeys;
            vMatchedMapPoints = mvpMatchedMapPoints;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release

    if(im.channels()<3)
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::INITIALIZING) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::WORKING) //TRACKING
    {
        mnTracked=0;
        const float r = 5;
        for(unsigned int i=0;i<vMatchedMapPoints.size();i++)
        {
            if(vMatchedMapPoints[i] || mvbOutliers[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                if(!mvbOutliers[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
            }
        }

    }

    cv::Mat imWithStatus, imWithInfo;
    string status   = GetStatusText(state);
    string distance = GetDistText(vCurrentKeys, vMatchedMapPoints, state);
    DrawTextInfo(im          ,status  , imWithStatus);
    DrawTextInfo(imWithStatus,distance, imWithInfo);

    return imWithInfo;
}

void FramePublisher::PublishFrame()
{
    cv::Mat im = DrawFrame();
    cv_bridge::CvImage rosImage;
    rosImage.image = im.clone();
    rosImage.header.stamp = ros::Time::now();
    rosImage.encoding = "bgr8";

    mImagePub.publish(rosImage.toImageMsg());
    ros::spinOnce();
}

string FramePublisher::GetStatusText(int nState) const
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << "WAITING FOR IMAGES. (Topic: /camera/image_raw)";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " NOT INITIALIZED ";
    else if(nState==Tracking::INITIALIZING)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::WORKING)
    {
        s << " TRACKING ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << " - KFs: " << nKFs << " , MPs: " << nMPs << " , Tracked: " << mnTracked;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    return s.str();
}

string FramePublisher::GetDistText(const vector<cv::KeyPoint> &vCurrentKeys,
                                   const vector<MapPoint*>    &vMatchedMapPoints,
                                   int state) const
{
    if (state != Tracking::WORKING)
        return string("[unavailable]");

    int idxA = -1, idxB = -1;
    if (vCurrentKeys.size() != vMatchedMapPoints.size())
        ROS_ERROR("SIZES DIFFER!");
    for (size_t i = 0; i < vCurrentKeys.size(); ++i) {
        if (!vMatchedMapPoints[i])
            continue;
        if (IsCloseTo(vCurrentKeys[i].pt, mLeftClick))
            idxA = i;
        if (IsCloseTo(vCurrentKeys[i].pt, mRightClick))
            idxB = i;
    }
    double dist = -1.0;
    if (idxA >= 0 && idxB >= 0)
        dist = cv::norm(vMatchedMapPoints[idxA]->GetWorldPos(),
                        vMatchedMapPoints[idxB]->GetWorldPos());

    ostringstream s;
    s << fixed;
    s << "Point A(" << setprecision(3) << vCurrentKeys[idxA].pt.x << ','
                    << setprecision(3) << vCurrentKeys[idxA].pt.y << "); "
      << "Point B(" << setprecision(3) << vCurrentKeys[idxB].pt.x << ','
                    << setprecision(3) << vCurrentKeys[idxB].pt.y << ". "
      << "AB=";
    if (dist < 0) s << "[unavailable]";
    else          s << dist;

    return s.str();
}

void FramePublisher::DrawTextInfo(cv::Mat &im, string text, cv::Mat &imText)
{
    int baseline=0;
    cv::Size textSize = cv::getTextSize(text,cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,text,cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FramePublisher::Update(Tracking *pTracker)
{
    boost::mutex::scoped_lock lock(mMutex);
    pTracker->mCurrentFrame.im.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    mvpMatchedMapPoints=pTracker->mCurrentFrame.mvpMapPoints;
    mvbOutliers = pTracker->mCurrentFrame.mvbOutlier;

    if(pTracker->mLastProcessedState==Tracking::INITIALIZING)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);

    mbUpdated=true;
}

} //namespace ORB_SLAM
