/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/
/**
* @file simple_single.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <string>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>


#include <boost/thread.hpp>
using namespace aruco;

struct MultiBoard {
  double markerSize;
  int boardId;
  std::vector<int> vMarkerId;
  std::vector<double> vMarkerX;
  std::vector<double> vMarkerY;
  std::vector<double> vMarkerR;
  std::vector<double> vMarkerG;
  std::vector<double> vMarkerB;
  std::vector<ros::Publisher> vMarkerPublisher; //rviz visualization marker
  std::unordered_map<int,int> markerIDToVectorPosition;
  std::vector<bool> vMarkerDetected;
  bool detected;
} ;

class AnyArucoMultiConcurrent
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  //MarkerDetector mDetector;
  vector<MarkerDetector> vDetector;
  vector<int> availableDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  std::string refinementMethod_;


  vector<int> vMultiBoardId_;
  vector<MultiBoard> vMultiBoard_;
  std::unordered_map<int,int> markerIDToBoardVectorPosition_;
  ros::Publisher markerArrayPublisher_; //rviz visualization marker
  vector<ros::Publisher> vPosePublisher_;

  boost::mutex myMutex_;
  boost::mutex detectorMutex_;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

public:
  AnyArucoMultiConcurrent()
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {
    readParameters();
    create();
    initilizePublishers();
    initilizeSubscribers();
    initilizeServices();
    initilize();
  }

  void readParameters()
  {
    nh.param<std::string>("corner_refinement", refinementMethod_, "LINES");

    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    //nh.param<double>("marker_size", marker_size, 0.09);
    //nh.param<int>("marker_id", marker_id, 300);

    nh.getParam("/aruco_multi/multi_boards/multi_board_id", vMultiBoardId_);

    std::cerr<< "Board IDs ";
    for(int id : vMultiBoardId_)
      std::cerr<< id <<"," ;
    std::cerr<<"\n";

    for(auto id : vMultiBoardId_){
      MultiBoard b;
      b.boardId = id;
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/marker_size", b.markerSize);
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/board_id", b.vMarkerId);
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/board_x", b.vMarkerX);
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/board_y", b.vMarkerY);
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/color_r", b.vMarkerR);
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/color_g", b.vMarkerG);
      nh.getParam("/aruco_multi/multi_boards/multi_board_"+std::to_string(id)+"/color_b", b.vMarkerB);
      std::cerr<< "Marker IDs ";
      for(int id :  b.vMarkerId)
        std::cerr<< id <<"," ;
      std::cerr<<"\n";
      vMultiBoard_.push_back(b);
    }


  }
  void create()
  {

  }
  void initilizePublishers()
  {
    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    for(int i=0; i<vMultiBoard_.size();i++){
      vPosePublisher_.push_back( nh.advertise<geometry_msgs::PoseStamped>("pose_"+std::to_string(i), 100)) ;
    }
    markerArrayPublisher_ = nh.advertise<visualization_msgs::MarkerArray>("markers", 1);

  }
  void initilizeSubscribers()
  {
    image_sub = it.subscribe("/image", 1, &AnyArucoMultiConcurrent::image_callback, this);

    cam_info_sub = nh.subscribe("/camera_info", 1, &AnyArucoMultiConcurrent::cam_info_callback, this);
  }
  void initilizeServices()
  {
  }
  void initilize()
  {
    for(int i = 0 ; i < 20 ; i ++){
      MarkerDetector detector;
      if ( refinementMethod_ == "SUBPIX" )
        detector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
      else if ( refinementMethod_ == "HARRIS" )
        detector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
      else if ( refinementMethod_ == "NONE" )
        detector.setCornerRefinementMethod(aruco::MarkerDetector::NONE);
      else
        detector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
      availableDetector.push_back(i);
      vDetector.push_back(detector);
    }


    // marker ID arrays
    for(int j=0;j<vMultiBoard_.size();j++){
      for(int i=0;i<vMultiBoard_[j].vMarkerId.size();i++){
        vMultiBoard_[j].markerIDToVectorPosition.insert({vMultiBoard_[j].vMarkerId[i],i});
        markerIDToBoardVectorPosition_.insert({vMultiBoard_[j].vMarkerId[i],j});
        vMultiBoard_[j].vMarkerDetected.push_back(false) ;
      }
      std::cerr<< "id to vector : ";
      for(int i=0;i<vMultiBoard_[j].vMarkerId.size();i++)
        std::cerr<< vMultiBoard_[j].markerIDToVectorPosition[i] <<"," ;
      std::cerr<<"\n";
      vMultiBoard_[j].detected = false;
    }

    ROS_ASSERT(camera_frame != "" && marker_frame != "");
    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    // bind
    dyn_rec_server.setCallback(boost::bind(&AnyArucoMultiConcurrent::reconf_callback, this, _1, _2));
  }
  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(0.5),
                                       ros::Duration(0.01),
                                       &errMsg)
         )
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame,
                                     ros::Time(0),                  //get latest available
                                     transform);
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    std::cerr << ros::Time::now() << std::endl;
    std::cerr << availableDetector.size() << std::endl;


    boost::thread threadAlpha(&AnyArucoMultiConcurrent::myThread,this,msg);
  }
  void myThread(const sensor_msgs::ImageConstPtr& msg){
    ros::Time threadStartTime(ros::Time::now());

    bool isSubscribed = false;

    for(auto& v :vPosePublisher_){
      isSubscribed |= v.getNumSubscribers() != 0;
    }
    isSubscribed |= markerArrayPublisher_.getNumSubscribers() != 0;
    if (!isSubscribed)
    {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }

    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {

      ros::Time curr_stamp(ros::Time::now());
      cv_bridge::CvImagePtr cv_ptr;
      try
      {

        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        //detection results will go into "markers"
        markers.clear();
        // Vector of vector of poses
        std::vector<std::vector<geometry_msgs::PoseStamped>> mPoseMsg;
        //Ok, let's detect



        MarkerDetector mDetector;
        mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
        try{
          mDetector.detect(inImage, markers, camParam, vMultiBoard_[0].markerSize, false);
        }
        catch (...){
          ROS_ERROR("\t CRAZY EXCEPTION ");
          return;
        }


        myMutex_.lock();
        for (auto b : vMultiBoard_){
          std::vector<geometry_msgs::PoseStamped> vPoseMsg(b.vMarkerId.size());
          mPoseMsg.push_back(vPoseMsg);
        }

        visualization_msgs::MarkerArray visMarkerArray;


        bool detected = false ;
        //for each marker, draw info and its boundaries in the image
        for(size_t i=0; i<markers.size(); ++i)
        {
          //check if the marker is in the one of the boards
          if(markerIDToBoardVectorPosition_.count(markers[i].id)>0){
            // get the index of the marker
            int bIndex = markerIDToBoardVectorPosition_[markers[i].id];
            int vIndex = vMultiBoard_[bIndex].markerIDToVectorPosition[markers[i].id];

            tf::Transform transform ;
            try{
              transform = aruco_ros::arucoMarker2Tf(markers[i]);
            }
            catch (cv::Exception& e){
              ROS_ERROR("\t CV EXCEPTION: %s", e.what());
              return;
            }

            tf::StampedTransform cameraToReference;
            cameraToReference.setIdentity();

            if ( reference_frame != camera_frame )
            {
              getTransform(reference_frame,
                           camera_frame,
                           cameraToReference);
            }
            transform =
              static_cast<tf::Transform>(cameraToReference)
              * static_cast<tf::Transform>(rightToLeft)
              * transform;

            tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                  reference_frame, marker_frame);
            //br.sendTransform(stampedTransform);
            geometry_msgs::TransformStamped transformMsg;
            tf::transformStampedTFToMsg(stampedTransform, transformMsg);

            // XXX : record poses of each marker
            tf::poseTFToMsg(transform, mPoseMsg[bIndex][vIndex].pose);

            visualization_msgs::Marker visMarker;
            visMarker.header = transformMsg.header;
            visMarker.id = markers[i].id;
            visMarker.type   = visualization_msgs::Marker::CUBE;
            visMarker.action = visualization_msgs::Marker::ADD;
            visMarker.pose = mPoseMsg[bIndex][vIndex].pose;
            visMarker.scale.x = vMultiBoard_[bIndex].markerSize;
            visMarker.scale.y = 0.001;
            visMarker.scale.z = vMultiBoard_[bIndex].markerSize;
            visMarker.color.a = 1.0;
            visMarker.lifetime = ros::Duration(3.0);
            visMarker.color.r = vMultiBoard_[bIndex].vMarkerR[vIndex];
            visMarker.color.b = vMultiBoard_[bIndex].vMarkerB[vIndex];
            visMarker.color.g = vMultiBoard_[bIndex].vMarkerG[vIndex];
            visMarkerArray.markers.push_back(visMarker);

            mPoseMsg[bIndex][vIndex].pose.position.x -= vMultiBoard_[bIndex].vMarkerX[vIndex] ;
            mPoseMsg[bIndex][vIndex].pose.position.z -= vMultiBoard_[bIndex].vMarkerY[vIndex] ;
            vMultiBoard_[bIndex].vMarkerDetected[vIndex] = true;
            vMultiBoard_[bIndex].detected |= true ;
            // but drawing all the detected markers
            markers[i].draw(inImage,cv::Scalar(0,0,255),2);
          }

        }

        // PUBLISH MarkerArray
        markerArrayPublisher_.publish(visMarkerArray);

        for(int j=0 ; j<vMultiBoard_.size() ;j++){

          if (vMultiBoard_[j].detected){
            // XXX : average the positions
            double numberOfDetectedMarker = 0.0;
            geometry_msgs::PoseStamped poseMsg;
            for(int i=0 ; i<vMultiBoard_[j].vMarkerId.size() ;i++){
              if(vMultiBoard_[j].vMarkerDetected[i]){
                poseMsg.pose.position.x += mPoseMsg[j][i].pose.position.x ;
                poseMsg.pose.position.y += mPoseMsg[j][i].pose.position.y ;
                poseMsg.pose.position.z += mPoseMsg[j][i].pose.position.z ;
                // TODO : Average the pose as well
                poseMsg.pose.orientation = mPoseMsg[j][i].pose.orientation ;
                numberOfDetectedMarker += 1.0 ;
              }
              vMultiBoard_[j].vMarkerDetected[i]=false;
            }
            vMultiBoard_[j].detected = false;

            poseMsg.pose.position.x /= numberOfDetectedMarker ;
            poseMsg.pose.position.y /= numberOfDetectedMarker ;
            poseMsg.pose.position.z /= numberOfDetectedMarker ;
            poseMsg.header.frame_id = reference_frame;
            poseMsg.header.stamp = curr_stamp;
            vPosePublisher_[j].publish(poseMsg);

            geometry_msgs::TransformStamped transformMsg;
            transformMsg.header = msg->header;
            transformMsg.header.stamp = curr_stamp;
            transformMsg.header.frame_id = reference_frame;
            transformMsg.child_frame_id = marker_frame+"_"+std::to_string(vMultiBoard_[j].boardId);
            transformMsg.transform.translation.x = poseMsg.pose.position.x ;
            transformMsg.transform.translation.y = poseMsg.pose.position.y ;
            transformMsg.transform.translation.z = poseMsg.pose.position.z ;
            transformMsg.transform.rotation.x = poseMsg.pose.orientation.x ;
            transformMsg.transform.rotation.y = poseMsg.pose.orientation.y ;
            transformMsg.transform.rotation.z = poseMsg.pose.orientation.z ;
            transformMsg.transform.rotation.w = poseMsg.pose.orientation.w ;
            tf::StampedTransform stampedTransform;
            tf::transformStampedMsgToTF (transformMsg, stampedTransform);
            br.sendTransform(stampedTransform);
          }
        }

        myMutex_.unlock();
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }


    //std::cout << "\t" << threadStartTime << std::endl;
    //std::cout << "\t\t" << ros::Time::now() << std::endl;
  }
  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));
    cam_info_received = true;
    cam_info_sub.shutdown();
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level)
  {
    for(int i = 0 ; i<20 ; i++){
      vDetector[i].setThresholdParams(config.param1,config.param2);
    }
    if (config.normalizeImage)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }
};



int main(int argc,char **argv)
{
  ros::init(argc, argv, "any_aruco_multi");

  AnyArucoMultiConcurrent node;

  //ros::spin();
  ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
  spinner.start();
  ros::waitForShutdown();
}
