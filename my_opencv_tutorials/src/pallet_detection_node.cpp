#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// OpenCV specific includes
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <recognition/pallet_detection.h>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include <iostream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

class recogNode
{
  public:
   recogNode();
   virtual ~recogNode();
   void subcribeTopics();
   void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
   void advertiseTopics();
   void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
   void cloudPublish();
   void markersPublish();
   void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
   void datasets_extraction();

   std::string objectName;

   recog_opencl *myRecog;

  private:
   ros::NodeHandle nh_;
   ros::NodeHandle nh_depth_img, nh_rgb_img, nh_cloud, nh_feature, nh_marker;
   ros::Subscriber depth_img_sub, rgb_img_sub, cloud_sub;
   ros::Publisher cloud_detected_pub, cloud_segmented_pub, feature_pub, markers_pub;
   std_msgs::Float64MultiArray array_feature;
   std::vector<string> ObjectNameList;
};

void recogNode::datasets_extraction()
{
    std::string img_dir;
    for(int i=0; i < 1000; i++)
    {
       std::ostringstream convert;
       convert << i;
       Mat visual_img, rgb_img, depth_img,label_img;

       img_dir = "/home/hoang/Datasets/two-pallet/visual/two_pallet_";
       img_dir.append(convert.str());   
       img_dir.append(".png");
       visual_img = imread(img_dir, 0);

       img_dir = "/home/hoang/Datasets/two-pallet/depth/two_pallet_";
       img_dir.append(convert.str());   
       img_dir.append(".png");
       depth_img = imread(img_dir, 0);

       img_dir = "/home/hoang/Datasets/two-pallet/rgb/two_pallet_";
       img_dir.append(convert.str());   
       img_dir.append(".png");
       rgb_img = imread(img_dir, 1);

       img_dir = "/home/hoang/Datasets/two-pallet/label/two_pallet_";
       img_dir.append(convert.str());   
       img_dir.append(".png");
       label_img = imread(img_dir, 0);

       if(!visual_img.data || !depth_img.data || !rgb_img.data || !label_img.data)
       {
          std::cerr << "Cannot read image " << img_dir << "\n";
          if(!visual_img.data) std::cerr << "Cannot read visual_img" << "\n";
          if(!depth_img.data) std::cerr << "Cannot read depth_img" << "\n";
          if(!rgb_img.data) std::cerr << "Cannot read rgb_img" << "\n";
          if(!label_img.data) std::cerr << "Cannot read label_img" << "\n";
       }
       else
       {
          std::ostringstream convert1;
          convert1 << myRecog->count;
          std::string numOfSam;
          if(myRecog->count < 1000) numOfSam = "0";
          if(myRecog->count < 100) numOfSam = "00"; 
          if(myRecog->count < 10) numOfSam = "000"; 
          numOfSam.append(convert1.str());

           img_dir = "/home/hoang/Datasets/two-pallet/visual-final/test_labels_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, visual_img);

           img_dir = "/home/hoang/Datasets/two-pallet/depth-final/test_depth_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, depth_img);

           img_dir = "/home/hoang/Datasets/two-pallet/label-final/test_labels_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, label_img);

           img_dir = "/home/hoang/Datasets/two-pallet/rgb-final/test_rgb_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, rgb_img);

           std::cerr << "Save images " <<  myRecog->count << "\n";

           myRecog->count++;
       }
    }
}

recogNode::recogNode()
{
  myRecog = new recog_opencl();
  array_feature.layout.dim.push_back(std_msgs::MultiArrayDimension());
  array_feature.layout.dim[0].label = "feature";
  array_feature.layout.dim[0].size = 308;
  array_feature.layout.dim[0].stride = 308;
  array_feature.layout.data_offset = 0;
  for(int i = 0; i < 308; i++) array_feature.data.push_back(0);

  nh_ = ros::NodeHandle("~");
  nh_.getParam("vfh_tolerance", myRecog->vfh_tolerance);
  nh_.getParam("vfh_subRange", myRecog->vfh_subRange);
  nh_.getParam("normalThresh", myRecog->normalThresh);
  nh_.getParam("distThresh", myRecog->distThresh);
  nh_.getParam("min_numPoints_segmentation", myRecog->min_numPoints);
  nh_.getParam("gaussian_size", myRecog->gausize);
  nh_.getParam("Min_OBB_MaxAxis", myRecog->OBBthresh_Max[0]);
  nh_.getParam("Max_OBB_MaxAxis", myRecog->OBBthresh_Max[1]);
  nh_.getParam("Min_OBB_MidAxis", myRecog->OBBthresh_Mid[0]);
  nh_.getParam("Max_OBB_MidAxis", myRecog->OBBthresh_Mid[1]);
  nh_.getParam("Min_OBB_MinAxis", myRecog->OBBthresh_Min[0]);
  nh_.getParam("Max_OBB_MinAxis", myRecog->OBBthresh_Min[1]); 
  nh_.getParam("ground_tolerance", myRecog->ground_tolerance);
  nh_.getParam("count_saved_Img", myRecog->count);
  nh_.getParam("depthMax", myRecog->depthMax);
  datasets_extraction();
}

recogNode::~recogNode()
{
  delete myRecog;
};

void recogNode::subcribeTopics()
{
  std::string subcribe_depth_topic= "/camera/depth/image"; 
  std::string subcribe_color_topic= "/camera/rgb/image_rect_color"; 
  rgb_img_sub = nh_rgb_img.subscribe (subcribe_color_topic, 1, &recogNode::rgbCallback, this);  
  depth_img_sub = nh_depth_img.subscribe (subcribe_depth_topic, 1, &recogNode::depthCallback, this);
  
  //cloud_sub = nh_cloud.subscribe ("/camera/depth_registered/points", 1, &recogNode::cloudCallback, this);
}

void recogNode::advertiseTopics()
{
  cloud_detected_pub = nh_cloud.advertise<sensor_msgs::PointCloud2> ("Detected_Cloud", 1);
  cloud_segmented_pub = nh_cloud.advertise<sensor_msgs::PointCloud2> ("Segmented_Cloud", 1);
  feature_pub = nh_feature.advertise<std_msgs::Float64MultiArray>("features", 1);
  markers_pub = nh_marker.advertise<visualization_msgs::MarkerArray>( "objects_recognition", 1);
}

void recogNode::depthCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;

  try
  {
    bridge = cv_bridge::toCvCopy(msg, "32FC1");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform depth image.");
    return;
  }

  *myRecog->depthImg = bridge->image.clone();
  myRecog->activate();
  cloudPublish();
  //markersPublish();
}

void recogNode::rgbCallback (const sensor_msgs::Image::ConstPtr& msg)
{
  cv_bridge::CvImageConstPtr bridge;
  try
  {
    //bridge = cv_bridge::toCvCopy(msg, "8UC3");
    bridge = cv_bridge::toCvCopy(msg, "bgr8");    
    //bridge = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Failed to transform rgb image.");
    return;
  }
  //cv::Mat rgb_image;
  //rgb_image = bridge->image;
  //cv::imshow("RGB image", rgb_image);
  //cv::waitKey(3);
  *myRecog->rgbImg = bridge->image.clone();
}

void recogNode::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{}

void recogNode::cloudPublish()
{
  //if(myRecog->clusters->size() == 0) return;
  pcl::PCLPointCloud2 cloud_filtered;
  sensor_msgs::PointCloud2 output;
  myRecog->clusters->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*myRecog->clusters, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  cloud_segmented_pub.publish (output);

  //if(myRecog->cloud_->size() == 0) return;
  myRecog->cloud_->header.frame_id = "camera_depth_optical_frame";
  pcl::toPCLPointCloud2(*myRecog->cloud_, cloud_filtered);
  pcl_conversions::fromPCL(cloud_filtered, output);
  cloud_detected_pub.publish (output);

  /* if(myRecog->vfhs->size() == 0) 
  { 
    for(int i = 0; i < 308; i++) array_feature.data[i] = 0;    
    std::cerr << "No VFH data!\n"; 
  }
  else for(int i = 0; i < 308; i++) array_feature.data[i] = myRecog->vfhs->points[0].histogram[i];
  feature_pub.publish(array_feature); */
}

void recogNode::markersPublish()
{
    visualization_msgs::MarkerArray multiMarker;
    visualization_msgs::Marker OBB;
    geometry_msgs::Point p;

    OBB.header.frame_id = "camera_depth_optical_frame";
    OBB.header.stamp = ros::Time::now();
    OBB.ns = "OBBs";
    OBB.id = 0;
    OBB.type = visualization_msgs::Marker::LINE_LIST;
    OBB.action = visualization_msgs::Marker::ADD;
    OBB.pose.position.x = 0;
    OBB.pose.position.y = 0;
    OBB.pose.position.z = 0;
    OBB.pose.orientation.x = 0.0;
    OBB.pose.orientation.y = 0.0;
    OBB.pose.orientation.z = 0.0;
    OBB.pose.orientation.w = 1.0;
    OBB.scale.x = 0.003; OBB.scale.y = 0.003; OBB.scale.z = 0.003;
    OBB.color.r = 1.0f; OBB.color.g = 1.0f; OBB.color.b = 1.0f; OBB.color.a = 1.0;

    //std::vector<visualization_msgs::Marker> ObjectNameList;
    visualization_msgs::Marker ObjectName;
    ObjectName.header.frame_id = "camera_depth_optical_frame";
    ObjectName.header.stamp = ros::Time::now();
    ObjectName.id = 0;
    ObjectName.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    ObjectName.action = visualization_msgs::Marker::ADD;
    ObjectName.pose.orientation.x = 0.0;
    ObjectName.pose.orientation.y = 0.0;
    ObjectName.pose.orientation.z = 0.0;
    ObjectName.pose.orientation.w = 1.0;
    ObjectName.scale.x = 0.05; ObjectName.scale.y = 0.05; ObjectName.scale.z = 0.05;
    ObjectName.color.r = 1.0f; ObjectName.color.g = 1.0f; ObjectName.color.b = 1.0f; ObjectName.color.a = 1.0;

    if(myRecog->OBB->size() == 0) 
    { 
      std::cerr << "No OBB data!";
      OBB.lifetime = ros::Duration();
      multiMarker.markers.push_back(OBB);
      if(!ObjectNameList.empty())
      {
        for(int i=0; i < ObjectNameList.size(); i++)
        {
          ObjectName.ns  = ObjectNameList[i];
          ObjectName.text = "Cuong";
          ObjectName.lifetime = ros::Duration();
          multiMarker.markers.push_back(ObjectName);
        }
        ObjectNameList.clear();
      }
      
      markers_pub.publish(multiMarker);
      return;
    };

   // Delete previous texts
    if(!ObjectNameList.empty())
      {
        for(int i=0; i < ObjectNameList.size(); i++)
        {
          ObjectName.ns  = ObjectNameList[i];
          ObjectName.text = "Cuong";
          ObjectName.lifetime = ros::Duration();
          multiMarker.markers.push_back(ObjectName);
        }
      }
    ObjectNameList.clear();

    for(int k = 0; k < myRecog->recognizedObjects.size(); k++)
    {
       int begin = k*9; int stop = begin + 8;
       for(int i = begin; i < stop; i++)
       {
          if(i == begin + 3 || i == begin + 7)
          {
             p.x = myRecog->OBB->points[i].x;
             p.y = myRecog->OBB->points[i].y; p.z = myRecog->OBB->points[i].z;
             OBB.points.push_back(p);
             p.x = myRecog->OBB->points[i-3].x;
             p.y = myRecog->OBB->points[i-3].y; p.z = myRecog->OBB->points[i-3].z;
             OBB.points.push_back(p);
             if(i == begin + 3)
             {
                p.x = myRecog->OBB->points[i].x;
                p.y = myRecog->OBB->points[i].y; p.z = myRecog->OBB->points[i].z;
                OBB.points.push_back(p);
                p.x = myRecog->OBB->points[begin + 5].x;
                p.y = myRecog->OBB->points[begin + 5].y; 
                p.z = myRecog->OBB->points[begin + 5].z;
                OBB.points.push_back(p);
             }
             if(i == begin + 7)
             {
                p.x = myRecog->OBB->points[i].x;
                p.y = myRecog->OBB->points[i].y; p.z = myRecog->OBB->points[i].z;
                OBB.points.push_back(p);
                p.x = myRecog->OBB->points[begin + 1].x;
                p.y = myRecog->OBB->points[begin + 1].y; 
                p.z = myRecog->OBB->points[begin + 1].z;
                OBB.points.push_back(p);
             }
          }
          else
          {
             p.x = myRecog->OBB->points[i].x;
             p.y = myRecog->OBB->points[i].y; p.z = myRecog->OBB->points[i].z;
             OBB.points.push_back(p);
             p.x = myRecog->OBB->points[i+1].x;
             p.y = myRecog->OBB->points[i+1].y; p.z = myRecog->OBB->points[i+1].z;
             OBB.points.push_back(p);
             if(i == begin + 0)
             {
                p.x = myRecog->OBB->points[i].x;
                p.y = myRecog->OBB->points[i].y; p.z = myRecog->OBB->points[i].z;
                OBB.points.push_back(p);
                p.x = myRecog->OBB->points[begin + 6].x;
                p.y = myRecog->OBB->points[begin + 6].y; 
                p.z = myRecog->OBB->points[begin + 6].z;
                OBB.points.push_back(p);
             }
             if(i == begin + 2)
             {
                p.x = myRecog->OBB->points[i].x;
                p.y = myRecog->OBB->points[i].y; p.z = myRecog->OBB->points[i].z;
                OBB.points.push_back(p);
                p.x = myRecog->OBB->points[begin + 4].x;
                p.y = myRecog->OBB->points[begin + 4].y; 
                p.z = myRecog->OBB->points[begin + 4].z;
                OBB.points.push_back(p);
             }
          }
       }
       
       ostringstream convert;
       convert << k;
       ObjectName.ns = myRecog->objectName[myRecog->recognizedObjects[k]] + convert.str();
       ObjectName.pose.position.x = myRecog->OBB->points[begin + k].x;
       ObjectName.pose.position.y = myRecog->OBB->points[begin + k].y;
       ObjectName.pose.position.z = myRecog->OBB->points[begin + k].z;
       ObjectName.color.a = 1.0;
       ObjectName.text = myRecog->objectName[myRecog->recognizedObjects[k]];
       ObjectNameList.push_back(ObjectName.text);

       std::cerr << ObjectName.text << "\n";
       ObjectName.lifetime = ros::Duration();
       multiMarker.markers.push_back(ObjectName);
    }

    OBB.lifetime = ros::Duration();
    multiMarker.markers.push_back(OBB);

    markers_pub.publish(multiMarker);
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "sdf_tracker_node");
  recogNode mainRecogNode;
  if(argc < 2) std::cerr << "\nName of Object is empty\n\n";
  else mainRecogNode.objectName = std::string(argv[1]);
  mainRecogNode.subcribeTopics();
  mainRecogNode.advertiseTopics();
  ros::spin();

  return 0;
}