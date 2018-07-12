#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <sstream>

#include <iostream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

 void datasets_extraction()
  {
    int count = 0; 
    std::string img_dir;
    for(int i=0; i < 3000; i++)
    {
       std::ostringstream convert;
       convert << i;
       Mat visual_img, rgb_img, depth_img,label_img;
       
       std::string numOfSam;
       if(i < 10000) numOfSam = "000000";
       if(i < 1000) numOfSam = "0000000";
       if(i < 100) numOfSam = "00000000"; 
       if(i < 10) numOfSam = "000000000"; 
       numOfSam.append(convert.str());

       img_dir = "/home/aass/Hoang-Cuong/LabelFusion/data/logs_test/1/images/";
       img_dir = img_dir.append(numOfSam);
       img_dir.append("_color_labels.png");
       visual_img = imread(img_dir, 1);

       img_dir = "/home/aass/Hoang-Cuong/LabelFusion/data/logs_test/1/images/";
       img_dir = img_dir.append(numOfSam);
       img_dir.append("_depth.png");
       depth_img = imread(img_dir, 0);

       img_dir = "/home/aass/Hoang-Cuong/LabelFusion/data/logs_test/1/images/";
       img_dir = img_dir.append(numOfSam);
       img_dir.append("_rgb.png");
       rgb_img = imread(img_dir, 1);

       img_dir = "/home/aass/Hoang-Cuong/LabelFusion/data/logs_test/1/images/";
       img_dir = img_dir.append(numOfSam);
       img_dir.append("_labels.png");
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
          convert1 << count;
          std::string numOfSam;
          if(count < 1000) numOfSam = "0";
          if(count < 100) numOfSam = "00"; 
          if(count < 10) numOfSam = "000"; 
          numOfSam.append(convert1.str());

           img_dir = "/home/aass/Hoang-Cuong/FCN/Datasets/multi-objects/visual/test_labels_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, visual_img);

           img_dir = "/home/aass/Hoang-Cuong/FCN/Datasets/multi-objects/depth/test_depth_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, depth_img);

           img_dir = "/home/aass/Hoang-Cuong/FCN/Datasets/multi-objects/label/test_labels_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, label_img);

           img_dir = "/home/aass/Hoang-Cuong/FCN/Datasets/multi-objects/rgb/test_rgb_";
           img_dir.append(numOfSam);
           img_dir.append(".png");
           imwrite(img_dir, rgb_img);

           std::cerr << "Save images " << count << "\n";

           count++;
       }
    }
  }

int main(int argc, char** argv)
{
  datasets_extraction();
  return 0;
}
