/////////////////////////////////////////////////////////////////////////////////
//
// Optical Flow using ROS OpenCV
//   http://answers.ros.org/question/195154/optical-flow-using-ros-opencv/
// 
/////////////////////////////////////////////////////////////////////////////////


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
static const char WINDOW[]="OUTPUT";
namespace enc = sensor_msgs::image_encodings;
image_transport::Publisher image_pub;


void drawOptFlowMap(const cv::Mat& flow, cv::Mat& flowmap, int step, const cv::Scalar& color) {
  float ttc, suml=0, sumr=0, sumc=0, Rvmag, Cvmag, Lvmag;
      for(int y = 0; y < flowmap.rows; y += step) { 
        for (int x=0;x<flowmap.cols*0.33;x += step) { 
          const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x); 
          cv::line(flowmap, cv::Point(x,y), cv::Point((x+fxy.x),(y+fxy.y)), CV_RGB(255,0,0), 2); 
          //cv::circle(flowmap, cv::Point(x,y), 0.1, color, 1); 
          Lvmag = sqrt((int)((fxy.x)*(fxy.x)+(fxy.y)*(fxy.y))); 
          suml+=Lvmag; 
          //ttc= 1/sum * 100; 
        }

        for (int x=flowmap.cols*0.33;x<flowmap.cols*0.33*2;x += step) { 
          const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x); 
          cv::line(flowmap, cv::Point(x,y), cv::Point((x+fxy.x),(y+fxy.y)), CV_RGB(0,255,0), 2); 
          //cv::circle(flowmap, cv::Point(x,y), 0.1, color, 1); 
          Cvmag = sqrt((int)((fxy.x)*(fxy.x)+(fxy.y)*(fxy.y))); 
          sumc+=Cvmag; 
         }

         for (int x=flowmap.cols*0.33*2; x<flowmap.cols; x += step) { 
           const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x); 
           cv::line(flowmap, cv::Point(x,y), cv::Point((x+fxy.x),(y+fxy.y)), CV_RGB(0,0,255), 2); 
           //cv::circle(flowmap, cv::Point(x,y), 0.1, color, 1); 
           Rvmag = sqrt((int)((fxy.x)*(fxy.x)+(fxy.y)*(fxy.y))); 
           sumr+=Rvmag; 
         } 
      }   

      float sum = suml+sumr+sumc; 
      ttc= 1/sum;

      if (ttc < 0.04) { 
        if (suml<sumr && suml<sumc){ 
          cout<< "TURN LEFT" <<endl;
        }else if(sumr<suml && sumr<sumc){ 
          cout<<"TURN RIGHT" <<endl; 
        }
      }else{ 
        cout<<"MOVE FORWARD" <<endl; 
      } 
}


void obstacleAvoidance(cv::Mat& image) {   
    // get a image from camera 
    Mat imageA, imageB; 

    double pyr_scale = 0.5; 
    int levels = 3; int winsize = 5; 
    int iterations = 1; int poly_n = 5; 
    double poly_sigma = 1.1; 
    int flags = 0; 


    cv::resize(image,image, cv::Size(240,160)); 
    cvtColor(image,imageA, CV_BGR2GRAY); 
    imageB = imageA.clone(); 
    Mat flow = Mat(imageA.size(), CV_32FC2); 

    /* Do optical flow computation */ 
    calcOpticalFlowFarneback( imageB, imageA, flow, pyr_scale, levels, winsize, iterations, poly_n, poly_sigma, flags); 
    drawOptFlowMap(flow, image, 20, CV_RGB(0,255,0)); 
    imshow("Output", image); 
    waitKey(100); 
    imageB = imageA.clone(); 
    //image_pub.publish(cv_ptr->toImageMsg());
}

void process(const sensor_msgs::ImageConstPtr& original_image) {
     cv_bridge::CvImagePtr cv_ptr;

     try {
       cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
     } catch (cv_bridge::Exception& e) {
       ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
       return;
     }

     cv::imshow(WINDOW, cv_ptr->image);
     obstacleAvoidance(cv_ptr->image);

     cv::waitKey(3);

}



int main(int argc, char **argv){
    ros::init(argc,argv,"Optical_Flow");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub = it.subscribe("/ardrone/image_raw",1,process);
    image_pub = it.advertise("/arcv/Image",1);
    cv::namedWindow(WINDOW);

    cv::destroyWindow(WINDOW);

    ros::spin();
    return 0;

}
