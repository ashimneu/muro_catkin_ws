#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Header.h>

#include <math.h>

using namespace std;
using namespace cv;
using namespace aruco;
// Subscriber to camera
image_transport::Subscriber sub;

image_transport::Publisher imagepub;
image_transport::Publisher drawpub;

ros::Publisher datapub;

float pi = 3.14159265358979;

namespace enc = sensor_msgs::image_encodings;

// Mark info
float MarkX, MarkY; // Mark center
float lastMarkX, lastMarkY; // Last mark center

//Image center
float ImageX, ImageY, D2Center;
Mat camMatrix;
Mat distCoeffs;
CameraParameters CP; 

// static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
//     FileStorage fs(filename, FileStorage::READ);
//     fs["camera_matrix"] >> camMatrix;
//     fs["distortion_coefficients"] >> distCoeffs;
//     fs["image_height"] >> distCoeffs;
//     fs["image_width"] >> distCoeffs;
//     return true;
// }

float linefitinch(float x)
{
    float a,b;
    a = 39.9145433;
    b = -0.357229;
    return a*x+b;
}
float getDistanceToMarker(Marker m)
{
    float distance = 0;
    Mat R(3, 3, CV_32F);
    Mat C(3, 3, CV_32F);
    Rodrigues(m.Rvec,R);
    C = -R.t()*m.Tvec;
    distance = (float) norm(C);

    return distance;
}

Mat getCameraVector(Marker m)
{
    Mat R(3, 3, CV_32F);
    Mat C;
    Rodrigues(m.Rvec,R);
    C = -R.t()*m.Tvec;
    return C;
}
void drawCameraPose(float d, float theta)
{
    float pi = 3.14159265358979;
    float rcoord, ccoord;
    Mat D(480, 640, CV_8UC3, Scalar(0,0,0));
    Point centerPoint(320,240);
    rcoord = 150*d*sin(theta) + centerPoint.x;
    ccoord = 150*d*cos(theta) + centerPoint.y;

    if (rcoord < 0)
    {
        rcoord = centerPoint.x;
    }

    if (ccoord < 0)
    {
        ccoord = centerPoint.y;
    }

    Point cameraPoint(rcoord,ccoord);
    line(D,centerPoint,cameraPoint, Scalar(0, 0, 255), 3);

    // cout << cameraPoint << "\n";

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", D).toImageMsg();
    drawpub.publish(msg);
}
void draw3dAxis(cv::Mat &Image, Marker &m, const CameraParameters &CP, float scale) {

    float size = m.ssize * 3;
    Mat objectPoints(4, 3, CV_32FC1);
    objectPoints.at< float >(0, 0) = 0;
    objectPoints.at< float >(0, 1) = 0;
    objectPoints.at< float >(0, 2) = 0;
    objectPoints.at< float >(1, 0) = size*scale;
    objectPoints.at< float >(1, 1) = 0;
    objectPoints.at< float >(1, 2) = 0;
    objectPoints.at< float >(2, 0) = 0;
    objectPoints.at< float >(2, 1) = size*scale;
    objectPoints.at< float >(2, 2) = 0;
    objectPoints.at< float >(3, 0) = 0;
    objectPoints.at< float >(3, 1) = 0;
    objectPoints.at< float >(3, 2) = size*scale;

    vector< Point2f > imagePoints;
    cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);

    // draw lines of different colours
    cv::line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), 1, CV_AA);
    cv::line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), 1, CV_AA);
    cv::line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), 1, CV_AA);
    putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
    putText(Image, "y", imagePoints[2], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0, 255), 2);
    putText(Image, "z", imagePoints[3], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0, 255), 2);
}

void dataPublisher(int markno, float d, float theta, float distCenter)
{
    geometry_msgs::Pose2D robot_pose;
    float d_t = d*.2/12;

    robot_pose.x = -d_t*cos(theta-pi/2);
    robot_pose.y = d_t*sin(theta-pi/2);

    if (distCenter > 100)
        distCenter = 1;
    else if (distCenter < -100)
        distCenter = -1;
    else
        distCenter = 0;

    robot_pose.theta = distCenter;

    if (markno == 0 && d  == 0 && theta  == 0 && distCenter == 0)
    {
        robot_pose.x = 3;
        robot_pose.y = 3;
        robot_pose.theta = 3;
    }
    datapub.publish(robot_pose);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    //webcam
    // float camData[3][3] = {{584.520663, 0.000000, 316.452759},{0.000000, 548.691007, 214.308927},{0.000000, 0.000000, 1.000000}};
    // float distData[1][5] = {0.083629, -0.211421, -0.012300, -0.007804, 0.000000};
    //picam
    // float camData[3][3] = {{507.798747, 0.000000, 335.021961}, \
    //                        {0.000000, 478.902636, 255.248126}, \
    //                        {0.000000, 0.000000, 1.000000}};

    // float distData[1][5] = {0.116661, -0.209662, -0.002704, 0.000408, 0.000000};
    //picam
    float camData[3][3] = {{514.229146, 0.000000, 305.891459}, \
                           {0.000000, 485.715062, 231.801431}, \
                           {0.000000, 0.000000, 1.000000}};

    float distData[1][5] = {0.196381, -0.295423, -0.003783, -0.009786, 0.000000};

    camMatrix = Mat(3, 3, CV_32FC1, camData);
    distCoeffs = Mat(1, 5, CV_32FC1, distData);

    CP.setParams(camMatrix,distCoeffs,cv::Size(-1, -1));

    MarkerDetector MDetector;
    vector<Marker> Markers;
    cv_bridge::CvImagePtr cv_ptr;

    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

    MDetector.detect(cv_ptr->image, Markers, CP.CameraMatrix, CP.Distorsion, .14); //.053

    float MarkDistances[Markers.size()];
    float MarkDistancesInch[Markers.size()];
    float MarkAngles[Markers.size()];
    for (unsigned int i = 0; i<Markers.size(); i++)
    {
        if (Markers[i].id == 22)
        {
            Markers[i].draw(cv_ptr->image,Scalar(0,0,255),2);
            draw3dAxis(cv_ptr->image, Markers[i], CP, .3);
            MarkDistances[i] = getDistanceToMarker(Markers[i]);
            MarkDistancesInch[i] = linefitinch(MarkDistances[i]);


            Mat C;

            C = getCameraVector(Markers[i]);
            MarkAngles[i] = atan2(C.at<float>(1,0),C.at<float>(2,0));
            cout << fixed;
            cout << setprecision(2);
            cout << "Marker Pose | Distance: " << MarkDistancesInch[i] << " inches | " \
                 << "Angle: " << MarkAngles[i]*180/3.1415 << " degrees \n";
            drawCameraPose(MarkDistances[i],MarkAngles[i]);
            ImageX = cv_ptr->image.cols/2;
            D2Center = Markers[i].getCenter().x - ImageX;

            dataPublisher(i, MarkDistancesInch[i], MarkAngles[i], D2Center);
        }

    }
    if (Markers.size() == 0) 
    {
        dataPublisher(0, 0, 0, 0);
    }


    imagepub.publish(cv_ptr->toImageMsg());
}

int main(int argc, char **argv)
{   
    // readCameraParameters("/home/akhoury/Desktop/raspi_cam_info/camerav2_1280x720.yaml", camMatrix, distCoeffs);
    // CP.readFromXMLFile("/home/akhoury/Documents/raspicam_calibration/640x480/ost.yaml");
    
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/camera/image", 1, imageCallback);
    imagepub = it.advertise("/camera/image_processed", 10);
    drawpub = it.advertise("/camera/drawing", 10);
    datapub = nh.advertise<geometry_msgs::Pose2D>("/aruco/robot_pose",1);
    ros::spin();
}
