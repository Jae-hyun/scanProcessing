#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

const double PI = 3.1515926;

const int keepFrameNum = 100;
int pointCloudSize[keepFrameNum] = {0};
float *pointCloudArray[keepFrameNum] = {NULL};
int pointCloudIndex = keepFrameNum - 1;
int totalPointNum = 0;

pcl::PointCloud<pcl::PointXYZ> pointCloud;

bool newLaserPoints = false;

double vehXCur, vehZCur, vehHeadingCur;
double vehXLast, vehZLast, vehHeadingLast;

bool newEncoderOdometry = false;

void myEncoderOdometryHandler(const nav_msgs::Odometry::ConstPtr& encoderOdometry)
{
  vehXCur = encoderOdometry->pose.pose.position.y;
  vehZCur = encoderOdometry->pose.pose.position.x;
  //vehHeadingCur = tf::getYaw(encoderOdometry->pose.pose.orientation);
  
  newEncoderOdometry = true;
}

void myLaserPointsHandler(const sensor_msgs::PointCloud2ConstPtr& laserPoints) 
{
  pcl::fromROSMsg(*laserPoints, pointCloud);
  newLaserPoints = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointRegistration");
  ros::NodeHandle nh;

  ros::Subscriber subvo = nh.subscribe<nav_msgs::Odometry> ("/pose", 1, myEncoderOdometryHandler);
  ros::Subscriber subpc = nh.subscribe<sensor_msgs::PointCloud2> ("/sync_scan_cloud_filtered", 1, myLaserPointsHandler);
  ros::Publisher pubpc = nh.advertise<sensor_msgs::PointCloud2> ("/laser_points_Registered", 1);

  //IplImage* image = cvCreateImage(cvSize(480, 480), IPL_DEPTH_8U, 3);

  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserPoints) {
      newLaserPoints = false;

      int pointNum = pointCloud.points.size();
      pointCloudIndex = (pointCloudIndex + 1) % keepFrameNum;
      delete[] pointCloudArray[pointCloudIndex];
      pointCloudArray[pointCloudIndex] = new float[3 * pointNum];
      int pointCount = 0;
      double pointX, pointY, pointZ;
      for (int i = 0; i < pointNum; i++) {
        if (pointCloud.points[i].y > -0.6) {
          pointCloudArray[pointCloudIndex][3 * pointCount] = pointCloud.points[i].x;
          pointCloudArray[pointCloudIndex][3 * pointCount + 1] = pointCloud.points[i].y;
          pointCloudArray[pointCloudIndex][3 * pointCount + 2] = pointCloud.points[i].z;
          pointCount++;
        }
      }
      pointNum = pointCount;
      totalPointNum -= pointCloudSize[pointCloudIndex];
      pointCloudSize[pointCloudIndex] = pointNum;
      totalPointNum += pointNum;

      //ROS_INFO ("Total point nmumber: %d", totalPointNum);

      if (newEncoderOdometry) {
        newEncoderOdometry = false;

        float driveDis = sqrt((vehXCur - vehXLast) * (vehXCur - vehXLast) + (vehZCur - vehZLast) * (vehZCur - vehZLast));
        float angTemp = atan2(vehXCur - vehXLast, vehZCur - vehZLast) - vehHeadingCur;
        if (angTemp > PI) {
          angTemp -= 2 * PI;
        } else if (angTemp < -PI) {
          angTemp += 2 * PI;
        }
        if (angTemp > PI / 2 || angTemp < -PI / 2) {
          driveDis *= -1;
        }
        float headingAng = vehHeadingCur - vehHeadingLast;
        vehXLast = vehXCur;
        vehZLast = vehZCur;
        vehHeadingLast = vehHeadingCur;

        float pointX, pointZ;
        for (int j = pointCloudIndex + 1; j < pointCloudIndex + keepFrameNum; j++) {
          int index = j % keepFrameNum;
          pointNum = pointCloudSize[index];
          for (int i = 0; i < pointNum; i++) {
            pointX = pointCloudArray[index][3 * i];
            pointZ = pointCloudArray[index][3 * i + 2];
            pointZ -= driveDis;
            pointCloudArray[index][3 * i] = pointX * cos(headingAng) - pointZ * sin(headingAng);
            pointCloudArray[index][3 * i + 2] = pointX * sin(headingAng) + pointZ * cos(headingAng);
          }
        }
      }

      pcl::PointCloud<pcl::PointXYZ> pointCloudReg(totalPointNum, 1);
      pointCloudReg.header.stamp = pointCloud.header.stamp;
      pointCloudReg.header.frame_id = "/camera";
      pointCount = 0;
      for (int j = pointCloudIndex + 1; j < pointCloudIndex + keepFrameNum; j++) {
        int index = j % keepFrameNum;
        pointNum = pointCloudSize[index];
        for (int i = 0; i < pointNum; i++) {
          pointCloudReg.points[pointCount].x = pointCloudArray[index][3 * i];
          pointCloudReg.points[pointCount].y = pointCloudArray[index][3 * i + 1];
          pointCloudReg.points[pointCount].z = pointCloudArray[index][3 * i + 2];
          pointCount++;
        }
      }
      sensor_msgs::PointCloud2 laserPointsReg;
      pcl::toROSMsg(pointCloudReg, laserPointsReg);
      pubpc.publish(laserPointsReg);

      /*for (int j = 0; j < keepFrameNum; j++) {
        pointNum = pointCloudSize[j];
        for (int i = 0; i < pointNum; i++) {
          cvCircle(image, cvPoint(240 - 20 * pointCloudArray[j][3 * i], 400 - 20 * pointCloudArray[j][3 * i + 2]), 
                   1, CV_RGB(0, 0, 255), 1);
        }
      }
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(255, 0, 0), 2);
      cvShowImage("Laser Points", image);
      for (int j = 0; j < keepFrameNum; j++) {
        pointNum = pointCloudSize[j];
        for (int i = 0; i < pointNum; i++) {
          cvCircle(image, cvPoint(240 - 20 * pointCloudArray[j][3 * i], 400 - 20 * pointCloudArray[j][3 * i + 2]),
                   1, CV_RGB(0, 0, 0), 1);
        }
      }
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(0, 0, 0), 2);*/
    }

    char c = cvWaitKey(10);
    if(c == 27) {
      return 0;
    }

    status = ros::ok();
  }

  return 0;
}
