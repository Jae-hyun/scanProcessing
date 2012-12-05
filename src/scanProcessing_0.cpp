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

/*const int keepFilteredFrameNum = 100;
int pointCloudFilteredSize[keepFilteredFrameNum] = {0};
float *pointCloudFilteredArray[keepFilteredFrameNum] = {NULL};
int pointCloudFilteredIndex = keepFilteredFrameNum - 1;

const int gridMapWidth = 200;
const int gridMapHeight = 200;
const int gridMapCellNum = gridMapWidth * gridMapHeight;
const float gridMapScale = 10;
const int gridMapCenterX = 100;
const int gridMapCenterZ = 50;*/

pcl::PointCloud<pcl::PointXYZ> pointCloud;
int totalPointNum = 0;

bool newLaserPoints = false;

float vehXCur, vehZCur, vehHeadingCur;
float vehXLast, vehZLast, vehHeadingLast;

//double v0, v1, v2, m00, m01, m02, m10, m11, m12, m20, m21, m22;

bool newEncoderOdometry = false;

void myEncoderOdometryHandler(const nav_msgs::Odometry::ConstPtr& encoderOdometry)
{
  vehXCur = encoderOdometry->pose.pose.position.y;
  vehZCur = encoderOdometry->pose.pose.position.x;
  vehHeadingCur = tf::getYaw(encoderOdometry->pose.pose.orientation);

  /*v0 = visualOdometry->pose.pose.position.x;
  v1 = visualOdometry->pose.pose.position.y;
  v2 = visualOdometry->pose.pose.position.z;

  double qx = visualOdometry->pose.pose.orientation.x;
  double qy = visualOdometry->pose.pose.orientation.y;
  double qz = visualOdometry->pose.pose.orientation.z;
  double qw = visualOdometry->pose.pose.orientation.w;
  double sqx = qx*qx;
  double sqy = qy*qy;
  double sqz = qz*qz;
  double sqw = qw*qw;

  double invs = 1 / (sqx + sqy + sqz + sqw);
  m00 = (sqx - sqy - sqz + sqw) * invs;
  m11 = (-sqx + sqy - sqz + sqw) * invs;
  m22 = (-sqx - sqy + sqz + sqw)* invs;
  double tmp1 = qx * qy;
  double tmp2 = qz * qw;
  m10 = 2.0 * (tmp1 + tmp2) * invs;
  m01 = 2.0 * (tmp1 - tmp2) * invs;
  tmp1 = qx * qz;
  tmp2 = qy * qw;
  m20 = 2.0 * (tmp1 - tmp2) * invs;
  m02 = 2.0 * (tmp1 + tmp2) * invs;
  tmp1 = qy * qz;
  tmp2 = qx * qw;
  m21 = 2.0 * (tmp1 + tmp2) * invs;
  m12 = 2.0 * (tmp1 - tmp2) * invs;*/   
  
  newEncoderOdometry = true;
}

void myLaserPointsHandler(const sensor_msgs::PointCloud2ConstPtr& laserPoints) 
{
  pcl::fromROSMsg(*laserPoints, pointCloud);
  newLaserPoints = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanProcessing");
  ros::NodeHandle nh;

  ros::Subscriber subvo = nh.subscribe<nav_msgs::Odometry> ("/accumulated/vo", 1, myEncoderOdometryHandler);
  ros::Subscriber subpc = nh.subscribe<sensor_msgs::PointCloud2> ("/sync_scan_cloud_filtered", 1, myLaserPointsHandler);
  ros::Publisher pubpc = nh.advertise<sensor_msgs::PointCloud2> ("/laser_points_Reg", 1);

  IplImage* image = cvCreateImage(cvSize(480, 480), IPL_DEPTH_8U, 3);

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
        if (pointCloud.points[i].y > -10) {
          /*pointX = pointCloud.points[i].x + v0;
          pointY = pointCloud.points[i].y + v1;
          pointZ = pointCloud.points[i].z + v2;
          pointCloudArray[pointCloudIndex][3 * pointCount] = pointX;
          pointCloudArray[pointCloudIndex][3 * pointCount + 1] = pointY;
          pointCloudArray[pointCloudIndex][3 * pointCount + 2] = pointZ;*/
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

      if (newEncoderOdometry) {
        newEncoderOdometry = false;

        float driveDis = sqrt((vehXCur - vehXLast) * (vehXCur - vehXLast) + (vehZCur - vehZLast) * (vehZCur - vehZLast));
        float headingAng = vehHeadingCur - vehHeadingLast;
        vehXLast = vehXCur;
        vehZLast = vehZCur;
        vehHeadingLast = vehHeadingCur;

        float pointX, pointZ;
        for (int j = pointCloudIndex + 1; j < pointCloudIndex + keepFrameNum; j++) {
          int index = j % keepFrameNum;
          int pointNum = pointCloudSize[index];
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
        int pointNum = pointCloudSize[index];
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

      /*int lastIndex = (pointCloudIndex + 1) % keepFrameNum;
      int lastPointNum = pointCloudSize[lastIndex];
      int *lastPointValid = new int[lastPointNum];
      for (int i = 0; i < lastPointNum; i++) {
        lastPointValid[i] = 1;
      }
      for (int j = pointCloudIndex + 2; j <= pointCloudIndex + keepFrameNum; j++) {
        int curIndex = j % keepFrameNum;
        int curPointNum = pointCloudSize[curIndex];
        int *curPointValid = new int[curPointNum];
        for (int i = 0; i < curPointNum; i++) {
          curPointValid[i] = 0;
        }
        for (int i = 0; i < lastPointNum; i++) {
          if (lastPointValid[i] == 1) {
            float lastPointX = pointCloudArray[lastIndex][3 * i];
            float lastPointZ = pointCloudArray[lastIndex][3 * i + 2];
            for (int k = 0; k < curPointNum; k++) {
              if ((lastPointX - pointCloudArray[curIndex][3 * k]) * (lastPointX - pointCloudArray[curIndex][3 * k]) + 
                  (lastPointZ - pointCloudArray[curIndex][3 * k + 2]) * (lastPointZ - pointCloudArray[curIndex][3 * k + 2]) < 0.1 * 0.1) {
                curPointValid[k] = 1;
              }
            }
          }
        }
        lastIndex = curIndex;
        lastPointNum = curPointNum;
        delete[] lastPointValid;
        lastPointValid = curPointValid;
      }
      int pointValidNum = 0;
      for (int i = 0; i < pointNum; i++) {
        if (lastPointValid[i] == 1) {
          pointValidNum++;
        }
      }

      pointCloudFilteredIndex = (pointCloudFilteredIndex + 1) % keepFilteredFrameNum;
      delete[] pointCloudFilteredArray[pointCloudFilteredIndex];
      pointCloudFilteredArray[pointCloudFilteredIndex] = new float[3 * pointValidNum];
      int pointValidIndex = 0;
      for (int i = 0; i < pointNum; i++) {
        if (lastPointValid[i] == 1) {
          pointCloudFilteredArray[pointCloudFilteredIndex][3 * pointValidIndex] = pointCloudArray[pointCloudIndex][3 * i];
          pointCloudFilteredArray[pointCloudFilteredIndex][3 * pointValidIndex + 1] = pointCloudArray[pointCloudIndex][3 * i + 1];
          pointCloudFilteredArray[pointCloudFilteredIndex][3 * pointValidIndex + 2] = pointCloudArray[pointCloudIndex][3 * i + 2];
          pointValidIndex++;
        }
      }
      pointCloudFilteredSize[pointCloudFilteredIndex] = pointValidNum;

      if (newEncoderOdometry) {
        newEncoderOdometry = false;

        float driveDis = sqrt((vehXCur - vehXLast) * (vehXCur - vehXLast) + (vehZCur - vehZLast) * (vehZCur - vehZLast));
        if (cos(vehHeadingCur) < 0) {
          driveDis *= -1;
        }
        float headingAng = vehHeadingCur - vehHeadingLast;
        vehXLast = vehXCur;
        vehZLast = vehZCur;
        vehHeadingLast = vehHeadingCur;

        float filteredPointX, filteredPointZ;
        for (int j = pointCloudFilteredIndex + 1; j < pointCloudFilteredIndex + keepFilteredFrameNum; j++) {
          int filteredIndex = j % keepFilteredFrameNum;
          int filteredPointNum = pointCloudFilteredSize[filteredIndex];
          for (int i = 0; i < filteredPointNum; i++) {
            filteredPointX = pointCloudFilteredArray[filteredIndex][3 * i];
            filteredPointZ = pointCloudFilteredArray[filteredIndex][3 * i + 2];
            filteredPointZ -= driveDis;
            pointCloudFilteredArray[filteredIndex][3 * i] = filteredPointX * cos(headingAng) - filteredPointZ * sin(headingAng);
            pointCloudFilteredArray[filteredIndex][3 * i + 2] = filteredPointX * sin(headingAng) + filteredPointZ * cos(headingAng);
          }
        }
      }

      int mapPointX, mapPointZ;
      int gridMap[gridMapCellNum] = {0};
      for (int j = 0; j < keepFilteredFrameNum; j++) {
        int filteredPointNum = pointCloudFilteredSize[j];
        for (int i = 0; i < filteredPointNum; i++) {
          mapPointX = gridMapScale * pointCloudFilteredArray[j][3 * i] + gridMapCenterX;
          mapPointZ = gridMapScale * pointCloudFilteredArray[j][3 * i + 2] + gridMapCenterZ;
          if (mapPointX >= 0 && mapPointX < gridMapWidth && mapPointZ >= 0 && mapPointZ < gridMapHeight) {
            gridMap[gridMapWidth * mapPointZ + mapPointX]++;
          }
        }
      }
      int mapPointScore;
      int gridMapValid[gridMapCellNum] = {0};
      for (mapPointX = 1; mapPointX < gridMapWidth - 1; mapPointX++) {
        for (mapPointZ = 1; mapPointZ < gridMapHeight - 1; mapPointZ++) {
          mapPointScore = gridMap[gridMapWidth * mapPointZ + mapPointX];
          mapPointScore += gridMap[gridMapWidth * (mapPointZ - 1) + mapPointX];
          mapPointScore += gridMap[gridMapWidth * mapPointZ + (mapPointX - 1)];
          mapPointScore += gridMap[gridMapWidth * (mapPointZ + 1) + mapPointX];
          mapPointScore += gridMap[gridMapWidth * mapPointZ + (mapPointX + 1)];
          mapPointScore += gridMap[gridMapWidth * (mapPointZ - 1) + (mapPointX - 1)];
          mapPointScore += gridMap[gridMapWidth * (mapPointZ + 1) + (mapPointX - 1)];
          mapPointScore += gridMap[gridMapWidth * (mapPointZ - 1) + (mapPointX + 1)];
          mapPointScore += gridMap[gridMapWidth * (mapPointZ + 1) + (mapPointX + 1)];
          if (mapPointScore > 100) {
            gridMapValid[gridMapWidth * mapPointZ + mapPointX] = 1;
          }
        }
      }

      pcl::PointCloud<pcl::PointXYZ> pointCloudFiltered(pointValidNum, 1);
      pointCloudFiltered.header.stamp = pointCloud.header.stamp;
      pointCloudFiltered.header.frame_id = "/camera";
      int pointValidIndex = 0;
      for (int i = 0; i < pointNum; i++) {
        if (lastPointValid[i] == 1) {
          pointCloudFiltered.points[pointValidIndex].x = pointCloudArray[pointCloudIndex][3 * i];
          pointCloudFiltered.points[pointValidIndex].y = pointCloudArray[pointCloudIndex][3 * i + 1];
          pointCloudFiltered.points[pointValidIndex].z = pointCloudArray[pointCloudIndex][3 * i + 2];
          pointValidIndex++;
        }
      }
      sensor_msgs::PointCloud2 laserPointsFiltered;
      pcl::toROSMsg(pointCloudFiltered, laserPointsFiltered);
      pubpc.publish(laserPointsFiltered);
      delete[] lastPointValid;

      for (int j = 0; j < keepFilteredFrameNum; j++) {
        int filteredPointNum = pointCloudFilteredSize[j];
        for (int i = 0; i < filteredPointNum; i++) {
          mapPointX = gridMapScale * pointCloudFilteredArray[j][3 * i] + gridMapCenterX;
          mapPointZ = gridMapScale * pointCloudFilteredArray[j][3 * i + 2] + gridMapCenterZ;
          if (mapPointX >= 0 && mapPointX < gridMapWidth && mapPointZ >= 0 && mapPointZ < gridMapHeight) {
            if (gridMapValid[gridMapWidth * mapPointZ + mapPointX] == 1) {
              cvCircle(image, cvPoint(240 - 20 * pointCloudFilteredArray[j][3 * i], 400 - 20 * pointCloudFilteredArray[j][3 * i + 2]), 
                       1, CV_RGB(0, 0, 255), 1);
            }
          }
        }
      }*/

      for (int j = 0; j < keepFrameNum; j++) {
        int pointNum = pointCloudSize[j];
        for (int i = 0; i < pointNum; i++) {
          cvCircle(image, cvPoint(240 - 20 * pointCloudArray[j][3 * i], 400 - 20 * pointCloudArray[j][3 * i + 2]), 
                   1, CV_RGB(0, 0, 255), 1);
        }
      }
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(255, 0, 0), 2);
      cvShowImage("Laser Points", image);
      for (int j = 0; j < keepFrameNum; j++) {
        int pointNum = pointCloudSize[j];
        for (int i = 0; i < pointNum; i++) {
          cvCircle(image, cvPoint(240 - 20 * pointCloudArray[j][3 * i], 400 - 20 * pointCloudArray[j][3 * i + 2]),
                   1, CV_RGB(0, 0, 0), 1);
        }
      }
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(0, 0, 0), 2);
    }

    char c = cvWaitKey(10);
    if(c == 27) {
      return 0;
    }

    status = ros::ok();
  }

  return 0;
}
