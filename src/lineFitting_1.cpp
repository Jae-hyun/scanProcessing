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

int oriPointNum;
const int MAXORIPOINTNUM = 10000;
float oriPointX[MAXORIPOINTNUM];
float oriPointY[MAXORIPOINTNUM];
float oriPointZ[MAXORIPOINTNUM];

const int cellNumX = 100;
const int cellNumY = 12;
const double cellScale = 10;
const int cellCenterX = 50;
const int cellCenterY = 5;
const int totalCellNum = cellNumX * cellNumY;
int gridmapXY[totalCellNum];
int cellPointNumXL[cellNumY];
int cellPointNumXR[cellNumY];
double offsetMeanXL[cellNumY];
double offsetMeanXR[cellNumY];
double offsetVarXL[cellNumY];
double offsetVarXR[cellNumY];

pcl::PointCloud<pcl::PointXYZ> pointCloud;

bool newLaserPoints = false;

float vehXCur, vehZCur, vehHeadingCur;
float vehXLast, vehZLast, vehHeadingLast;

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
  oriPointNum = 0;
  int totalPointNum = pointCloud.points.size();
  for (int i = 0; i < totalPointNum; i++) {
    if (pointCloud.points[i].y > 0.2 && pointCloud.points[i].y < 1.2 && pointCloud.points[i].x > -4 && pointCloud.points[i].x > -4 &&
        pointCloud.points[i].x < 3 + 1 * pointCloud.points[i].z && pointCloud.points[i].x > -3 - 1 * pointCloud.points[i].z && 
        oriPointNum < MAXORIPOINTNUM) {
      oriPointX[oriPointNum] = pointCloud.points[i].x;
      oriPointY[oriPointNum] = pointCloud.points[i].y;
      oriPointZ[oriPointNum] = pointCloud.points[i].z;
      oriPointNum++;
    }
  }

  newLaserPoints = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lineFitting");
  ros::NodeHandle nh;

  ros::Subscriber subvo = nh.subscribe<nav_msgs::Odometry> ("/pose", 1, myEncoderOdometryHandler);
  ros::Subscriber subpc = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_points_Registered", 1, myLaserPointsHandler);

  IplImage* image = cvCreateImage(cvSize(480, 480), IPL_DEPTH_8U, 3);

  double intB = 0;
  double intAL = 2;
  double intAR = -2;
  int inlierDisZ[200];
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserPoints) {
      newLaserPoints = false;

      if (oriPointNum < 10) {
        continue;
      }

      int seedID1, seedID2;
      float seedX1, seedZ1, seedX2, seedZ2;
      double lineB, lineA, bestA, bestB;
      double sumx, sumxz, sumz, sumzz, den, dis;
      int dep, inlierCount, inlierNum, bestInlerNum = 0;
      for (int iterNum = 0; iterNum < 200; iterNum++) {
        seedID1 = rand() % oriPointNum;
        seedX1 = oriPointX[seedID1];
        seedZ1 = oriPointZ[seedID1];

        seedID2 = rand() % oriPointNum;
        if (seedID2 == seedID1) {
          seedID2 = (seedID2 + oriPointNum / 2) % oriPointNum;
        }
        seedX2 = oriPointX[seedID2];
        seedZ2 = oriPointZ[seedID2];

        lineB = (seedX1 - seedX2) / (seedZ1 - seedZ2);
        lineA = seedX1 - lineB * seedZ1;

        if (lineB > 1 || lineB < -1) {
          continue;
        }

        inlierNum = 0;
        sumx = 0; sumxz = 0; sumz = 0; sumzz = 0;
        den = sqrt(lineB * lineB + 1);
        for (int i = 0; i < oriPointNum; i++) {
          dis = (lineB * oriPointZ[i] - oriPointX[i] + lineA) / den;
          if (dis < 0.5 && dis > -0.5) {
            sumx += oriPointX[i];
            sumxz += oriPointX[i] * oriPointZ[i];
            sumz += oriPointZ[i];
            sumzz += oriPointZ[i] * oriPointZ[i];
            inlierNum++;
          }
        }

        lineB = (inlierNum * sumxz - sumx * sumz) / (inlierNum * sumzz - sumz * sumz);
        lineA = (sumx * sumzz - sumz * sumxz) / (inlierNum * sumzz - sumz * sumz);

        den = sqrt(lineB * lineB + 1);
        for (int i = 0; i < 200; i++) {
          inlierDisZ[i] = 0;
        }
        for (int i = 0; i < oriPointNum; i++) {
          dis = (lineB * oriPointZ[i] - oriPointX[i] + lineA) / den;
          if (dis < 0.5 && dis > -0.5) {
            dep = 10 * sqrt(oriPointX[i] * oriPointX[i] + oriPointZ[i] * oriPointZ[i] - dis * dis);
            if (dep > 200 - 1) {
              dep = 200 - 1;
            }
            inlierDisZ[dep]++;
          }
        }
        inlierNum = 0;
        for (int i = 2; i < 200 - 2; i++) {
          inlierCount = inlierDisZ[i - 2] + inlierDisZ[i - 1] + inlierDisZ[i] + inlierDisZ[i + 1] + inlierDisZ[i + 2];
          if (inlierCount > 25) {
            inlierCount = 25;
          }
          inlierNum += inlierCount;
        }
        inlierNum /= 5;

        if (bestInlerNum < inlierNum && lineB < 1 && lineB > -1) {
          bestA = lineA;
          bestB = lineB;
          bestInlerNum = inlierNum;
        }
      }

      if (newEncoderOdometry) {
        newEncoderOdometry = false;
        intB = tan(atan(intB) - vehHeadingCur + vehHeadingLast);
        vehXLast = vehXCur;
        vehZLast = vehZCur;
        vehHeadingLast = vehHeadingCur;
      }
      if (intB - bestB < 0.5 && intB - bestB > -0.5) {
        intB = (1 - 0.05) * intB + 0.05 * bestB;
      } else {
        intB = (1 - 0.005) * intB + 0.005 * bestB;
      }

      for (int i = 0; i < totalCellNum; i++) {
        gridmapXY[i] = 0;
      }
      den = sqrt(intB * intB + 1);
      int totalPointNum = pointCloud.points.size();
      int cellIndX, cellIndY;
      for (int i = 0; i < totalPointNum; i++) {
        if (pointCloud.points[i].y > -0.5 && pointCloud.points[i].y < 0.7 && pointCloud.points[i].x > -4 && pointCloud.points[i].x > -4 &&
            pointCloud.points[i].x < 3 + 1 * pointCloud.points[i].z && pointCloud.points[i].x > -3 - 1 * pointCloud.points[i].z) {
          dis = (pointCloud.points[i].x - intB * pointCloud.points[i].z) / den;
          cellIndX = cellScale * dis + cellCenterX;
          cellIndY = cellScale * pointCloud.points[i].y + cellCenterY;
          if (cellIndX >= 0 && cellIndX < cellNumX && cellIndY >= 0 && cellIndY < cellNumY) {
            gridmapXY[cellNumX * cellIndY + cellIndX]++;
          }
        }
      }
      
      for (int i = 0; i < cellNumY; i++) {
        cellPointNumXL[i] = 0;
        cellPointNumXR[i] = 0;
        offsetMeanXL[i] = 0;
        offsetMeanXR[i] = 0;
        offsetVarXL[i] = 0;
        offsetVarXR[i] = 0;
      }
      for (int i = 1; i < cellNumY - 1; i++) {
        for (int j = cellCenterX; j < cellNumX; j++) {
          cellPointNumXL[i] += gridmapXY[cellNumX * (i - 1) + j] + gridmapXY[cellNumX * i + j] + gridmapXY[cellNumX * (i + 1) + j];
          offsetMeanXL[i] += (gridmapXY[cellNumX * (i - 1) + j] + gridmapXY[cellNumX * i + j] + gridmapXY[cellNumX * (i + 1) + j]) * j;
        }
        for (int j = 0; j < cellCenterX; j++) {
          cellPointNumXR[i] += gridmapXY[cellNumX * (i - 1) + j] + gridmapXY[cellNumX * i + j] + gridmapXY[cellNumX * (i + 1) + j];
          offsetMeanXR[i] += (gridmapXY[cellNumX * (i - 1) + j] + gridmapXY[cellNumX * i + j] + gridmapXY[cellNumX * (i + 1) + j]) * j;
        }
      }
      for (int i = 1; i < cellNumY - 1; i++) {
        if (cellPointNumXL[i] > 0) {
          offsetMeanXL[i] /= cellPointNumXL[i];
          for (int j = cellCenterX; j < cellNumX; j++) {
            offsetVarXL[i] += (gridmapXY[cellNumX * (i - 1) + j] + gridmapXY[cellNumX * i + j] + gridmapXY[cellNumX * (i + 1) + j]) 
                            / double(cellPointNumXL[i]) * (j - offsetMeanXL[i]) * (j - offsetMeanXL[i]);
           }
        }
        if (cellPointNumXR[i] > 0) {
          offsetMeanXR[i] /= cellPointNumXR[i];
          for (int j = 0; j < cellCenterX; j++) {
            offsetVarXR[i] += (gridmapXY[cellNumX * (i - 1) + j] + gridmapXY[cellNumX * i + j] + gridmapXY[cellNumX * (i + 1) + j]) 
                            / double(cellPointNumXR[i]) * (j - offsetMeanXR[i]) * (j - offsetMeanXR[i]);
          }
        }
      }
      for (int i = 1; i < cellNumY - 1; i++) {
        offsetMeanXL[i] = (offsetMeanXL[i] - cellCenterX) / cellScale;
        offsetMeanXR[i] = (offsetMeanXR[i] - cellCenterX) / cellScale;
        offsetVarXL[i] /= cellScale * cellScale;
        offsetVarXR[i] /= cellScale * cellScale;
      }
      int minIDXL, minIDXR;
      double minVarXL = 100000;
      double minVarXR = 100000;
      for (int i = 1; i < cellNumY - 1; i++) {
        if (cellPointNumXL[i] > 0 && offsetVarXL[i] < minVarXL) {
          minVarXL = offsetVarXL[i];
          minIDXL = i;
        }
        if (cellPointNumXR[i] > 0 && offsetVarXR[i] < minVarXR) {
          minVarXR = offsetVarXR[i];
          minIDXR = i;
        }
      }

      intAL = (1 - 0.05) * intAL + 0.05 * offsetMeanXL[minIDXL];
      intAR = (1 - 0.05) * intAR + 0.05 * offsetMeanXR[minIDXR];

      ROS_INFO ("Ori point nmumber: %d %d %d %d %d", oriPointNum, minIDXL, minIDXR, 
                                                     cellPointNumXL[minIDXL], cellPointNumXR[minIDXR]);

      for (int i = 0; i < oriPointNum; i++) {
        cvCircle(image, cvPoint(240 - 20 * oriPointX[i], 400 - 20 * oriPointZ[i]), 1, CV_RGB(0, 0, 255), 1);
      }
      cvLine(image, cvPoint(240 - 20 * bestA, 400), cvPoint(240 - 20 * (10 * bestB + bestA), 400 - 20 * 10), CV_RGB(0, 255, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intAL, 400), cvPoint(240 - 20 * (10 * intB + intAL), 400 - 20 * 10), CV_RGB(255, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intAR, 400), cvPoint(240 - 20 * (10 * intB + intAR), 400 - 20 * 10), CV_RGB(255, 0, 0), 2);
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(255, 0, 0), 2);
      cvShowImage("Laser Points", image);
      for (int i = 0; i < oriPointNum; i++) {
        cvCircle(image, cvPoint(240 - 20 * oriPointX[i], 400 - 20 * oriPointZ[i]), 1, CV_RGB(0, 0, 0), 1);
      }
      cvLine(image, cvPoint(240 - 20 * bestA, 400), cvPoint(240 - 20 * (10 * bestB + bestA), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intAL, 400), cvPoint(240 - 20 * (10 * intB + intAL), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intAR, 400), cvPoint(240 - 20 * (10 * intB + intAR), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
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
