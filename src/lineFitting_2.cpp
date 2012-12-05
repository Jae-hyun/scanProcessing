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

int oriPointNum = 0;
const int MAXORIPOINTNUM = 10000;
float oriPointX[MAXORIPOINTNUM];
float oriPointY[MAXORIPOINTNUM];
float oriPointZ[MAXORIPOINTNUM];

//const int cellNumByZ = 200;
//const double scaleByZ = 10;
//const int maxInlierCountByZ = 50;
//int inlierCountByZ[200];

/*const int cellNumX = 100;
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
double offsetVarXR[cellNumY];*/

pcl::PointCloud<pcl::PointXYZ> pointCloud;
int totalPointNum = 0;
pcl::PointCloud<pcl::PointXYZ> pointCloudDS;
int totalPointNumDS = 0;

bool newLaserPoints = false;

float vehXCur, vehZCur, vehHeadingCur;
float vehXLast, vehZLast, vehHeadingLast;

bool newEncoderOdometry = false;

void myEncoderOdometryHandler(const nav_msgs::Odometry::ConstPtr& encoderOdometry)
{
  vehXCur = encoderOdometry->pose.pose.position.y;
  vehZCur = encoderOdometry->pose.pose.position.x;
  vehHeadingCur = tf::getYaw(encoderOdometry->pose.pose.orientation);
  
  newEncoderOdometry = true;
}

void myLaserPointsHandler(const sensor_msgs::PointCloud2ConstPtr& laserPoints) 
{
  pcl::fromROSMsg(*laserPoints, pointCloud);
  totalPointNum = pointCloud.points.size();
}

void myLaserPointsHandlerDS(const sensor_msgs::PointCloud2ConstPtr& laserPoints) 
{
  pcl::fromROSMsg(*laserPoints, pointCloudDS);
  oriPointNum = 0;
  totalPointNumDS = pointCloudDS.points.size();
  for (int i = 0; i < totalPointNumDS; i++) {
    if (pointCloudDS.points[i].y > 0.2 && pointCloudDS.points[i].y < 1.2 && pointCloudDS.points[i].x > -6 && pointCloudDS.points[i].x > -6 &&
        pointCloudDS.points[i].x < 3 + 2 * pointCloudDS.points[i].z && pointCloudDS.points[i].x > -3 - 2 * pointCloudDS.points[i].z && 
        oriPointNum < MAXORIPOINTNUM) {
      oriPointX[oriPointNum] = pointCloudDS.points[i].x;
      oriPointY[oriPointNum] = pointCloudDS.points[i].y;
      oriPointZ[oriPointNum] = pointCloudDS.points[i].z;
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
  ros::Subscriber subpcds = nh.subscribe<sensor_msgs::PointCloud2> ("/laser_points_Registered_DS", 1, myLaserPointsHandlerDS);

  IplImage* image = cvCreateImage(cvSize(480, 480), IPL_DEPTH_8U, 3);

  double intB = 0;
  double intA1 = 2;
  double intA2 = -2;
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserPoints) {
      newLaserPoints = false;

      if (oriPointNum < 10) {
        continue;
      }

      int bestInlerNum = 0;
      double bestA1 = 2, bestB1 = 0, bestA2 = -2, bestB2 = 0;
      int seedID1, seedID2, seedID3;
      double seedX1, seedZ1, seedX2, seedZ2, seedX3, seedZ3;
      double lineA1, lineB1, lineA2, lineB2;
      double dis1, dis2, den, den1, den2;
      double sumx1, sumxz1, sumz1, sumzz1;
      double sumx2, sumxz2, sumz2, sumzz2;
      int inlierNum, inlierCount, inlierNum1, inlierNum2, indByZ;
      for (int iterNum = 0; iterNum < 600; iterNum++) {
        seedID1 = rand() % oriPointNum;
        seedX1 = oriPointX[seedID1];
        seedZ1 = oriPointZ[seedID1];

        seedID2 = rand() % oriPointNum;
        if (seedID2 == seedID1) {
          seedID2 = (seedID2 + 1) % oriPointNum;
        }
        seedX2 = oriPointX[seedID2];
        seedZ2 = oriPointZ[seedID2];

        seedID3 = rand() % oriPointNum;
        if (seedID3 == seedID1 || seedID3 == seedID2) {
          seedID3 = (seedID3 + 1) % oriPointNum;
        }
        if (seedID3 == seedID1 || seedID3 == seedID2) {
          seedID3 = (seedID3 + 1) % oriPointNum;
        }
        seedX3 = oriPointX[seedID3];
        seedZ3 = oriPointZ[seedID3];

        lineB1 = (seedX1 - seedX2) / (seedZ1 - seedZ2);
        lineA1 = seedX1 - lineB1 * seedZ1;
        lineB2 = lineB1;
        lineA2 = seedX3 - lineB2 * seedZ3;

        if (!(lineA1 * lineA2 < 0 && lineA1 - lineA2 < 8 && lineA1 - lineA2 > -8 &&
            (lineA1 - lineA2 > 2 || lineA1 - lineA2 < -2) && lineA1 < 6 && lineA1 > -6 && 
            lineA2 < 6 && lineA2 > -6 && lineB1 < 2 && lineB1 > -2 && lineB2 < 2 && lineB2 > -2)) {
          continue;
        }

        inlierNum1 = 0, inlierNum2 = 0;
        sumx1 = 0; sumxz1 = 0; sumz1 = 0; sumzz1 = 0; 
        sumx2 = 0; sumxz2 = 0; sumz2 = 0; sumzz2 = 0; 
        den = sqrt(lineB1 * lineB1 + 1);
        for (int i = 0; i < oriPointNum; i++) {
          dis1 = abs(lineB1 * oriPointZ[i] - oriPointX[i] + lineA1) / den;
          dis2 = abs(lineB2 * oriPointZ[i] - oriPointX[i] + lineA2) / den;

          if (dis1 < 0.5 && dis1 < dis2) {
            sumz1 += oriPointZ[i];
            sumzz1 += oriPointZ[i] * oriPointZ[i];
            sumx1 += oriPointX[i];
            sumxz1 += oriPointZ[i] * oriPointX[i];
            inlierNum1++;
          } else if (dis2 < 0.5 && dis2 < dis1) {
            sumz2 += oriPointZ[i];
            sumzz2 += oriPointZ[i] * oriPointZ[i];
            sumx2 += oriPointX[i];
            sumxz2 += oriPointZ[i] * oriPointX[i];
            inlierNum2++;
          }
        }

        lineB1 = (inlierNum1 * sumxz1 - sumx1 * sumz1) / (inlierNum1 * sumzz1 - sumz1 * sumz1);
        lineA1 = (sumx1 * sumzz1 - sumz1 * sumxz1) / (inlierNum1 * sumzz1 - sumz1 * sumz1);
        lineB2 = (inlierNum2 * sumxz2 - sumx2 * sumz2) / (inlierNum2 * sumzz2 - sumz2 * sumz2);
        lineA2 = (sumx2 * sumzz2 - sumz2 * sumxz2) / (inlierNum2 * sumzz2 - sumz2 * sumz2);

        lineB1 = (lineB1 + lineB2) / 2;
        lineB2 = lineB1;
        if (lineA1 < lineA2) {
          double tempA = lineA1;
          lineA1 = lineA2;
          lineA2 = tempA;
        }

        inlierNum = 0;
        den1 = sqrt(lineB1 * lineB1 + 1);
        den2 = sqrt(lineB2 * lineB2 + 1);
        //for (int i = 0; i < cellNumByZ; i++) {
        //  inlierCountByZ[i] = 0;
        //}
        for (int i = 0; i < oriPointNum; i++) {
          dis1 = abs(lineB1 * oriPointZ[i] - oriPointX[i] + lineA1) / den1;
          dis2 = abs(lineB2 * oriPointZ[i] - oriPointX[i] + lineA2) / den2;
          if (dis1 < 0.5 && dis1 < dis2) {
            //indByZ = scaleByZ * sqrt(oriPointX[i] * oriPointX[i] + oriPointZ[i] * oriPointZ[i] - dis1 * dis1);
            //if (indByZ > cellNumByZ - 1) {
            //  indByZ = cellNumByZ - 1;
            //}
            //inlierCountByZ[indByZ]++;
            inlierNum++;
          } else if (dis2 < 0.5 && dis2 < dis1) {
            //indByZ = scaleByZ * sqrt(oriPointX[i] * oriPointX[i] + oriPointZ[i] * oriPointZ[i] - dis2 * dis2);
            //if (indByZ > cellNumByZ - 1) {
            //  indByZ = cellNumByZ - 1;
            //}
            //inlierCountByZ[indByZ]++;
            inlierNum++;
          }
        }
        //inlierNum = 0;
        //for (int i = 2; i < cellNumByZ - 2; i++) {
        //  inlierCount = inlierCountByZ[i - 2] + inlierCountByZ[i - 1] + inlierCountByZ[i] + inlierCountByZ[i + 1] + inlierCountByZ[i + 2];
        //  if (inlierCount > maxInlierCountByZ) {
        //    inlierCount = maxInlierCountByZ;
        //  }
        //  inlierNum += inlierCount;
        //}
        //inlierNum /= 5;

        if (bestInlerNum < inlierNum && lineA1 * lineA2 < 0 && lineA1 - lineA2 < 8 && lineA1 - lineA2 > -8 &&
            (lineA1 - lineA2 > 2 || lineA1 - lineA2 < -2) && lineA1 < 6 && lineA1 > -6 && lineA2 < 6 && lineA2 >-6 && 
            lineB1 < 2 && lineB1 > -2 && lineB2 < 2 && lineB2 > -2) {
          bestB1 = lineB1;
          bestB2 = lineB2;
          bestInlerNum = inlierNum;
        }

        if (0.5 * bestInlerNum < inlierNum && bestB1 - lineB1 < 0.1 && bestB1 - lineB1 > -0.1 &&
            lineA1 * lineA2 < 0 && lineA1 - lineA2 < 8 && lineA1 - lineA2 > -8 && (lineA1 - lineA2 > 2 || lineA1 - lineA2 < -2) && 
            lineA1 < 6 && lineA1 > -6 && lineA2 < 6 && lineA2 >-6 && 
            lineB1 < 2 && lineB1 > -2 && lineB2 < 2 && lineB2 > -2 && iterNum > 300) {
          if (bestA1 > lineA1) {
            bestA1 = lineA1;
          }
          if (bestA2 < lineA2) {
            bestA2 = lineA2;
          }
        }
      }

      if (newEncoderOdometry) {
        newEncoderOdometry = false;
        intB = tan(atan(intB) - vehHeadingCur + vehHeadingLast);
        vehXLast = vehXCur;
        vehZLast = vehZCur;
        vehHeadingLast = vehHeadingCur;
      }
      if (intB - bestB1 < 0.5 && intB - bestB1 > -0.5 && 
          intA1 - bestA1 < 0.5 && intA1 - bestA1 > -0.5 && 
          intA2 - bestA2 < 0.5 && intA2 - bestA2 > -0.5) {
        intB = (1 - 0.05) * intB + 0.05 * bestB1;
        intA1 = (1 - 0.2) * intA1 + 0.2 * bestA1;
        intA2 = (1 - 0.2) * intA2 + 0.2 * bestA2;
      } else {
        intB = (1 - 0.005) * intB + 0.005 * bestB1;
        intA1 = (1 - 0.02) * intA1 + 0.02 * bestA1;
        intA2 = (1 - 0.02) * intA2 + 0.02 * bestA2;
      }

      ROS_INFO ("Total: %d DS: %d Ori: %d Inliers: %d", totalPointNum, totalPointNumDS, oriPointNum, inlierNum);

      for (int i = 0; i < oriPointNum; i++) {
        cvCircle(image, cvPoint(240 - 20 * oriPointX[i], 400 - 20 * oriPointZ[i]), 1, CV_RGB(0, 0, 255), 1);
      }
      cvLine(image, cvPoint(240 - 20 * bestA1, 400), cvPoint(240 - 20 * (10 * bestB1 + bestA1), 400 - 20 * 10), CV_RGB(0, 255, 0), 2);
      cvLine(image, cvPoint(240 - 20 * bestA2, 400), cvPoint(240 - 20 * (10 * bestB2 + bestA2), 400 - 20 * 10), CV_RGB(0, 255, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intA1, 400), cvPoint(240 - 20 * (10 * intB + intA1), 400 - 20 * 10), CV_RGB(255, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intA2, 400), cvPoint(240 - 20 * (10 * intB + intA2), 400 - 20 * 10), CV_RGB(255, 0, 0), 2);
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(255, 0, 0), 2);
      cvShowImage("Laser Points", image);
      for (int i = 0; i < oriPointNum; i++) {
        cvCircle(image, cvPoint(240 - 20 * oriPointX[i], 400 - 20 * oriPointZ[i]), 1, CV_RGB(0, 0, 0), 1);
      }
      cvLine(image, cvPoint(240 - 20 * bestA1, 400), cvPoint(240 - 20 * (10 * bestB1 + bestA1), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * bestA2, 400), cvPoint(240 - 20 * (10 * bestB2 + bestA2), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intA1, 400), cvPoint(240 - 20 * (10 * intB + intA1), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 20 * intA2, 400), cvPoint(240 - 20 * (10 * intB + intA2), 400 - 20 * 10), CV_RGB(0, 0, 0), 2);
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(0, 0, 0), 2);

/*      int seedID1, seedID2;
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
