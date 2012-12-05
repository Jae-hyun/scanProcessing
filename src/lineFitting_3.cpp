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

const int cellNumX = 20;
const int cellNumY = 8;
const float cellScale = 10;
const int cellCenterXL = 5;
const int cellCenterXR = 15;
const int cellCenterY = 4;
const int totalCellNum = cellNumX * cellNumY;
int gridmapXYL[totalCellNum];
int gridmapXYR[totalCellNum];
int cellPointNumXL[cellNumY];
int cellPointNumXR[cellNumY];
float offsetMeanXL[cellNumY];
float offsetMeanXR[cellNumY];
float offsetVarXL[cellNumY];
float offsetVarXR[cellNumY];

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

  float intB = 0;
  float intA1 = 2, intA2 = -2;
  float intA1R = 2, intA2R = -2;
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserPoints) {
      newLaserPoints = false;

      if (oriPointNum < 10) {
        continue;
      }

      int bestInlerNum = 0;
      float bestA1 = 2, bestB1 = 0, bestA2 = -2, bestB2 = 0;
      int seedID1, seedID2, seedID3;
      float seedX1, seedZ1, seedX2, seedZ2, seedX3, seedZ3;
      float lineA1, lineB1, lineA2, lineB2;
      float dis1, dis2, den;
      float sumx1, sumxz1, sumz1, sumzz1;
      float sumx2, sumxz2, sumz2, sumzz2;
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
          dis1 = (lineB1 * oriPointZ[i] - oriPointX[i] + lineA1) / den;
          dis2 = (lineB2 * oriPointZ[i] - oriPointX[i] + lineA2) / den;
          if (dis1 * dis1 < dis2 * dis2 && dis1 < 1 && dis1 > -1) {
            sumz1 += oriPointZ[i];
            sumzz1 += oriPointZ[i] * oriPointZ[i];
            sumx1 += oriPointX[i];
            sumxz1 += oriPointZ[i] * oriPointX[i];
            inlierNum1++;
          } else if (dis2 * dis2 < dis1 * dis1 && dis2 < 1 && dis2 > -1) {
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
          float tempA = lineA1;
          lineA1 = lineA2;
          lineA2 = tempA;
        }

        inlierNum = 0;
        den = sqrt(lineB1 * lineB1 + 1);
        //for (int i = 0; i < cellNumByZ; i++) {
        //  inlierCountByZ[i] = 0;
        //}
        for (int i = 0; i < oriPointNum; i++) {
          dis1 = (lineB1 * oriPointZ[i] - oriPointX[i] + lineA1) / den;
          dis2 = (lineB2 * oriPointZ[i] - oriPointX[i] + lineA2) / den;
          if (dis1 * dis1 < dis2 * dis2 && dis1 < 1 && dis1 > -1) {
            //indByZ = scaleByZ * sqrt(oriPointX[i] * oriPointX[i] + oriPointZ[i] * oriPointZ[i] - dis1 * dis1);
            //if (indByZ > cellNumByZ - 1) {
            //  indByZ = cellNumByZ - 1;
            //}
            //inlierCountByZ[indByZ]++;
            inlierNum++;
          } else if (dis2 * dis2 < dis1 * dis1 && dis2 < 1 && dis2 > -1) {
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

      float dis;
      int cellIndX, cellIndY;
      den = sqrt(intB * intB + 1);
      float intA1den = intA1 / den;
      float intA2den = intA2 / den;
      for (int i = 0; i < totalCellNum; i++) {
        gridmapXYL[i] = 0;
        gridmapXYR[i] = 0;
      }
      for (int i = 0; i < totalPointNum; i++) {
        if (pointCloud.points[i].y > -0.4 && pointCloud.points[i].y < 0.4) {
          dis = (pointCloud.points[i].x - intB * pointCloud.points[i].z) / den;
          if (dis - intA1den > -0.5 && dis - intA1den < 1.5) {
            cellIndX = cellScale * (dis - intA1den) + cellCenterXL;
            cellIndY = cellScale * pointCloud.points[i].y + cellCenterY;
            if (cellIndX >= 0 && cellIndX < cellNumX && cellIndY >= 0 && cellIndY < cellNumY) {
              gridmapXYL[cellNumX * cellIndY + cellIndX]++;
            }
          } else if (dis - intA2den > -1.5 && dis - intA2den < 0.5) {
            cellIndX = cellScale * (dis - intA2den) + cellCenterXR;
            cellIndY = cellScale * pointCloud.points[i].y + cellCenterY;
            if (cellIndX >= 0 && cellIndX < cellNumX && cellIndY >= 0 && cellIndY < cellNumY) {
              gridmapXYR[cellNumX * cellIndY + cellIndX]++;
            }
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
        for (int j = 0; j < cellNumX; j++) {
          cellPointNumXL[i] += gridmapXYL[cellNumX * (i - 1) + j] + gridmapXYL[cellNumX * i + j] + gridmapXYL[cellNumX * (i + 1) + j];
          offsetMeanXL[i] += (gridmapXYL[cellNumX * (i - 1) + j] + gridmapXYL[cellNumX * i + j] + gridmapXYL[cellNumX * (i + 1) + j]) * j;
          cellPointNumXR[i] += gridmapXYR[cellNumX * (i - 1) + j] + gridmapXYR[cellNumX * i + j] + gridmapXYR[cellNumX * (i + 1) + j];
          offsetMeanXR[i] += (gridmapXYR[cellNumX * (i - 1) + j] + gridmapXYR[cellNumX * i + j] + gridmapXYR[cellNumX * (i + 1) + j]) * j;
        }
      }
      for (int i = 1; i < cellNumY - 1; i++) {
        if (cellPointNumXL[i] > 0) {
          offsetMeanXL[i] /= cellPointNumXL[i];
          for (int j = 0; j < cellNumX; j++) {
            offsetVarXL[i] += (gridmapXYL[cellNumX * (i - 1) + j] + gridmapXYL[cellNumX * i + j] + gridmapXYL[cellNumX * (i + 1) + j]) 
                            / double(cellPointNumXL[i]) * (j - offsetMeanXL[i]) * (j - offsetMeanXL[i]);
           }
        }
        if (cellPointNumXR[i] > 0) {
          offsetMeanXR[i] /= cellPointNumXR[i];
          for (int j = 0; j < cellNumX; j++) {
            offsetVarXR[i] += (gridmapXYR[cellNumX * (i - 1) + j] + gridmapXYR[cellNumX * i + j] + gridmapXYR[cellNumX * (i + 1) + j]) 
                            / double(cellPointNumXR[i]) * (j - offsetMeanXR[i]) * (j - offsetMeanXR[i]);
          }
        }
      }
      for (int i = 1; i < cellNumY - 1; i++) {
        offsetMeanXL[i] = (offsetMeanXL[i] - cellCenterXL) / cellScale * den;
        offsetMeanXR[i] = (offsetMeanXR[i] - cellCenterXR) / cellScale * den;
      }

      int minIDXL = -1, minIDXR = -1;
      float minVarXL = 100000;
      float minVarXR = 100000;
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

      if (minIDXL >= 1 && minIDXL < cellNumY - 1) {
        intA1R = (1 - 0.1) * intA1R + 0.1 * (offsetMeanXL[minIDXL] + intA1);
      }
      if (minIDXR >= 1 && minIDXR < cellNumY - 1) {
        intA2R = (1 - 0.1) * intA2R + 0.1 * (offsetMeanXR[minIDXR] + intA2);
      }

      ROS_INFO ("Total: %d DS: %d Ori: %d Inliers: %d IDL: %d IDR: %d", 
                totalPointNum, totalPointNumDS, oriPointNum, inlierNum, minIDXL, minIDXR);

      //for (int i = 0; i < totalPointNum; i++) {
      //  cvCircle(image, cvPoint(240 - 25 * pointCloud.points[i].x, 400 - 25 * pointCloud.points[i].y), 1, CV_RGB(0, 255, 255), 1);
      //}
      for (int i = 0; i < oriPointNum; i++) {
        cvCircle(image, cvPoint(240 - 25 * oriPointX[i], 400 - 25 * oriPointZ[i]), 1, CV_RGB(0, 0, 255), 1);
      }
      cvLine(image, cvPoint(240 - 25 * bestA1, 400), cvPoint(240 - 25 * (10 * bestB1 + bestA1), 400 - 25 * 10), CV_RGB(255, 255, 0), 2);
      cvLine(image, cvPoint(240 - 25 * bestA2, 400), cvPoint(240 - 25 * (10 * bestB2 + bestA2), 400 - 25 * 10), CV_RGB(255, 255, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA1, 400), cvPoint(240 - 25 * (10 * intB + intA1), 400 - 25 * 10), CV_RGB(0, 255, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA2, 400), cvPoint(240 - 25 * (10 * intB + intA2), 400 - 25 * 10), CV_RGB(0, 255, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA1R, 400), cvPoint(240 - 25 * (10 * intB + intA1R), 400 - 25 * 10), CV_RGB(255, 0, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA2R, 400), cvPoint(240 - 25 * (10 * intB + intA2R), 400 - 25 * 10), CV_RGB(255, 0, 0), 2);
      //if (minIDXL >= 1 && minIDXL < cellNumY - 1) {
      //  cvCircle(image, cvPoint(240 - 25 * (offsetMeanXL[minIDXL] + intA1), 400 - 25 / double(cellScale) * (minIDXL - cellCenterY)), 
      //           1, CV_RGB(255, 0, 0), 4);
      //}
      //if (minIDXR >= 1 && minIDXR < cellNumY - 1) {
      //  cvCircle(image, cvPoint(240 - 25 * (offsetMeanXR[minIDXR] + intA2), 400 - 25 / double(cellScale) * (minIDXR - cellCenterY)), 
      //           1, CV_RGB(255, 0, 0), 4);
      //}
      cvCircle(image, cvPoint(240, 400), 1, CV_RGB(255, 0, 0), 2);
      cvShowImage("Laser Points", image);
      //for (int i = 0; i < totalPointNum; i++) {
      //  cvCircle(image, cvPoint(240 - 25 * pointCloud.points[i].x, 400 - 25 * pointCloud.points[i].y), 1, CV_RGB(0, 0, 0), 1);
      //}
      for (int i = 0; i < oriPointNum; i++) {
        cvCircle(image, cvPoint(240 - 25 * oriPointX[i], 400 - 25 * oriPointZ[i]), 1, CV_RGB(0, 0, 0), 1);
      }
      cvLine(image, cvPoint(240 - 25 * bestA1, 400), cvPoint(240 - 25 * (10 * bestB1 + bestA1), 400 - 25 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 25 * bestA2, 400), cvPoint(240 - 25 * (10 * bestB2 + bestA2), 400 - 25 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA1, 400), cvPoint(240 - 25 * (10 * intB + intA1), 400 - 25 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA2, 400), cvPoint(240 - 25 * (10 * intB + intA2), 400 - 25 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA1R, 400), cvPoint(240 - 25 * (10 * intB + intA1R), 400 - 25 * 10), CV_RGB(0, 0, 0), 2);
      cvLine(image, cvPoint(240 - 25 * intA2R, 400), cvPoint(240 - 25 * (10 * intB + intA2R), 400 - 25 * 10), CV_RGB(0, 0, 0), 2);
      //if (minIDXL >= 1 && minIDXL < cellNumY - 1) {
      //  cvCircle(image, cvPoint(240 - 25 * (offsetMeanXL[minIDXL] + intA1), 400 - 25 / double(cellScale) * (minIDXL - cellCenterY)), 
      //           1, CV_RGB(0, 0, 0), 4);
      //}
      //if (minIDXR >= 1 && minIDXR < cellNumY - 1) {
      //  cvCircle(image, cvPoint(240 - 25 * (offsetMeanXR[minIDXR] + intA2), 400 - 25 / double(cellScale) * (minIDXR - cellCenterY)), 
      //           1, CV_RGB(0, 0, 0), 4);
      //}
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
