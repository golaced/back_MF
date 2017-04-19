/*
*  flow_opencv.cpp
*
*  Created on: Dec 13, 2016
*      Author: Christoph
*/

#include "flow_opencv.hpp"

/****************************************************************************
 * OpenCV optical flow calculation
 ****************************************************************************/

 OpticalFlowOpenCV::OpticalFlowOpenCV( float f_length_x, float f_length_y, int num_feat, float conf_multi ) :
   num_features(num_feat),
   confidence_multiplier(conf_multi)
{
   setFocalLengthX(f_length_x);
   setFocalLengthY(f_length_y);
 }

OpticalFlowOpenCV::~OpticalFlowOpenCV( void )
{

}

int OpticalFlowOpenCV::calcFlow(const cv::Mat &img_current, float &flow_x, float &flow_y) {

  if (updateVector.empty())
    updateVector.resize(num_features, 2);

  int meancount = 0;
	float pixel_flow_x_mean = 0.0;
	float pixel_flow_y_mean = 0.0;
	float pixel_flow_x_stddev = 0.0;
	float pixel_flow_y_stddev = 0.0;

	trackFeatures( img_current, img_current, features_current, useless, updateVector, 0 );

  //TODO undistort points? not necessary if small field of view?

  if ( !features_current.empty() && !features_previous.empty() ) {
    //calculate pixel flow
    for ( int i = 0; i < updateVector.size(); i++ ) {
      //just use active features
      if (updateVector[i] == 1) {
        pixel_flow_x_mean += features_current[i].x - features_previous[i].x;
        pixel_flow_y_mean += features_current[i].y - features_previous[i].y;
        meancount++;
      }
    }
    //check if there are active features
    if (meancount) {
      pixel_flow_x_mean /= meancount;
      pixel_flow_y_mean /= meancount;

      //calculate variance
      for ( int i = 0; i < updateVector.size(); i++ ) {
        if (updateVector[i] == 1) {
          pixel_flow_x_stddev += pow(features_current[i].x - features_previous[i].x - pixel_flow_x_mean, 2);
          pixel_flow_y_stddev += pow(features_current[i].y - features_previous[i].y - pixel_flow_y_mean, 2);
        }
      }
      //convert to standard deviation
      pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev / meancount);
      pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev / meancount);

      //recalculate pixel flow with 90% confidence interval
      float temp_flow_x_mean = 0.0;
      float temp_flow_y_mean = 0.0;
      meancount = 0;

      for ( int i = 0; i < updateVector.size(); i++ ) {
        //check if active
        if ( updateVector[i] == 1 ) {
          //flow of feature i
          float temp_flow_x = features_current[i].x - features_previous[i].x;
          float temp_flow_y = features_current[i].y - features_previous[i].y;
          //check if inside confidence interval

          if ( fabs(temp_flow_x - pixel_flow_x_mean) < pixel_flow_x_stddev*confidence_multiplier &&
               fabs(temp_flow_y - pixel_flow_y_mean) < pixel_flow_y_stddev*confidence_multiplier ) {
            temp_flow_x_mean += temp_flow_x;
            temp_flow_y_mean += temp_flow_y;
            meancount++;
          } else {
            updateVector[i] = 0;
          }
        }
      }
      if (meancount) {
        //new mean
        pixel_flow_x_mean = temp_flow_x_mean / meancount;
        pixel_flow_y_mean = temp_flow_y_mean / meancount;
      }
    }
  }

  //remember features
  features_previous = features_current;
  //update feature status
  for (int i = 0; i < updateVector.size(); i++) {
    //new and now active
    if (updateVector[i] == 2) {
      updateVector[i] = 1;
    }
    //inactive
    if (updateVector[i] == 0) {
      updateVector[i] = 2;
    }
  }

  //output
  flow_x = atan2(pixel_flow_x_mean, focal_length_x);
  flow_y = atan2(pixel_flow_y_mean, focal_length_y);

  int flow_quality = round(255.0 * meancount / updateVector.size());

  return flow_quality;
}

#include <string.h>
#include <unistd.h>
#include "aruco/aruco.h"
//cv::Mat_<float> cam_matrix_flow(3, 3);
//cv::Mat_<float> distortion_flow(1, 4);
int OpticalFlowOpenCV::calcFlow_dis(const cv::Mat &img_current, float &flow_x, float &flow_y) {

//    if (!img_timestamp_prev) {
//		img_timestamp_prev = img_timestamp;
//		return;
//	}

    std::vector<cv::Point2f> useless;
    int meancount = 0;

    double pixel_flow_x_mean = 0.0;
    double pixel_flow_y_mean = 0.0;
    double pixel_flow_x_integral = 0.0;
    double pixel_flow_y_integral = 0.0;
    double pixel_flow_x_stddev = 0.0;
    double pixel_flow_y_stddev = 0.0;

    trackFeatures(img_current, img_current, features_current, useless, updateVector, 0);
    aruco::CameraParameters cameraParams;
    string cameraParamFileName("/home/pi/QT/MF/rasp.yml");
    cameraParams.readFromXMLFile(cameraParamFileName);
//    cam_matrix_flow <<   cameraParams.CameraMatrix.at[0], cameraParams.CameraMatrix.at[1], cameraParams.CameraMatrix[0][2],
//           cameraParams.CameraMatrix[1][0], cameraParams.CameraMatrix[1][1], cameraParams.CameraMatrix[1][2],
//           cameraParams.CameraMatrix[2][0], cameraParams.CameraMatrix[2][1], cameraParams.CameraMatrix[2][2];
//    distortion_flow <<   cameraParams.Distortion[0], cameraParams.Distortion[1], 0, 0,
//           cameraParams.Distortion[2];
//    cv::Mat_<float> cam_matrix(3, 3);
//    cam_matrix <<   cameraParams.CameraMatrix[0][0], cameraParams.CameraMatrix[0][1], cameraParams.CameraMatrix[0][2],
//           cameraParams.CameraMatrix[1][0], cameraParams.CameraMatrix[1][1], cameraParams.CameraMatrix[1][2],
//           cameraParams.CameraMatrix[2][0], cameraParams.CameraMatrix[2][1], cameraParams.CameraMatrix[2][2];
//    cv::Mat_<float> distortion(1, 4);
//    distortion <<   cameraParams.RadialDistortion[0], cameraParams.RadialDistortion[1], 0, 0,
//           cameraParams.RadialDistortion[2];

    int npoints = updateVector.size();
    cv::Mat_<cv::Point2f> out_features_current(1, npoints);
   static cv::Mat_<cv::Point2f> out_features_previous(1, npoints);
    cv::undistortPoints(features_current, out_features_current, cameraParams.CameraMatrix, cameraParams.Distorsion);

    // cv::undistortPoints returns normalized coordinates... -> convert
//    for (int i = 0; i < npoints; i++) {
//        out_features_current(i).x = out_features_current(i).x * cameraParams.CameraMatrix.at[0][0] +
//                        cameraParams.CameraMatrix.at[0][2];
//        out_features_current(i).y = out_features_current(i).y * cameraParams.CameraMatrix.at[1][1] +
//                        cameraParams.CameraMatrix.at[1][2];
//    }

    if (!out_features_current.empty() && !out_features_previous.empty()) {
        // compute the mean flow
        for (int i = 0; i < updateVector.size(); i++) {
            if (updateVector[i] == 1) {
                pixel_flow_x_mean += out_features_current(i).x - out_features_previous(i).x;
                pixel_flow_y_mean += out_features_current(i).y - out_features_previous(i).y;
                meancount++;
            }
        }

        if (meancount) {
            pixel_flow_x_mean /= meancount;
            pixel_flow_y_mean /= meancount;

            // compute the flow variance
            for (int i = 0; i < updateVector.size(); i++) {
                if (updateVector[i] == 1) {
                    pixel_flow_x_stddev += (out_features_current(i).x - out_features_previous(i).x - pixel_flow_x_mean) *
                                   (out_features_current(i).x - out_features_previous(i).x - pixel_flow_x_mean);
                    pixel_flow_y_stddev += (out_features_current(i).y - out_features_previous(i).y - pixel_flow_y_mean) *
                                   (out_features_current(i).y - out_features_previous(i).y - pixel_flow_y_mean);
                }
            }

            pixel_flow_x_stddev /= meancount;
            pixel_flow_y_stddev /= meancount;

            // convert to std deviation
            pixel_flow_x_stddev = sqrt(pixel_flow_x_stddev);
            pixel_flow_y_stddev = sqrt(pixel_flow_y_stddev);

            // re-compute the mean flow with only the 95% consenting features
            meancount = 0;

            for (int i = 0; i < updateVector.size(); i++) {
                if (updateVector[i] == 1) {
                    double this_flow_x = out_features_current(i).x - out_features_previous(i).x;
                    double this_flow_y = out_features_current(i).y - out_features_previous(i).y;

                    if (abs(this_flow_x - pixel_flow_x_mean) < 2 * pixel_flow_x_stddev
                        && abs(this_flow_y - pixel_flow_y_mean) < 2 * pixel_flow_y_stddev) {
                        pixel_flow_x_integral += out_features_current(i).x - out_features_previous(i).x;
                        pixel_flow_y_integral += out_features_current(i).y - out_features_previous(i).y;
                        meancount++;

                    } else {
                        updateVector[i] = 0;
                    }
                }

            }

            if (meancount) {
                pixel_flow_x_integral /= meancount;
                pixel_flow_y_integral /= meancount;

                double flow_quality = 255.0 * meancount / updateVector.size();
                //uint32_t delta_time = img_timestamp - img_timestamp_prev;

                double flow_x_ang = atan2(pixel_flow_x_integral, focal_length_x);
                double flow_y_ang = atan2(pixel_flow_y_integral, focal_length_y);
                flow_x=flow_x_ang;
                flow_y=flow_y_ang;

            }

        } else {
            printf("No valid measurements");
        }
    }

    for (int i = 0; i < updateVector.size(); i++) {
        if (updateVector[i] == 2) {
            updateVector[i] = 1;
        }

        if (updateVector[i] == 0) {
            updateVector[i] = 2;
        }
    }

    out_features_previous = out_features_current;
    //img_timestamp_prev = img_timestamp;

}
