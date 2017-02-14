/*!
  \file        yaml2calib.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2017/2/4
________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */

#ifndef YAML2CALIB_H
#define YAML2CALIB_H

#include <camera_calibration_parsers/parse.h>
#include <image_geometry/pinhole_camera_model.h>

bool yaml2calib(const std::string & cal_filename,
                sensor_msgs::CameraInfo & caminfo,
                image_geometry::PinholeCameraModel & cam_model,
                cv::Mat & intrinsics,
                cv::Mat & distortion) {
  std::string cal_camname;
  if (!camera_calibration_parsers::readCalibration
      (cal_filename, cal_camname, caminfo)) {
    ROS_FATAL("Could not read camera '%s' in camera_calibration file '%s'",
              cal_camname.c_str(), cal_filename.c_str());
    return false;
  }
  if (!cam_model.fromCameraInfo(caminfo)) {
    ROS_FATAL("Could not convert CameraInf --> PinholeCameraModel");
    return false;
  }
  // Read camera parameters from CamerInfo
  //  printf("K:%i, D:%i\n", caminfo.K.size(), caminfo.D.size());
  //  std::cout << "intrinsics:" << intrinsics << std::endl;
  //  std::cout << "distortion:" << distortion << std::endl;
  intrinsics.create(3, 3, CV_32FC1);
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col)
      intrinsics.at<float>(row, col) = caminfo.K[3*row+col];
  distortion.create(5, 1, CV_32FC1); // rows, cols
  for (int col = 0; col < 5; ++col)
    distortion.at<float>(col, 0) = caminfo.D[col];
  return true;
}

#endif // YAML2CALIB_H
