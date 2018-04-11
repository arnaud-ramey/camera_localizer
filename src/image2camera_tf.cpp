/*!
  \file        speed_calibration.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/10

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
 */
// C++
#include <fstream>
// OpenCV
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
// ROS
#include <cv_bridge/cv_bridge.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
// our packages
#include <camera_localizer/yaml2calib.h>
#include <vision_utils/string_split.h>
#include <vision_utils/print_point.h>

geometry_msgs::TransformStamped transform;
std::vector<cv::Point2f> pixels;

void mouse_cb(int event, int x, int y, int /*flags*/, void* /*userdata*/) {
  if (event == CV_EVENT_LBUTTONDOWN)
    pixels.push_back(cv::Point2f(x, y));
}

bool calib_from_cam_and_exit(const std::string & configfile,
                             cv::Mat3b & rgb) {
  ros::NodeHandle nh_public, nh_private("~");
  double cell_width = 1;
  nh_private.param("cell_width", cell_width, cell_width);
  std::string worldxyz_str;
  std::vector<double> worldxyz;
  nh_private.param("worldxyz", worldxyz_str, worldxyz_str);
  vision_utils::StringSplit_<double>(worldxyz_str,",",&worldxyz);
  if (worldxyz.size() % 3 != 0) {
    ROS_FATAL("You must indicate triplets of coordinates, we got %li coordinates",
              worldxyz.size());
    return false;
  }
  std::vector<cv::Point3f> world_pos;
  unsigned int npts = worldxyz.size() / 3;
  if (npts < 4) {
    ROS_FATAL("You must specify at least 4 pts, we got %i", npts);
    return false;
  }
  for (unsigned int i = 0; i < npts; ++i) {
    cv::Point3f newpt(worldxyz[3*i], worldxyz[3*i+1], worldxyz[3*i+2]);
    world_pos.push_back(newpt);
    ROS_INFO("Adding %s", vision_utils::print_point(newpt).c_str());
  } // end for i

  // acquire the four corners
  cv::Mat viz;
  std::string winname = "image2camera_tf";
  cv::namedWindow(winname);
  cv::setMouseCallback(winname, mouse_cb);
  while(pixels.size() < world_pos.size()) {
    unsigned int ptidx = pixels.size();
    rgb.copyTo(viz);
    // draw previously clicked pts
    for (unsigned int i = 0; i < ptidx; ++i)
      cv::circle(viz, pixels[i], 3, CV_RGB(0, 255, 0), -1);
    // write caption
    std::ostringstream caption;
    caption << "Click on pt (" << world_pos[ptidx].x << "," << world_pos[ptidx].y << ")";
    cv::putText(viz, caption.str(), cv::Point(20, 20), CV_FONT_HERSHEY_PLAIN,
                1, CV_RGB(0, 255, 0));
    cv::imshow(winname, viz);
    char c = cv::waitKey(50);
    if (c == 27)
      return false;
  }

  // compute extrinsics with solvePnP()
  std::string calib_filename = ros::package::getPath("camera_localizer") + "/data/camcalib.yaml";
  nh_private.param("calib_filename", calib_filename, calib_filename);
  sensor_msgs::CameraInfo cam_info;
  image_geometry::PinholeCameraModel cam_model;
  cv::Mat intrinsics, distortion;
  if (!yaml2calib(calib_filename, cam_info, cam_model, intrinsics, distortion)) {
    return -1;
  }
  ROS_INFO_STREAM("Read from '" << calib_filename << "' distortion.t()="
                  << distortion.t() << ", intrinsics=" << std::endl << intrinsics);

  // solvePnP works with undistorted coordinates
  // source: http://stackoverflow.com/questions/34550499/undistort-image-before-estimating-pose-using-solvepnp
  cv::Mat rvec, tvec;
  if (!cv::solvePnP(world_pos, pixels, intrinsics, distortion, rvec, tvec)) {
    ROS_FATAL("cv::solvePnP() failed!");
    return false;
  }
  std::cout << "rvec:" << rvec.t() << ", type:" << rvec.type() << std::endl;
  std::cout << "tvec:" << tvec.t() << ", type:" << tvec.type() << std::endl;
  // type 5 = CV_32F
  assert(rvec.type() = CV_32F);
  assert(tvec.type() = CV_32F);

  // convert to rotation matrix
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  assert(rmat.type() = CV_32F);
  //std::cout << "rmat:" << rmat << std::endl;
  tf::Matrix3x3 mat(rmat.at<float>(0), rmat.at<float>(1), rmat.at<float>(2),
                    rmat.at<float>(3), rmat.at<float>(4), rmat.at<float>(5),
                    rmat.at<float>(6), rmat.at<float>(7), rmat.at<float>(8));
  // convert to quaternion
  tf::Quaternion q;
  mat.getRotation(q);
  q.normalize();

  // covnert to transform
  // http://stackoverflow.com/questions/36561593/opencv-rotation-rodrigues-and-translation-vectors-for-positioning-3d-object-in
  transform.header.frame_id = "/camera_frame";
  transform.child_frame_id = "/world";
  transform.transform.translation.x = tvec.at<float>(0);
  transform.transform.translation.y = tvec.at<float>(1);
  transform.transform.translation.z = tvec.at<float>(2);
  tf::quaternionTFToMsg(q, transform.transform.rotation);

  // save to file
  ROS_INFO("Saving TF to file '%s'.", configfile.c_str());
  std::ofstream myfile;
  myfile.open (configfile.c_str());
  if (!myfile.is_open()) {
    ROS_FATAL("Could not open '%s'", configfile.c_str());
    return false;
  }
  myfile << transform.header.frame_id << std::endl <<
            transform.child_frame_id << std::endl <<
            transform.transform.translation.x << std::endl <<
            transform.transform.translation.y << std::endl <<
            transform.transform.translation.z << std::endl <<
            transform.transform.rotation.x << std::endl <<
            transform.transform.rotation.y << std::endl <<
            transform.transform.rotation.z << std::endl <<
            transform.transform.rotation.w;
  myfile.close();

  // draw an illustration image
  rgb.copyTo(viz); // new frame
  // draw a regular grid
  world_pos.clear();
  for (double x = 0; x < cell_width; x+=.1)
    for (double y = 0; y < cell_width; y+=.1)
      world_pos.push_back(cv::Point3f(x, y, 0));
  unsigned int w = sqrt(world_pos.size());
  pixels.clear();
  cv::projectPoints(world_pos, rvec, tvec, intrinsics, distortion, pixels);
  cv::drawChessboardCorners(viz, cv::Size(w, w), pixels, true);
  // draw a caption
  std::ostringstream caption;
  caption << "Computed pos: (" << std::setprecision(3)
          << tvec.at<float>(0) << "," << tvec.at<float>(1) << "," << tvec.at<float>(2)
          << "). Press a key to continue";
  cv::putText(viz, caption.str(), cv::Point(20, 20), CV_FONT_HERSHEY_PLAIN,
              1, CV_RGB(0, 255, 0));
  ROS_INFO("%s", caption.str().c_str());
  // show
  cv::imshow(winname, viz);
  cv::waitKey(0);
  cv::destroyAllWindows();
  return true;
} // end from_cam

////////////////////////////////////////////////////////////////////////////////

bool load_from_file(const std::string & configfile) {
  std::ifstream myfile (configfile.c_str());
  if (!myfile.is_open()) {
    ROS_FATAL("Could not open '%s'", configfile.c_str());
    return false;
  }
  std::string line;
  std::vector<std::string> lines;
  while ( getline (myfile,line) )
    lines.push_back(line);
  myfile.close();
  if (lines.size() != 9) {
    ROS_FATAL("Expected 9 lines in '%s', got %li",
              configfile.c_str(), lines.size());
    return false;
  }
  transform.header.frame_id = lines[0];
  transform.child_frame_id = lines[1];
  transform.transform.translation.x = atof(lines[2].c_str());
  transform.transform.translation.y = atof(lines[3].c_str());
  transform.transform.translation.z = atof(lines[4].c_str());
  transform.transform.rotation.x = atof(lines[5].c_str());
  transform.transform.rotation.y = atof(lines[6].c_str());
  transform.transform.rotation.z = atof(lines[7].c_str());
  transform.transform.rotation.w = atof(lines[8].c_str());
  return true;
} // end from_file()

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "image2camera_tf");
  std::string configfile = ros::package::getPath("camera_localizer") + "/config/camera2world_param.txt";
  ros::NodeHandle nh_public, nh_private("~");
  bool reset = true;
  nh_private.param("reset", reset, reset);
  if (reset) { // calib from cam and save
    cv::Mat3b rgb;
    std::string img_topic = "";
    int camera_index = -1;
    nh_private.param("img_topic", img_topic, img_topic);
    nh_private.param("camera_index", camera_index, camera_index);
    if (img_topic.size()) {
      ROS_INFO("Waiting for image on topic '%s'...", img_topic.c_str());
      sensor_msgs::ImageConstPtr rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>
          (img_topic, nh_public, ros::Duration(1));
      if (!rgb_msg) {
        ROS_FATAL("could not obtain an image on '%s'!\n",
                  img_topic.c_str());
        return -1;
      }
      cv_bridge::CvImageConstPtr rgb_bridge = cv_bridge::toCvShare(rgb_msg);
      rgb_bridge->image.copyTo(rgb);
    } else if (camera_index >= 0){ // try to open CV camera
      cv::VideoCapture cap(camera_index); // open the default camera
      assert(cap.isOpened());  // check if we succeeded
      cap >> rgb; // get a new frame from camera
    }
    else { // fail
      ROS_FATAL("You need to specify either 'img_topic' or 'camera_index' parameters");
      return -1;
    }

    if (rgb.empty()) {
      ROS_FATAL("Empty image, corrupted?");
      return -1;
    }
    return (calib_from_cam_and_exit(configfile, rgb) ? 0 : -1);
  }
  if (!load_from_file(configfile))
    return -1;
  tf::TransformBroadcaster br;
  ros::Rate rate(50);
  while(ros::ok()) {
    transform.header.stamp = ros::Time::now();
    br.sendTransform(transform);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
