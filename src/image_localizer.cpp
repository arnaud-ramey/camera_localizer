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
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
// ROS msgs
#include <geometry_msgs/PointStamped.h>
// ROS
#include <ros/node_handle.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_localizer/yaml2calib.h>

bool display = true;
double learningRate = 0;
std::string bg_filename;
// camera
image_geometry::PinholeCameraModel cam_model;
cv::Mat mapx, mapy, intrinsics, distortion;
// http://docs.opencv.org/master/d1/dc5/tutorial_background_subtraction.html#gsc.tab=0
// create background substractor and set the background we loaded
cv::Ptr<cv::BackgroundSubtractor> bgsub = //cv::createBackgroundSubtractorKNN();
    cv::createBackgroundSubtractorMOG2();
cv::Mat3b frame, frame_rect, viz;
cv::Mat bg, fg, fg_thres;
std::vector<cv::KeyPoint> keypoints;
// http://www.learnopencv.com/blob-detection-using-opencv-python-c/
cv::Ptr<cv::SimpleBlobDetector> blob_detec;
// ROS
cv_bridge::CvImageConstPtr frame_bridge;
ros::Publisher pt_pub;
geometry_msgs::PointStamped msg;
cv::Point2f prev_pt;
bool prev_pt_set = false;

////////////////////////////////////////////////////////////////////////////////

void set_v4l_param(int fd, unsigned int id, int value, const std::string & descr) {
  v4l2_control c;
  c.id = id;
  c.value = value;
  printf("Retval for %s:%i\n", descr.c_str(), v4l2_ioctl(fd, VIDIOC_S_CTRL, &c));
}

////////////////////////////////////////////////////////////////////////////////

template<class P2a, class P2b>
inline double dist2sq(const P2a & a, const P2b & b) {
  double dx = a.x-b.x, dy = a.y-b.y;
  return dx * dx + dy * dy;
}

////////////////////////////////////////////////////////////////////////////////

bool process_frame(const cv::Mat3b & frame) {
  ros::Time begin_time = ros::Time::now();
  if (bgsub->empty()) { // set background from frame
    bgsub->apply(frame_rect, fg, 1);
  }

  // rectify image
  cv::remap(frame, frame_rect, mapx, mapy, cv::INTER_LINEAR);

  // find object by differenciation - do not update model
  bgsub->apply(frame_rect, fg, learningRate);
  cv::threshold(fg, fg_thres, 250, 255, CV_THRESH_BINARY);

  // find biggest blob
  keypoints.clear();
  double white_rate = 1. * cv::countNonZero(fg_thres) / (fg.cols * fg.rows);
  if (white_rate > .05) {
    ROS_WARN_THROTTLE(1, "Foreground too white, you should reset it!");
  }
  else {
    blob_detec->detect(fg_thres, keypoints);
    ROS_INFO_THROTTLE(5, "%li keypoints found in %i ms",
                      keypoints.size(),
                      (int) (1000 * (ros::Time::now() - begin_time).toSec()));
  }

  // convert pixel to point
  cv::Point2f curr_pt;
  bool curr_pt_set = false;
  unsigned int nkp = keypoints.size();
  if (nkp == 1) {
    curr_pt_set = true;
    curr_pt = keypoints.front().pt;
  }
  else if (nkp >= 2 && prev_pt_set) { // find closest keypoint from prev_pt
    curr_pt_set = true;
    curr_pt = keypoints.front().pt;
    double best_dist = dist2sq(curr_pt, prev_pt);
    for (unsigned int i = 1; i < nkp; ++i) {
      double curr_dist = dist2sq(keypoints[i].pt, prev_pt);
      if (best_dist > curr_dist) {
        best_dist = curr_dist;
        curr_pt = keypoints[i].pt;
      }
    } // end for i
  }

  if (curr_pt_set) {
    prev_pt = curr_pt;
    prev_pt_set = true;
    cv::Point3d ray = cam_model.projectPixelTo3dRay(curr_pt);
    msg.header.stamp = begin_time;
    msg.point.x = ray.x;
    msg.point.y = ray.y;
    msg.point.z = ray.z;
    pt_pub.publish(msg);
    ros::spinOnce();
  }

  //
  // display
  //
  if (!display)
    return true;
  // Draw detected blobs as red circles.
  // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
  frame_rect.copyTo(viz);
  cv::drawKeypoints( viz, keypoints, viz,
                     cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );
  cv::drawKeypoints( viz, keypoints, viz,
                     cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  if (prev_pt_set)
    cv::circle(viz, prev_pt, 5, CV_RGB(0, 170, 0), -1);
  if (curr_pt_set)
    cv::circle(viz, curr_pt, 5, CV_RGB(0, 255, 0), -1);

  // display
  //cv::imshow("frame", frame);
  cv::imshow("viz", viz);
  cv::imshow("fg", fg);
  //bgsub->getBackgroundImage(bg); cv::imshow("bg", bg);
  char c = cv::waitKey(5);
  if (c == 's') {
    ROS_INFO("Saving background '%s'", bg_filename.c_str());
    cv::imwrite(bg_filename, frame_rect);
    // set background from frame
    bgsub->apply(frame_rect, fg, 1);
  }
  else if (c == 'q')
    return false;
  return true;
}

////////////////////////////////////////////////////////////////////////////////

void frame_cb(const sensor_msgs::Image::ConstPtr& rgb_msg) {
  // DEBUG_PRINT("frame_cb()\n");
  try {
    frame_bridge = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  const cv::Mat* frame_ptr = &(frame_bridge->image);
  process_frame(*frame_ptr);
} // end frame_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_localizer");
  ros::NodeHandle nh_public, nh_private("~");
  int camera_index = -1;
  std::string img_topic = "";

  // get params
  nh_private.param("display", display, display);
  nh_private.param("learningRate", learningRate, learningRate);
  nh_private.param("img_topic", img_topic, img_topic);
  nh_private.param("camera_index", camera_index, camera_index);

  // read calibration data
  std::string calib_filename = ros::package::getPath("camera_localizer") + "/data/camcalib.yaml";
  nh_private.param("calib_filename", calib_filename, calib_filename);
  sensor_msgs::CameraInfo cam_info;
  if (!yaml2calib(calib_filename, cam_info, cam_model, intrinsics, distortion)) {
    return -1;
  }

  // Computes the undistortion and rectification transformation map.
  // cf http://docs.opencv.org/trunk/modules/imgproc/doc/geometric_transformations.html#initundistortrectifymap
  cv::Mat newCameraMatrix;
  cv::initUndistortRectifyMap(intrinsics, distortion, cv::Mat(), newCameraMatrix,
                              cv::Size(cam_info.width, cam_info.height), CV_32FC1,
                              mapx, mapy);

  // load background image
  bg_filename = ros::package::getPath("camera_localizer") + "/data/bg.png";
  bg = cv::imread(bg_filename, cv::IMREAD_COLOR);
  if (!bg.empty()) {
    ROS_INFO("Successfully loaded background '%s'", bg_filename.c_str());
    bgsub->apply(bg, fg, 1);
  }

  // blob detector
  // Setup SimpleBlobDetector parameters.
  cv::SimpleBlobDetector::Params params;
  // Change thresholds
  params.filterByColor = true;
  params.blobColor = 255;
  //params.minThreshold = 100;
  //params.maxThreshold = 256;
  // Filter by Area.
  params.filterByArea = true;
  params.minArea = 400;
  params.maxArea = 1E10;
  params.filterByCircularity = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;
  blob_detec = cv::SimpleBlobDetector::create(params);

  // set publisher
  pt_pub = nh_public.advertise<geometry_msgs::PointStamped>("robot_direction", 1);
  msg.header.frame_id = "camera_frame";

  // determine if using ROS image topic or OpenCV camera
  // OpenCV stuff
  cv::VideoCapture cap;
  // ROS stuff
  image_transport::Subscriber img_sub;
  image_transport::ImageTransport it(nh_public);


  // open corresponding camera
  if (img_topic.size()) {
    img_sub = it.subscribe(img_topic, 0, &frame_cb);
    ros::spin();

  } else if (camera_index >= 0){ // try to open V4L + OpenCV camera
    // configure camera with V4L2
    // http://www.linuxtv.org/downloads/legacy/video4linux/API/V4L2_API/spec-single/v4l2.html#camera-controls
    std::ostringstream devname; devname << "/dev/video" << camera_index;
    int fd = v4l2_open(devname.str().c_str(), O_RDWR);
    set_v4l_param(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL, "auto exposure disable");
    set_v4l_param(fd, V4L2_CID_EXPOSURE_ABSOLUTE, 300, "exposure control");
    set_v4l_param(fd, V4L2_CID_AUTO_WHITE_BALANCE, V4L2_WHITE_BALANCE_MANUAL, "auto wb disable");
    set_v4l_param(fd, V4L2_CID_WHITE_BALANCE_TEMPERATURE, 5000, "wb temp");

    cap.open(camera_index); // open the default camera
    //int w = 1280, h = 720;
    int w = cam_info.width, h = cam_info.height;
    printf("w:%i, h:%i\n", w, h);
    assert(cap.isOpened());  // check if we succeeded
    cap.set(CV_CAP_PROP_FRAME_WIDTH, w);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, h);

    // loop
    ros::Rate rate(20);
    while(ros::ok()) {
      cap >> frame; // get a new frame from camera
      process_frame(frame);
      rate.sleep();
    }
  }
  else { // fail
    ROS_FATAL("You need to specify either 'img_topic' or 'camera_index' parameters");
    return -1;
  }

  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}
