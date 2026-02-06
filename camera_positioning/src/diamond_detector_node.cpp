#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <iostream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

rclcpp::Node::SharedPtr node;
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

cv::Ptr<cv::aruco::Dictionary> dictionary;
float squareLength;
float markerLength;
float axisLength;
double sample_number; // [s]
unsigned int current_sampling_number = 0;
std::string camera_topic;

std::string camera_base_frame;
std::string board_frame;
std::string base_frame;

std::vector<tf2::Vector3> pos_vect;
std::vector<tf2::Vector3> rpy_vect;
bool done = false;

using namespace std::chrono_literals;

void imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg, 
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & cam_info)
{
  auto opt_center_to_cam_base = cv::Matx44d::zeros();
  if(!camera_base_frame.empty())
  {
    if(!tf_buffer->canTransform(cam_info->header.frame_id, camera_base_frame, tf2::TimePointZero, 0ms))
    {
      RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *(node->get_clock()), 1000,
        "Waiting for transform " << camera_base_frame << " -> " << cam_info->header.frame_id);
      return;
    }
    else 
    {
      auto transform_msg = tf_buffer->lookupTransform(cam_info->header.frame_id, camera_base_frame, tf2::TimePointZero, 0ms);
      tf2::Stamped<tf2::Transform> transform_tf2;
      tf2::fromMsg(transform_msg, transform_tf2);
      tf2::Matrix3x3 fixed_rot = transform_tf2.getBasis();
      opt_center_to_cam_base = cv::Matx44d( fixed_rot[0][0], fixed_rot[0][1], fixed_rot[0][2], transform_tf2.getOrigin().getX(),
                                          fixed_rot[1][0], fixed_rot[1][1], fixed_rot[1][2], transform_tf2.getOrigin().getY(),
                                          fixed_rot[2][0], fixed_rot[2][1], fixed_rot[2][2], transform_tf2.getOrigin().getZ(),
                                          0,               0,               0,               1);
    }
  }

  auto base_frame_to_board = cv::Matx44d::eye();
  if(!base_frame.empty())
  {
    if(!tf_buffer->canTransform(base_frame, board_frame, tf2::TimePointZero, 0ms))
    {
      RCLCPP_WARN_STREAM_THROTTLE(node->get_logger(), *(node->get_clock()), 1000,
        "Waiting for transform " << base_frame << " -> " << board_frame);
      return;
    }
    else 
    {
      auto transform_msg = tf_buffer->lookupTransform(base_frame, board_frame, tf2::TimePointZero, 0ms);
      tf2::Stamped<tf2::Transform> transform_tf2;
      tf2::fromMsg(transform_msg, transform_tf2);
      tf2::Matrix3x3 fixed_rot = transform_tf2.getBasis();
      base_frame_to_board = cv::Matx44d( fixed_rot[0][0], fixed_rot[0][1], fixed_rot[0][2], transform_tf2.getOrigin().getX(),
                                          fixed_rot[1][0], fixed_rot[1][1], fixed_rot[1][2], transform_tf2.getOrigin().getY(),
                                          fixed_rot[2][0], fixed_rot[2][1], fixed_rot[2][2], transform_tf2.getOrigin().getZ(),
                                          0,               0,               0,               1);
    }
  }

  const cv::Mat cameraMatrix(3, 3, CV_64F, const_cast<double*>(cam_info->k.data()));
  const cv::Mat distCoeffs(5, 1, CV_64F, const_cast<double*>(cam_info->d.data()));

  cv::Mat image, imageCopy;
  image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  image.copyTo(imageCopy);

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners;
  cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds);
  /* If at least one has been detected */
  if (markerIds.size() > 0)
  {
      cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
  }

  std::vector<cv::Vec4i> diamondIds;
  std::vector<std::vector<cv::Point2f>> diamondCorners;

  cv::aruco::detectCharucoDiamond(image, markerCorners, markerIds, squareLength/markerLength, diamondCorners, diamondIds);
  
  if (diamondIds.size() > 0)
  {
      /* Estimate the poses */
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(diamondCorners, squareLength, cameraMatrix, distCoeffs, rvecs, tvecs);

      /* Draw axis */
      for (unsigned int i = 0; i < rvecs.size(); i++)
      {
          cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], axisLength);
          
          std::ostringstream vector_to_marker;
          vector_to_marker << std::setprecision(4)
                          << "x: " << std::setw(8) << tvecs[0](0);
          cv::putText(imageCopy, vector_to_marker.str(),
                      cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cvScalar(0, 0, 255), 1, cv::LINE_AA);

          vector_to_marker = std::ostringstream();
          vector_to_marker << std::setprecision(4)
                          << "y: " << std::setw(8) << tvecs[0](1);
          cv::putText(imageCopy, vector_to_marker.str(),
                      cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cvScalar(0, 0, 255), 1, cv::LINE_AA);

          vector_to_marker = std::ostringstream();
          vector_to_marker << std::setprecision(4)
                          << "z: " << std::setw(8) << tvecs[0](2);
          cv::putText(imageCopy, vector_to_marker.str(),
                      cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                      cvScalar(0, 0, 255), 1, cv::LINE_AA);

          /* From the orientation vector, compute the rpy vecto to be used in ROS for camera positioning */
          cv::Matx33d ori, oriR, oriQ;
          cv::Rodrigues(rvecs[0], ori);
          cv::Vec3d ypr = cv::RQDecomp3x3(ori,oriR, oriQ)/180.0; 
          vector_to_marker = std::ostringstream();
          vector_to_marker << std::setprecision(4) << "Yaw (X): " << std::setw(8) << ypr(0);
          cv::putText(imageCopy, vector_to_marker.str(), cvPoint(10,90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar(0, 0, 255), 1, cv::LINE_AA);
          vector_to_marker = std::ostringstream();
          vector_to_marker << std::setprecision(4) << "Pitch (Y): " << std::setw(8) << ypr(1);
          cv::putText(imageCopy, vector_to_marker.str(), cvPoint(10,110), cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar(0, 0, 255), 1, cv::LINE_AA);
          vector_to_marker = std::ostringstream();
          vector_to_marker << std::setprecision(4) << "Roll (Z): " << std::setw(8) << ypr(2);
          cv::putText(imageCopy, vector_to_marker.str(), cvPoint(10,130), cv::FONT_HERSHEY_SIMPLEX, 0.6, cvScalar(0, 0, 255), 1, cv::LINE_AA);

          /* Once printed the position and orientation of the center of the diamond marker. The H^-{1} should be computed to position the camera */
          tf2::Vector3 temp_pos = tf2::Vector3(tvecs[0](0), tvecs[0](1), tvecs[0](2));
          tf2::Matrix3x3 temp_rot = tf2::Matrix3x3(ori(0,0), ori(0,1), ori(0,2), ori(1,0), ori(1,1), ori(1,2), ori(2,0), ori(2,1), ori(2,2));

          tf2::Stamped<tf2::Transform> transform;
          transform.setOrigin(temp_pos);
          transform.setBasis(temp_rot);

          double roll = 0.0;
          double pitch = 0.0;
          double yaw = 0.0;
          transform.getBasis().getRPY(roll, pitch, yaw);
          tf2::Vector3 temp_rpy = tf2::Vector3(roll, pitch, yaw);

          if (!camera_base_frame.empty() || !base_frame.empty())
          {
            tf2::Matrix3x3 temp_rot_inv = temp_rot.transpose();
            tf2::Vector3 temp_pos_inv = temp_rot_inv * (-temp_pos);
            cv::Matx44d temp_H_colorFrame_marker = cv::Matx44d(temp_rot_inv[0][0], temp_rot_inv[0][1], temp_rot_inv[0][2], temp_pos_inv[0],
                                                                temp_rot_inv[1][0], temp_rot_inv[1][1], temp_rot_inv[1][2], temp_pos_inv[1],
                                                                temp_rot_inv[2][0], temp_rot_inv[2][1], temp_rot_inv[2][2], temp_pos_inv[2], 0, 0, 0, 1);
            
            cv::Matx44d temp_H_camBase_marker = base_frame_to_board * temp_H_colorFrame_marker * opt_center_to_cam_base;
            tf2::Matrix3x3 temp_H_rot = tf2::Matrix3x3( temp_H_camBase_marker(0,0), temp_H_camBase_marker(0,1), temp_H_camBase_marker(0,2), 
                                                        temp_H_camBase_marker(1,0), temp_H_camBase_marker(1,1), temp_H_camBase_marker(1,2), 
                                                        temp_H_camBase_marker(2,0), temp_H_camBase_marker(2,1), temp_H_camBase_marker(2,2));

            temp_pos[0] = temp_H_camBase_marker(0,3);
            temp_pos[1] = temp_H_camBase_marker(1,3);
            temp_pos[2] = temp_H_camBase_marker(2,3);
            transform.setOrigin(temp_pos);
            transform.setBasis(temp_H_rot);
            
            transform.getBasis().getRPY(roll, pitch, yaw);
            temp_rpy = tf2::Vector3(roll, pitch, yaw);
          }
          else {
            auto inv_transform = transform.inverse();

            inv_transform.getBasis().getRPY(roll, pitch, yaw);
            temp_rpy = tf2::Vector3(roll, pitch, yaw);
            temp_pos = tf2::Vector3(
              inv_transform.getOrigin().getX(),
              inv_transform.getOrigin().getY(),
              inv_transform.getOrigin().getZ());
          }

          if (current_sampling_number < sample_number)
          {
              pos_vect.push_back(temp_pos);
              rpy_vect.push_back(temp_rpy);
              current_sampling_number++;
              if(current_sampling_number % (int) 10 == 0)
                  RCLCPP_INFO_STREAM(node->get_logger(), "Samples " << current_sampling_number << "/" << sample_number);
          }
          else if (current_sampling_number >= sample_number)
          {
              RCLCPP_INFO(node->get_logger(), "SAMPLING DONE");

              std::vector<double> pos_vect_add;
              std::vector<double> rpy_vect_add;
              pos_vect_add.resize(3);
              rpy_vect_add.resize(3);
              int stored_size = pos_vect.size();

              for (size_t i = 0; i < pos_vect.size(); i++)
              {
                  pos_vect_add[0] = pos_vect[i][0] + pos_vect_add[0];
                  pos_vect_add[1] = pos_vect[i][1] + pos_vect_add[1];
                  pos_vect_add[2] = pos_vect[i][2] + pos_vect_add[2];
                  rpy_vect_add[0] = rpy_vect[i][0] + rpy_vect_add[0];
                  rpy_vect_add[1] = rpy_vect[i][1] + rpy_vect_add[1];
                  rpy_vect_add[2] = rpy_vect[i][2] + rpy_vect_add[2];
              }

              if (camera_base_frame.empty())
              {
                auto origin = base_frame.empty() ? std::string("Calibration Board") : base_frame;
                RCLCPP_INFO_STREAM(node->get_logger(), origin << " -> " << cam_info->header.frame_id);
              }
              else
              {
                auto origin = base_frame.empty() ? std::string("Calibration Board") : base_frame;
                RCLCPP_INFO_STREAM(node->get_logger(), origin << " -> " << camera_base_frame);
              }
              
              RCLCPP_INFO(node->get_logger(), "Position mean values: (x,y,z) = %lf %lf %lf", pos_vect_add[0]/stored_size, pos_vect_add[1]/stored_size, pos_vect_add[2]/stored_size);
              RCLCPP_INFO(node->get_logger(), "Orientation mean values: (R(x),R(y),R(z)) = %lf %lf %lf", rpy_vect_add[0]/stored_size, rpy_vect_add[1]/stored_size, rpy_vect_add[2]/stored_size);

              current_sampling_number++;
          }
      }
  }

  cv::imshow(camera_topic, imageCopy);
  if (current_sampling_number > sample_number) {
    RCLCPP_INFO(node->get_logger(), "Exit");
    cv::destroyWindow(camera_topic);
    done = true;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

  node = rclcpp::Node::make_shared("diamond_detector_node", options);
  RCLCPP_INFO(node->get_logger(), "Camera Positioning Node");
  
  // get parameters
  
  node->declare_parameter("axis_length", rclcpp::ParameterType::PARAMETER_DOUBLE);
  axisLength = node->get_parameter_or("axis_length", 0.1);

  node->declare_parameter("board_length", rclcpp::ParameterType::PARAMETER_DOUBLE);
  double boardLength = node->get_parameter_or("board_length", 0.2);

  squareLength = boardLength / 3.0; // 3x3 matrix
  markerLength = squareLength * (296.0/360.0); // Marker generator proportions

  node->declare_parameter("camera_base_frame", rclcpp::ParameterType::PARAMETER_STRING);
  camera_base_frame = node->get_parameter_or("camera_base_frame", std::string(""));

  node->declare_parameter("base_frame", rclcpp::ParameterType::PARAMETER_STRING);
  base_frame = node->get_parameter_or("base_frame", std::string(""));

  node->declare_parameter("board_frame", rclcpp::ParameterType::PARAMETER_STRING);
  board_frame = node->get_parameter_or("board_frame", std::string(""));

  if (!base_frame.empty() && board_frame.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "board_frame must be specified if base_frame is given");
    return 1;
  }

  if (!camera_base_frame.empty() || !base_frame.empty()) {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

  node->declare_parameter("camera_topic", rclcpp::ParameterType::PARAMETER_STRING);
  camera_topic = node->get_parameter_or("camera_topic", std::string("/camera/image"));
  node->declare_parameter("sample_number", rclcpp::ParameterType::PARAMETER_INTEGER);
  sample_number = node->get_parameter_or("sample_number", 120);

  cv::startWindowThread();

  image_transport::ImageTransport it(node);
  image_transport::CameraSubscriber sub = it.subscribeCamera(camera_topic, 1, imageCallback);

  while(!done) rclcpp::spin_some(node);

  node.reset();
  rclcpp::shutdown();

  return 0;
}
