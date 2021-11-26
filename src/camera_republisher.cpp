/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>

#include <mrs_lib/param_loader.h>

#include <image_transport/image_transport.h>

//}

namespace camera_republisher
{

/* class CameraRepublisher //{ */

class CameraRepublisher : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  bool is_initialized_ = false;

  ros::NodeHandle nh_;

  std::string _frame_;

  double _calibration_width_;
  double _calibration_height_;

  std::vector<double> _calibration_D_;
  std::vector<double> _calibration_K_;
  std::vector<double> _calibration_R_;
  std::vector<double> _calibration_P_;

  ros::Publisher publisher_camera_info_;

  image_transport::Publisher publisher_image_;

  image_transport::Subscriber subscriber_image_;
  void                        callbackImage(const sensor_msgs::ImageConstPtr& msg);
};

//}

/* onInit() //{ */

void CameraRepublisher::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonConeGenerator]: initializing");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh_, "CameraRepublisher");

  param_loader.loadParam("frame", _frame_);

  param_loader.loadParam("calibration/D", _calibration_D_);
  param_loader.loadParam("calibration/K", _calibration_K_);
  param_loader.loadParam("calibration/R", _calibration_R_);
  param_loader.loadParam("calibration/P", _calibration_P_);
  param_loader.loadParam("calibration/width", _calibration_width_);
  param_loader.loadParam("calibration/height", _calibration_height_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[CameraRepublisher]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- image transport -------------------- |

  image_transport::ImageTransport it(nh_);

  // | ----------------------- subscribers ---------------------- |

  subscriber_image_ = it.subscribe("image_in", 1, &CameraRepublisher::callbackImage, this);

  // | ----------------------- publishers ----------------------- |

  publisher_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info_out", 1);
  publisher_image_       = it.advertise("image_out", 1);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[CameraRepublisher]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackImage() //{ */

void CameraRepublisher::callbackImage(const sensor_msgs::ImageConstPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  {
    sensor_msgs::Image image = *msg;

    image.header.frame_id = _frame_;

    publisher_image_.publish(image);

    ROS_INFO_ONCE("[CameraRepublisher]: republishing images");
  }

  {
    sensor_msgs::CameraInfo camera_info;

    camera_info.header.stamp    = msg->header.stamp;
    camera_info.header.frame_id = _frame_;

    camera_info.distortion_model = "plumb_bob";

    camera_info.width  = _calibration_width_;
    camera_info.height = _calibration_height_;

    for (size_t i = 0; i < _calibration_K_.size(); i++) {
      camera_info.K[i] = _calibration_K_[i];
    }

    for (size_t i = 0; i < _calibration_P_.size(); i++) {
      camera_info.P[i] = _calibration_P_[i];
    }

    for (size_t i = 0; i < _calibration_R_.size(); i++) {
      camera_info.R[i] = _calibration_R_[i];
    }

    /* for (size_t i = 0; i < _calibration_P_.size(); i++) { */
    /*   camera_info.D.push_back(_calibration_D_[i]); */
    /* } */

    publisher_camera_info_.publish(camera_info);
  }
}

//}

}  // namespace camera_republisher

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_republisher::CameraRepublisher, nodelet::Nodelet);
