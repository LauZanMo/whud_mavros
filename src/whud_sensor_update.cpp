#include <mavros/mavros_plugin.h>
#include <tf/transform_listener.h>

namespace mavros {
namespace extra_plugins {
class WhudSensorPlugin : public plugin::PluginBase {
 public:
  WhudSensorPlugin() : PluginBase(), whud_nh_("~whud_sensor") {}

  void initialize(UAS &uas_) override {
    PluginBase::initialize(uas_);

    whud_nh_.param<bool>("enable/lidar", lidar_enable_, true);
    whud_nh_.param<bool>("enable/visual_pos", visual_pos_enable_, false);
    whud_nh_.param<bool>("enable/visual_vel", visual_vel_enable_, false);

    whud_nh_.param<float>("period/lidar", lidar_period_, 0.1);
    whud_nh_.param<float>("period/visual_pos", visual_pos_period_, 0.1);
    whud_nh_.param<float>("period/visual_vel", visual_vel_period_, 0.02);

    whud_nh_.param<std::string>("id/lidar_target", lidar_target_id_, "map");
    whud_nh_.param<std::string>("id/lidar_source", lidar_source_id_,
                                "track_link");
    whud_nh_.param<std::string>("id/visual_pos_target", visual_pos_target_id_,
                                "camera_link");
    whud_nh_.param<std::string>("id/visual_pos_source", visual_pos_source_id_,
                                "camera_odom");

    if (lidar_enable_)
      lidar_timer_ = whud_nh_.createTimer(ros::Duration(lidar_period_),
                                          &WhudSensorPlugin::lidar_cb, this);

    if (visual_pos_enable_)
      visual_pos_timer_ =
          whud_nh_.createTimer(ros::Duration(visual_pos_period_),
                               &WhudSensorPlugin::visual_pos_cb, this);

    if (visual_vel_enable_)
      visual_vel_timer_ =
          whud_nh_.createTimer(ros::Duration(visual_vel_period_),
                               &WhudSensorPlugin::visual_vel_cb, this);
  }

  Subscriptions get_subscriptions() override {
    return {
        /* Rx disabled */
    };
  }

 private:
  ros::NodeHandle whud_nh_;

  bool lidar_enable_, visual_pos_enable_, visual_vel_enable_;
  float lidar_period_, visual_pos_period_, visual_vel_period_;
  std::string lidar_target_id_, lidar_source_id_;
  std::string visual_pos_target_id_, visual_pos_source_id_;

  bool lidar_frame_exist_ = false, visual_pose_frame_exist_ = false;

  ros::Timer lidar_timer_, visual_pos_timer_, visual_vel_timer_;
  tf::TransformListener tf_listener_;

  void lidar_cb(const ros::TimerEvent &event) {
    if (!lidar_frame_exist_) {
      bool res1 = tf_listener_.frameExists("/" + lidar_target_id_);
      bool res2 = tf_listener_.frameExists("/" + lidar_source_id_);

      if (res1 && res2) {
        lidar_frame_exist_ = true;
        ROS_INFO("Lidar SLAM transform detected.");
      }

    } else {
      tf::StampedTransform tf_transform;

      try {
        tf_listener_.lookupTransform("/" + lidar_target_id_,
                                     "/" + lidar_source_id_, ros::Time(0),
                                     tf_transform);
      } catch (tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        lidar_frame_exist_ = false;
        return;
      }

      mavlink::common::msg::ATT_POS_MOCAP msg;
      msg.x = tf_transform.getOrigin().getX();
      msg.y = tf_transform.getOrigin().getY();
      msg.z = tf_transform.getOrigin().getZ();

      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    }
  }

  void visual_pos_cb(const ros::TimerEvent &event) {
    if (!visual_pose_frame_exist_) {
      bool res1 = tf_listener_.frameExists("/" + visual_pos_target_id_);
      bool res2 = tf_listener_.frameExists("/" + visual_pos_source_id_);

      if (res1 && res2) {
        visual_pose_frame_exist_ = true;
        ROS_INFO("Visual SLAM transform detected.");
      }

    } else {
      tf::StampedTransform tf_transform;

      try {
        tf_listener_.lookupTransform("/" + visual_pos_target_id_,
                                     "/" + visual_pos_source_id_, ros::Time(0),
                                     tf_transform);
      } catch (tf::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        visual_pose_frame_exist_ = false;
        return;
      }

      mavlink::common::msg::VISION_POSITION_ESTIMATE msg;
      msg.x = tf_transform.getOrigin().getX();
      msg.y = tf_transform.getOrigin().getY();
      msg.z = tf_transform.getOrigin().getZ();

      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    }
  }
  void visual_vel_cb(const ros::TimerEvent &event) {}
};
}  // namespace extra_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::WhudSensorPlugin,
                       mavros::plugin::PluginBase)