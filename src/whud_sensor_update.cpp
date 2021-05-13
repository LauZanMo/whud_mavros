#include <geometry_msgs/Twist.h>
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
    whud_nh_.param<std::string>("id/visual_vel_track", visual_vel_track_id_,
                                "camera_link");
    whud_nh_.param<std::string>("id/visual_vel_observation",
                                visual_vel_observation_id_, "camera_odom");

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
  std::string visual_vel_track_id_, visual_vel_observation_id_;

  bool lidar_transform_exist_ = false, visual_pose_transform_exist_ = false,
       visual_vel_transform_exist_ = false;

  ros::Timer lidar_timer_, visual_pos_timer_, visual_vel_timer_;
  tf::TransformListener tf_listener_;

  void lidar_cb(const ros::TimerEvent &event) {
    if (!lidar_transform_exist_) {
      if (tf_listener_.canTransform("/" + lidar_target_id_,
                                    "/" + lidar_source_id_, ros::Time(0))) {
        lidar_transform_exist_ = true;
        ROS_INFO("Lidar SLAM transform detected.");
      }

    } else {
      tf::StampedTransform tf_transform;

      try {
        tf_listener_.lookupTransform("/" + lidar_target_id_,
                                     "/" + lidar_source_id_, ros::Time(0),
                                     tf_transform);
      } catch (tf::LookupException &ex) {
        lidar_transform_exist_ = false;
        ROS_WARN("%s", ex.what());
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
    if (!visual_pose_transform_exist_) {
      if (tf_listener_.canTransform("/" + visual_pos_target_id_,
                                    "/" + visual_pos_source_id_,
                                    ros::Time(0))) {
        visual_pose_transform_exist_ = true;
        ROS_INFO("Visual SLAM position transform detected.");
      }

    } else {
      tf::StampedTransform tf_transform;

      try {
        tf_listener_.lookupTransform("/" + visual_pos_target_id_,
                                     "/" + visual_pos_source_id_, ros::Time(0),
                                     tf_transform);
      } catch (tf::LookupException &ex) {
        visual_pose_transform_exist_ = false;
        ROS_WARN("%s", ex.what());
        return;
      }

      mavlink::common::msg::VISION_POSITION_ESTIMATE msg;
      msg.x = tf_transform.getOrigin().getX();
      msg.y = tf_transform.getOrigin().getY();
      msg.z = tf_transform.getOrigin().getZ();

      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    }
  }
  void visual_vel_cb(const ros::TimerEvent &event) {
    if (!visual_vel_transform_exist_) {
      if (tf_listener_.canTransform("/" + visual_vel_track_id_,
                                    "/" + visual_vel_observation_id_,
                                    ros::Time(0))) {
        visual_vel_transform_exist_ = true;
        ROS_INFO("Visual SLAM velocity transform detected.");
      }

    } else {
      geometry_msgs::Twist tf_twist;

      try {
        tf_listener_.lookupTwist(
            "/" + visual_vel_track_id_, "/" + visual_vel_observation_id_,
            ros::Time(0), ros::Duration(0.5 * visual_vel_period_), tf_twist);
      } catch (tf::LookupException &ex) {
        visual_vel_transform_exist_ = false;
        ROS_WARN("%s", ex.what());
        return;
      }

      mavlink::common::msg::VISION_SPEED_ESTIMATE msg;
      msg.x = tf_twist.linear.x;
      msg.y = tf_twist.linear.y;
      msg.z = tf_twist.linear.z;

      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    }
  }
};
} // namespace extra_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::WhudSensorPlugin,
                       mavros::plugin::PluginBase)