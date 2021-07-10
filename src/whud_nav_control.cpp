#include <geometry_msgs/Twist.h>
#include <mavros/mavros_plugin.h>
#include <std_msgs/Bool.h>

namespace mavros {
namespace extra_plugins {
class WhudNavPlugin : public plugin::PluginBase {
public:
  WhudNavPlugin() : PluginBase(), whud_nh_("~whud_nav") {}

  void initialize(UAS &uas_) override {
    PluginBase::initialize(uas_);

    whud_nh_.param<float>("max_roll", max_roll_, 0.5);
    whud_nh_.param<float>("max_pitch", max_pitch_, 0.5);

    cmd_vel_sub_ =
        whud_nh_.subscribe("cmd_vel", 1, &WhudNavPlugin::cmd_vel_cb, this);
    conversion_sub_ = whud_nh_.subscribe("conversion", 1,
                                         &WhudNavPlugin::conversion_cb, this);
    conversion_timer_ = whud_nh_.createTimer(ros::Duration(0.1),
                                             &WhudNavPlugin::timer_cb, this);
  }

  Subscriptions get_subscriptions() override {
    return {
        /* Rx disabled */
    };
  }

private:
  ros::NodeHandle whud_nh_;

  bool conversion_ = false;
  int conversion_delay_counter_ = 0;
  int max_delay_counter_ = 10;
  float max_roll_, max_pitch_;

  ros::Timer conversion_timer_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber conversion_sub_;

  void timer_cb(const ros::TimerEvent &event) {
    if (conversion_) {
      conversion_delay_counter_++;

      if (conversion_delay_counter_ >= max_delay_counter_) {
        conversion_delay_counter_ = 0;
        conversion_ = false;
      }
    } else {
      conversion_delay_counter_ = 0;
    }
  }

  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr &req) {
    if (!conversion_) {
      mavlink::common::msg::COMMAND_LONG msg;
      // MAV_CMD_USER_1
      msg.command = 31010;
      msg.param1 = req->linear.x;
      msg.param2 = req->linear.y;
      msg.param3 = req->linear.z;
      msg.param4 = req->angular.z;
      // maxRoll
      msg.param5 = max_roll_;
      // maxPitch
      msg.param6 = max_pitch_;
      msg.param7 = 0;

      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    }
  }

  void conversion_cb(const std_msgs::Bool::ConstPtr &conversion) {
    if (conversion->data) {
      conversion_ = conversion->data;
      conversion_delay_counter_ = 0;

      mavlink::common::msg::COMMAND_LONG msg;
      // MAV_CMD_USER_1
      msg.command = 31010;
      msg.param1 = 0;
      msg.param2 = 0;
      msg.param3 = 0;
      msg.param4 = 0;
      // maxRoll
      msg.param5 = 0;
      // maxPitch
      msg.param6 = 0;
      msg.param7 = 1;

      UAS_FCU(m_uas)->send_message_ignore_drop(msg);
    }
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::WhudNavPlugin,
                       mavros::plugin::PluginBase)