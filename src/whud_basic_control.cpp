#include <mavros/mavros_plugin.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

namespace mavros {
namespace extra_plugins {
class WhudBasicPlugin : public plugin::PluginBase {
 public:
  WhudBasicPlugin() : PluginBase(), whud_nh_("~whud_basic") {}

  void initialize(UAS &uas_) override {
    PluginBase::initialize(uas_);

    whud_nh_.param("ack_cmd_index", ack_cmd_index_, -1);
    whud_nh_.param("ack_result", ack_result_, -1);
    whud_nh_.param("last_reject_index", last_reject_index_, -1);
    whud_nh_.param("last_reject_reason", last_reject_reason_, -1);

    takeoff_sub_ = whud_nh_.subscribe("takeoff_height", 1,
                                      &WhudBasicPlugin::takeoff_cb, this);
    land_sub_ = whud_nh_.subscribe("land", 1, &WhudBasicPlugin::land_cb, this);
    height_sub_ =
        whud_nh_.subscribe("height", 1, &WhudBasicPlugin::height_cb, this);
    yaw_sub_ = whud_nh_.subscribe("yaw", 1, &WhudBasicPlugin::yaw_cb, this);
    set_mode_sub_ = whud_nh_.subscribe("set_mode", 1, &WhudBasicPlugin::set_mode_cb, this);
    play_tune_sub_ = whud_nh_.subscribe("play_tune", 1, &WhudBasicPlugin::play_tune_cb, this);
  }

  Subscriptions get_subscriptions() override {
    return {
        make_handler(&WhudBasicPlugin::handle_progress),
    };
  }

 private:
  ros::NodeHandle whud_nh_;

  int ack_cmd_index_, ack_result_;
  int last_reject_index_, last_reject_reason_;

  ros::Subscriber takeoff_sub_;
  ros::Subscriber land_sub_;
  ros::Subscriber height_sub_;
  ros::Subscriber yaw_sub_;
  ros::Subscriber set_mode_sub_;
  ros::Subscriber play_tune_sub_;

  void handle_progress(const mavlink::mavlink_message_t *msg,
                       mavlink::common::msg::COMMAND_ACK &progress_msg) {
    if (progress_msg.result == 0 || progress_msg.result == 5) {
      whud_nh_.setParam("ack_cmd_index", progress_msg.command);
      whud_nh_.setParam("ack_result", progress_msg.result);
    } else {
      whud_nh_.setParam("last_reject_index", progress_msg.command);
      whud_nh_.setParam("last_reject_reason", progress_msg.result_param2);
    }
  }

  void takeoff_cb(const std_msgs::Float64MultiArray::ConstPtr &takeoff_msg) {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_NAV_TAKEOFF_LOCAL
    msg.command = 24;
    // set z axis speed
    msg.param3 = takeoff_msg->data[0];
    // set height
    msg.param7 = takeoff_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void land_cb(const std_msgs::Float64::ConstPtr &land_msg) {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_NAV_LAND_LOCAL
    msg.command = 23;
    // set z axis speed
    msg.param3 = land_msg->data > 0 ? (-land_msg->data) : (land_msg->data);

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void height_cb(const std_msgs::Float64MultiArray::ConstPtr &height_msg) {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_CONDITION_CHANGE_ALT
    msg.command = 113;
    // set descent/ascend rate
    msg.param1 = height_msg->data[0];
    // set target altitude
    msg.param7 = height_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void yaw_cb(const std_msgs::Float64MultiArray::ConstPtr &yaw_msg) {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_CONDITION_YAW
    msg.command = 115;
    // set target angle
    msg.param1 = yaw_msg->data[0];
    // 0: absolute angle, 1: relative offset
    msg.param4 = yaw_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void set_mode_cb(const std_msgs::Float64MultiArray::ConstPtr &set_mode_msg) {
    mavlink::common::msg::COMMAND_LONG msg;
    // MAV_CMD_DO_SET_MODE
    msg.command = 176;
    // set mode
    msg.param1 = set_mode_msg->data[0];
    // set custom mode
    msg.param2 = set_mode_msg->data[1];

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }

  void play_tune_cb(const std_msgs::Bool::ConstPtr &play_tune_msg) {
    mavlink::common::msg::PLAY_TUNE msg;
    // PLAY_TUNE
    msg.tune[0] = (char)play_tune_msg->data;

    UAS_FCU(m_uas)->send_message_ignore_drop(msg);
  }
};
}  // namespace extra_plugins
}  // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::WhudBasicPlugin,
                       mavros::plugin::PluginBase)
