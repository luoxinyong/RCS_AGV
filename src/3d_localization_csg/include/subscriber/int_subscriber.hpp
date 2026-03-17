#ifndef SLAM_AD_SUBSCRIBER_INT_SUBSCRIBER_HPP_
#define SLAM_AD_SUBSCRIBER_INT_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

namespace h_x
{
class IntSubscriber
{
   public:
    IntSubscriber(ros::NodeHandle& nh, const std::string& topic_name, const size_t& buff_size);
    IntSubscriber() = default;
    void ParseData(int32_t& deque_int_data);

   private:
    void msg_callback(const std_msgs::Int32::ConstPtr& int_msg_ptr);

   private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    int32_t floor_data_ = 2; // 默认加载序号为1的submap
    std::mutex buff_mutex_;
};
}  // namespace h_x
#endif