#include "subscriber/int_subscriber.hpp"

namespace h_x
{
IntSubscriber::IntSubscriber(ros::NodeHandle& nh, const std::string& topic_name, const size_t& buff_size) : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IntSubscriber::msg_callback, this);
}

void IntSubscriber::msg_callback(const std_msgs::Int32::ConstPtr& int_msg_ptr)
{
    buff_mutex_.lock();
    floor_data_ = int_msg_ptr->data;
    buff_mutex_.unlock();
}

void IntSubscriber::ParseData(int32_t& deque_int_data)
{
    buff_mutex_.lock();
    deque_int_data = floor_data_;
    buff_mutex_.unlock();
}
}  // namespace h_x