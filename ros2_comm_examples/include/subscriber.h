#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sharedmem_msgs/msg/sample_message.hpp"

namespace ros2_demos {
  
class SubLatency {
 public:
  void Input(int latency_us, std::uintptr_t ptr) {
    if (min_us == 0 || latency_us < min_us) {
      min_us = latency_us;
    }
    if (latency_us > max_us) {
      max_us = latency_us;
    }
    sum_us += latency_us;
    count++;

    if (std::chrono::steady_clock::now() - time_last_tick_ > std::chrono::seconds(5)) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("minimal_subscriber"),
        "sub[" << ptr << "] latency(us): "
        << "avg[" << std::setprecision(6) << sum_us / static_cast<int>(count) << "] min[" << min_us << "] max[" << max_us
        << "] count[" << count << "]");
      min_us = 0;
      max_us = 0;
      sum_us = 0;
      count = 0;
      time_last_tick_ = std::chrono::steady_clock::now();
    }
  }
  
 private:
  int min_us = 0;
  int max_us = 0;
  uint64_t sum_us = 0;
  size_t count = 0;
  std::chrono::steady_clock::time_point time_last_tick_ = std::chrono::steady_clock::now();
};

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber(const rclcpp::NodeOptions& opts) :
    Node("minimal_subscriber", opts) {
    this->declare_parameter("buffer_type", cb_buffer_type_);
    this->get_parameter("buffer_type", cb_buffer_type_);
    RCLCPP_WARN_STREAM(this->get_logger(), "Input buffer_type[take_shared/take_ownership]: " << cb_buffer_type_);

    // 创建subscription，topic为"sample"，QoS使用rclcpp::SensorDataQoS()，对应的reliability为best effort
    if ("take_shared" == cb_buffer_type_) {
      subscription_ =
        this->create_subscription<sharedmem_msgs::msg::SampleMessage>(
            "topic", rclcpp::SensorDataQoS(),
              std::bind(&MinimalSubscriber::topic_cb_take_shared, this, std::placeholders::_1)
            );
    } else if ("take_ownership" == cb_buffer_type_) {
      subscription_ =
        this->create_subscription<sharedmem_msgs::msg::SampleMessage>(
            "topic", rclcpp::SensorDataQoS(),
              std::bind(&MinimalSubscriber::topic_cb_take_ownership, this, std::placeholders::_1)
            );
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid buffer_type: "
        << cb_buffer_type_ << ", which should be [take_shared/take_ownership]");
      rclcpp::shutdown();
      return;
    }
  }

 private:
  //  消息回调函数
  /**
   * 建议订阅ConstSharedPtr类型的msg，当有多个sub时也能实现zero copy。
   * 
   * 对于intra process的zero copy，sub端可以订阅RawPtr/ConstRawPtr/SharedPtr/ConstSharedPtr/UniquePtr等类型msg。
   * 不同类型msg对应的内存管理方式不同，决定着是否存在数据拷贝，进而影响效率。
   * 
   * 只有一个sub时，订阅两种类型msg没有区别，
   * 具体为：
   *    订阅UniquePtr类型msg时(take onwership)，则直接将pub端的unique_ptr msg发送给sub。
   *    订阅ConstSharedPtr类型msg时(take shared)，rclcpp中将UniquePtr转成ConstSharedPtr，不存在数据拷贝。
   * 
   * 当有超过1个sub时，例如N（N>1），订阅UniquePtr类型msg存在数据拷贝，而订阅ConstSharedPtr类型msg不存在数据拷贝。
   * 具体为：
   *    订阅UniquePtr类型msg时，
   *      对于前N-1个sub，使用传入的unique_ptr msg分别构造一个新的unique_ptr new_msg，用于消息的发布：
   *        https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/experimental/intra_process_manager.hpp#L462
   *      对于第N个sub，则直接将pub端的unique_ptr msg发送给sub：
   *        https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/experimental/intra_process_manager.hpp#L456
   *      因此存在N-1次数据拷贝。
   *    订阅ConstSharedPtr类型msg时，
   *      rclcpp中将UniquePtr转成ConstSharedPtr，分别发送给每个sub，
   *      因此不存在数据拷贝。
   *      https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/experimental/intra_process_manager.hpp#L206
   * 
   * 注意，SharedConstPtr时ROS2定义的msg类型。
   * 例如对于图像消息，cb的参数类型必须是sensor_msgs::msg::Image::ConstSharedPtr。
   * const sensor_msgs::msg::Image::SharedPtr、const std::shared_ptr<sensor_msgs::msg::Image>和sensor_msgs::msg::Image::ConstSharedPtr不是同一类型。
   */
  void topic_cb_take_shared(
      sharedmem_msgs::msg::SampleMessage::ConstSharedPtr msg
      ) {
    // 注意，如果发布的是loan msg只能在回调函数中使用，回调函数返回后，该消息就会被释放
    // 计算延时并打印出来
    auto time_diff_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count() - msg->time_stamp;
    RCLCPP_INFO_STREAM(this->get_logger(),
      "recved msg [" << msg->index
      << "], latency ["
      << std::setprecision(6)
      << time_diff_us << "]us"
      << std::hex
      << "\n\t msg address [0x" << reinterpret_cast<std::uintptr_t>(msg.get())
      << "], data address [0x" << reinterpret_cast<std::uintptr_t>(&msg->data[0]) << "]"
    );

    this->sub_latency_.Input(time_diff_us, reinterpret_cast<std::uintptr_t>(this));
  }
  
  void topic_cb_take_ownership(
      sharedmem_msgs::msg::SampleMessage::UniquePtr msg
      ) {
    // 注意，如果发布的是loan msg只能在回调函数中使用，回调函数返回后，该消息就会被释放
    // 计算延时并打印出来
    auto time_diff_us =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count() - msg->time_stamp;
    RCLCPP_INFO_STREAM(this->get_logger(),
      "recved msg [" << msg->index
      << "], latency ["
      << std::setprecision(6)
      << time_diff_us << "]us"
      << std::hex
      << "\n\t msg address [0x" << reinterpret_cast<std::uintptr_t>(msg.get())
      << "], data address [0x" << reinterpret_cast<std::uintptr_t>(&msg->data[0]) << "]"
    );

    this->sub_latency_.Input(time_diff_us, reinterpret_cast<std::uintptr_t>(this));
  }

  // subscription
  rclcpp::Subscription<sharedmem_msgs::msg::SampleMessage>::SharedPtr
      subscription_ = nullptr;

  // "take_shared", "take_ownership"
  std::string cb_buffer_type_{"take_shared"};

  SubLatency sub_latency_;
};

}
