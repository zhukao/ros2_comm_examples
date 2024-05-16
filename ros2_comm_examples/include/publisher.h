#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iomanip>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "sharedmem_msgs/msg/sample_message.hpp"

using namespace std::chrono_literals;

namespace ros2_demos {
class MinimalPublisher  : public rclcpp::Node {
 public:
  MinimalPublisher (const rclcpp::NodeOptions& opts) :
    Node("minimal_publisher", opts), count_(0) {
    // 创建publisher，topic为"topic"，QoS使用rclcpp::SensorDataQoS()，对应的reliability为best effort
    /**
     * 注意，当使用loan msg通信时，强烈建议QoS的reliability使用best effort。如果QoS的reliability为reliable，且订阅端未启用零拷贝，发布端存在稳定性问题。
     * 具体为：
     *    rmw层调用dds的申请缓存接口loan_sample可能会返回错误: 
     *      https://github.com/ros2/rmw_fastrtps/blob/humble/rmw_fastrtps_shared_cpp/src/rmw_publisher.cpp#L186
     *    返回错误导致rclcpp中由于申请loan msg失败而抛异常: 
     *      https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/loaned_message.hpp#L73
     */
    publisher_ = this->create_publisher<sharedmem_msgs::msg::SampleMessage>(
        "topic", rclcpp::SensorDataQoS());

    // 每隔40毫秒调用一次timer_callback进行消息发送
    /**
     * 定时器会占用调度器资源，导致订阅回调不能及时响应
     * 为了降低调度器负载，使用线程周期处理数据发布任务
     */
    sp_task_ = std::make_shared<std::thread>(
      [this]() {
        while (rclcpp::ok()) {
          timer_callback();
          std::this_thread::yield();
          rclcpp::sleep_for(std::chrono::milliseconds(40));
        }
      }
    );
  }

  ~MinimalPublisher() {
    if (sp_task_) {
      sp_task_->join();
    }
  }

 private:
  // 定时器回调函数
  void timer_callback() {
    if (!publisher_) {
      RCLCPP_ERROR(this->get_logger(), "publisher_ is nullptr");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "publish msg start");

    // 获取当前时间，单位为us
    auto time_now =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    int msg_index = 0;
    std::uintptr_t msg_address = 0;
    std::uintptr_t data_address = 0;

    std::string msg_type{""};
#ifdef USING_LOAN_MSG
    // 使用loan msg发布消息
      msg_type = "loan";
    // 获取要发送的消息
    auto loanedMsg = publisher_->borrow_loaned_message();
    // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
    if (loanedMsg.is_valid()) {
      // 引用方式获取实际的消息
      auto& msg = loanedMsg.get();
      // 对消息的index和time_stamp进行赋值
      msg.index = count_;
      msg.time_stamp = time_now;

      msg_index = msg.index;
      msg_address = reinterpret_cast<std::uintptr_t>(&msg);
      data_address = reinterpret_cast<std::uintptr_t>(&msg.data[0]);

      publisher_->publish(std::move(loanedMsg));
      
      // 计数器加一
      count_++;
    } else {
      // 获取消息失败，丢弃该消息
      RCLCPP_ERROR(this->get_logger(), "Failed to get LoanMessage!");
      return;
    }
#else
    // 不使用loan msg发布消息
    msg_type = "unique";
    // 创建要发送的消息
    /**
     * 对于intra process的zero copy，需要注意以下几点：
     * 
     * 1. 建议pub端发布unique ptr类型消息，避免在rclcpp层产生数据拷贝。
     * 
     * 如果pub端发布const &类型消息，rclcpp层会使用`const & msg`构造出`unique_ptr msg`，
     * 再使用`unique_ptr`类型的publish接口发布数据，因此存在数据拷贝。
     * https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/publisher.hpp#L302。
     * 这么做的目的是为了避免出现发布消息的过程中消息被改动？
     * 
     * 2. 创建消息时不进行初始化，避免初始化大块内存，造成性能损失。
     * https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/allocator/allocator_common.hpp#L38
     */
    auto msg = std::make_unique<sharedmem_msgs::msg::SampleMessage>(
        rosidl_runtime_cpp::MessageInitialization::SKIP
    );
    // 判断消息是否可用，可能出现获取消息失败导致消息不可用的情况
    if (msg)
    {
      RCLCPP_INFO(this->get_logger(), "set msg ts");
      // 对消息的index和time_stamp进行赋值
      msg->index = count_;
      msg->time_stamp = time_now;

      // 打印发送消息
      msg_index = msg->index;
      msg_address = reinterpret_cast<std::uintptr_t>(msg.get());
      data_address = reinterpret_cast<std::uintptr_t>(&msg->data[0]);

      publisher_->publish(std::move(msg));
      // 使用publish(const &)接口发布消息存在数据拷贝
      // publisher_->publish(std::move(*msg));

      // 计数器加一
      count_++;
    } else {
      // 获取消息失败，丢弃该消息
      RCLCPP_INFO(this->get_logger(), "Failed to get Message!");
      return;
    }
#endif

    int delay_us =
      std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count() - time_now;
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),
      *this->get_clock(),
      1000,
      "pub "
      << msg_type
      << " msg [" << msg_index
      << "], cost ["
      << std::setprecision(6)
      << delay_us << "]us"
      << std::hex
      << "\n\t msg address [0x" << msg_address
      << "], data address [0x" << data_address << "]"
      );
  }
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  // publisher
  rclcpp::Publisher<sharedmem_msgs::msg::SampleMessage>::SharedPtr publisher_ = nullptr;
  
  // 计数器
  size_t count_;

  std::shared_ptr<std::thread> sp_task_ = nullptr;
};
}
