# 说明

ROS2不同通信方式的使用示例，性能测试方法，测试报告，结合数据的通信机制分析。

包含的ROS2通信类型如下：

1. 进程间不使用borrow_loaned_message的非零拷贝通信
2. 进程间不使用borrow_loaned_message的零拷贝通信
3. 进程间使用borrow_loaned_message的非零拷贝通信
4. 进程间使用borrow_loaned_message的零拷贝通信
5. 单进程开启进程内通信
6. 单进程关闭进程内通信

其他说明：

1. 使用borrow_loaned_message的意思是发布端publisher使用borrow_loaned_message接口申请消息内存，填充消息内容后使用publish接口发布申请到的消息。
2. 通过设置环境变量`export ROS_DISABLE_LOANED_MESSAGES=0`禁用零拷贝通信。


# 测试方法

- 消息大小和频率
   [消息](../sharedmem_msgs/msg/SampleMessage.msg)大小为4.19MB，其中payload大小为4MB，发布频率为25Hz。

- 调度器
  使用效率最优的调度器：`rclcpp::executors::StaticSingleThreadedExecutor`。

- latency计算
  发布端发布的消息中填充时间戳，订阅端计算时间戳差值，计算出通信latency。

- CPU占用
  使用`top`命令记录CPU占用率。

- 其他
  不测试不同通信方式的内存和带宽占用差异。

# 测试环境

1. 硬件
  - [RDK X3](https://developer.horizon.ai/rdkx3)（4核ARM A53@1.5G）, 4G内存。
  - 锁定CPU频率：

  ```bash
  sudo bash -c 'echo 1 > /sys/devices/system/cpu/cpufreq/boost'
  sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
  ```

2. 软件
  - Ubuntu 22.04。
  - ROS2 Humble版本。

# 测试

## 1. 进程间不使用borrow_loaned_message的非零拷贝通信

终端1，发布：
```bash
export ROS_DISABLE_LOANED_MESSAGES=1
ros2 run ros2_comm_examples demo_talker --ros-args --log-level warn
```

终端2，订阅：
```bash
export ROS_DISABLE_LOANED_MESSAGES=1
ros2 run ros2_comm_examples demo_listener --ros-args --log-level warn
```

latency: 7440 us

cpu: 21%(11.0+10)

## 2. 进程间不使用borrow_loaned_message的零拷贝通信

终端1，发布：
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`ros2 pkg prefix ros2_comm_examples`/lib/ros2_comm_examples/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0

ros2 run ros2_comm_examples demo_talker --ros-args --log-level warn
```

终端2，订阅：
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`ros2 pkg prefix ros2_comm_examples`/lib/ros2_comm_examples/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0

ros2 run ros2_comm_examples demo_listener --ros-args --log-level warn
```

latency: 830 us

cpu: 2.3%(2.0+0.3)

为什么不使用borrow_loaned_message也能实现零拷贝通信？

原因可能是，虽然dds的零拷贝本身并不限制消息类型，只要是POD(Plain Old Data)类型就可以，但是dds提供了专用于零拷贝发布的接口（ros2 rclcpp同样也提供了专用接口），如果使用这个专用接口发布消息，dds认为用户就将这个消息的生命期移交给了dds，因为本身这个msg就是使用dds的接口申请的，因此可以直接使用borrow_loaned_message到的内存发布消息，并且发布和订阅段都可以跳过序列化处理。如果不使用borrow_loaned_message，dds可能会拷贝消息，然后进行序列化，再发布。

所以通信lantency能够从3230us降到830us，但是仍然远远高于使用borrow_loaned_message零拷贝的85us（下文测试结果）。另外从收发消息的内存地址也能看出，启用零拷贝时，如果使用borrow_loaned_message，收到的消息和发送的消息内存地址相同，而不使用borrow_loaned_message时，收到消息和发送消息是的内存不同。

## 3. 进程间使用borrow_loaned_message非零拷贝通信

以零拷贝方式发布消息，但是不启用零拷贝。

```bash
export ROS_DISABLE_LOANED_MESSAGES=1
ros2 run ros2_comm_examples demo_talker_loan --ros-args --log-level warn
```

```bash
export ROS_DISABLE_LOANED_MESSAGES=1
ros2 run ros2_comm_examples demo_listener --ros-args --log-level warn
```

latency: 9030 us

cpu: 24.3%(14.3+10)

对于非零拷贝通信，不使用borrow_loaned_message通信的lantency为7440us，而使用borrow_loaned_message通信的lantency反而更高。

原因？TODO


## 4. 进程间使用borrow_loaned_message的零拷贝通信

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`ros2 pkg prefix ros2_comm_examples`/lib/ros2_comm_examples/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0

ros2 run ros2_comm_examples demo_talker_loan --ros-args --log-level warn
```

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=`ros2 pkg prefix ros2_comm_examples`/lib/ros2_comm_examples/config/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0

ros2 run ros2_comm_examples demo_listener --ros-args --log-level warn
```

| subscriber数量 | latency(us) | cpu(%) |
| --------------- | --------- | ------ |
| 1               | 85      | 0.6%(0.3+0.3)    |
| 2               | 112, 116  | 0.9%(0.3+0.3+0.3)   |

## 5. 单进程开启进程内通信

```bash
# 通过设置SUB_NUM启动多个订阅
export SUB_NUM=1
export EXE_TYPE=2
export USE_IPC=1
ros2 run ros2_comm_examples demo_manual_composition --ros-args -p buffer_type:=take_shared --log-level warn
```

| subscriber数量 | latency(us) | cpu(%) |
| --------------- | --------- | ------ |
| 1               | 44        | 0.3    |
| 2               | 57, 68   | 0.3    |


## 6. 单进程关闭进程内通信

```bash
# 通过设置SUB_NUM启动多个订阅
export SUB_NUM=1
export EXE_TYPE=2
export USE_IPC=0
ros2 run ros2_comm_examples demo_manual_composition --ros-args -p buffer_type:=take_shared --log-level warn
```

| subscriber数量 | latency(us) | cpu(%) |
| --------------- | --------- | ------ |
| 1               | 3100       | 8.3    |
| 2               | 3140, 5400   | 13.7    |

## 7. 影响node composition通信效率的因素

### (1) 调度器类型

对比了ROS2的3种调度器类型，运行时使用环境变量`EXE_TYPE`设置调度器类型，具体为：

```bash
# 0: single threaded executor
# 1: multi threaded executor
# 2: static single threaded executor
```

启动命令：

```bash
export SUB_NUM=1
# 指定使用static single threaded executor
export EXE_TYPE=0
export USE_IPC=1
ros2 run ros2_comm_examples demo_manual_composition --ros-args -p buffer_type:=take_shared --log-level warn
```

测试结果：

| 调度器类型 | latency (us) | CPU占用(%) |
| ------------ | ------ | ---------- |
| single threaded executor | 51<br />64, 80 | 0.5<br />0.8 |
| multi threaded executor | 58<br />69, 82 | 0.5<br />0.8 |
| static single threaded executor | 45<br />59, 68 | 0.3<br />0.3 |

每个类别的调度器分别测试了一个和两个订阅时的数据，数据反映：

1. `static single threaded executor`调度器的性能最优。
2. 当有多个订阅时，后加入的订阅node的latency会依次增加。原因为intra_process_manager中按照node加入顺序，[依次给node中的sub提供数据](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/experimental/intra_process_manager.hpp#L359)，因此最后add的node的latency最大。

### (2) 订阅的消息类型

sub端可以订阅两种类型msg（订阅回调的入参）：UniquePtr类型(take onwership)和ConstSharedPtr类型(take shared)。

不同类型msg对应的内存管理方式不同，决定着是否存在数据拷贝，进而影响效率。

下面对比使用进程内通信，订阅两种类型msg的效率表现。运行时使用`buffer_type`参数指定消息类型，具体为：

```bash
# take_shared: 订阅ConstSharedPtr类型消息
# take_ownership: 订阅UniquePtr类型消息
```

启动命令：

```bash
export SUB_NUM=1
export EXE_TYPE=2
export USE_IPC=1
# 使用buffer_type参数指定消息类型
ros2 run ros2_comm_examples demo_manual_composition --ros-args -p buffer_type:=take_shared --log-level warn
```

测试结果：

| 消息类型 | latency (us) | CPU占用(%) |
| ------------ | ------ | ---------- |
| UniquePtr | 43<br />614, 625 | 0.3<br />1.8 |
| ConstSharedPtr | 45<br />58, 69 | 0.3<br />0.5 |

每个类别的调度器分别测试了一个和两个订阅时的数据，数据反映：

1. 只有一个sub时，订阅两种类型msg没有区别。
2. 当有超过1个sub时，从latency和CPU占用看，订阅UniquePtr类型msg存在数据拷贝，而订阅ConstSharedPtr类型msg不存在数据拷贝。代码实现层面的分析详见[示例代码说明](./include/subscriber.h)。
3. 当有多个sub时，订阅ConstSharedPtr类型的msg才能实现zero copy。

### (3) 发布的消息类型

发布`unique_ptr`类型消息的性能显著优于`const &`类型消息。原因为如果pub端发布`const &`类型消息，rclcpp层会使用`const & msg`构造出`unique_ptr msg`，再使用`unique_ptr`类型的publish接口发布数据，因此存在[数据拷贝](https://github.com/ros2/rclcpp/blob/humble/rclcpp/include/rclcpp/publisher.hpp#L302)。直接使用`unique_ptr`类型消息进行发布，不存在数据拷贝。

### (4) 发布端消息初始化方式

发布端创建消息时，可以通过给构造函数传入参数指定消息的初始化方式，支持的初始化方式[rosidl_runtime_cpp::MessageInitialization](https://github.com/ros2/rosidl/blob/humble/rosidl_runtime_cpp/include/rosidl_runtime_cpp/message_initialization.hpp#L27)如下：

```c++
enum class MessageInitialization
{
  ALL = ROSIDL_RUNTIME_C_MSG_INIT_ALL,
  SKIP = ROSIDL_RUNTIME_C_MSG_INIT_SKIP,
  ZERO = ROSIDL_RUNTIME_C_MSG_INIT_ZERO,
  DEFAULTS_ONLY = ROSIDL_RUNTIME_C_MSG_INIT_DEFAULTS_ONLY,
};
```

默认的初始化方式是`ALL`，即初始化所有字段。

创建消息时不进行初始化（`SKIP`），能够避免初始化大块内存造成的性能损失。

### (5) node composition方式

ROS2支持[多种node composition方式](https://github.com/ros2/demos/tree/humble/composition)，具体为：

| 编号 | 方式 | load阶段 | 优缺点 |
| ---------------------------- | ---------------------------- | ----------------------------------------------- | -------------------------------------------------- |
| 1 | rclcpp_components | 运行期 | 1. 灵活性最高，支持跨设备、运行期动态load/unload。<br />2. 不需要include和link加载的node。<br />3. 运行时效率低，ComponentManager中创建的用于动态load/unload的service，使调度器 scheduler overhead 增加。 |
| 2 | 基于ClassLoader的dlopen | 启动期 | 1. 不需要include和link加载的node。<br />2. 启动时可以批量load node。<br />3. 需要知道加载node的so文件名。<br />4. 只支持load，不支持unload。 |
| 3 | launch文件 & rclcpp_components | 运行期 | 1. 灵活性最高，支持跨设备、运行期动态load/unload。<br />2. 不需要include和link加载的node。<br />3. 可以批量load node。|
| 4 | 编写代码，include和link node | 编译期 | 1. 易于理解。<br />2. 加载速度快，不依赖class loader加载so。<br />3. 不依赖任何功能包，可用于最小化裁剪的ROS2。<br />4. 需要include和link所有load的node。|


本章节对比不同加载方式的性能差异，1和3本质上没有区别，因此只比较1、2、3方式的差异。

测试条件：启动一个订阅，使用single thread调度器，开启进程内通信，订阅ConstSharedPtr类型消息（take_shared）。

**运行期加载**

使用ROS2的rclcpp_components package进行运行期动态load。

终端1，启动component_container：

```bash
ros2 run rclcpp_components component_container --ros-args --log-level warn
```

终端2，load component：

```bash
ros2 component load /ComponentManager ros2_comm_examples ros2_demos::MinimalPublisher -e use_intra_process_comms:=True

# 使用SharedPtr类型消息订阅
ros2 component load /ComponentManager ros2_comm_examples ros2_demos::MinimalSubscriber -p buffer_type:=take_shared -e use_intra_process_comms:=True

# 通过多次load subscriber，可以启动多个订阅
```

latency(us): 59
cpu(%): 0.5




**启动期加载**

使用的测试程序[dlopen_composition](./src/dlopen_composition.cpp)参考 ROS2 demo中 `composition package` 的[dlopen_composition](https://github.com/ros2/demos/blob/humble/composition/src/dlopen_composition.cpp)。

```bash
ros2 run ros2_comm_examples demo_dlopen_composition `ros2 pkg prefix ros2_comm_examples`/lib/liblistener.so `ros2 pkg prefix ros2_comm_examples`/lib/libtalker.so --ros-args --log-level warn
```

latency(us): 52
cpu(%): 0.5

**编译期加载**

编写代码，include和link node。

```bash
# 通过设置SUB_NUM启动多个订阅
export SUB_NUM=1
export EXE_TYPE=0
export USE_IPC=1
ros2 run ros2_comm_examples demo_manual_composition --ros-args -p buffer_type:=take_shared --log-level warn
```

latency(us): 52
cpu(%): 0.5


**不同composition方式对比**

| composition方式 | latency (us) | CPU占用(%) |
| ------------ | ------ | ---------- |
| 运行期 | 59 | 0.3 |
| 启动期 | 52 | 0.3 |
| 编译期 | 52 | 0.3 |

启动期和编译期composition方式性能差异不大，而运行期composition方式的lantency略高，原因为这种load方式需要ComponentManager中创建用于动态load/unload的service，使得调度器 scheduler overhead 增加。

# 性能对比

| 通信类型 | latency (us) | CPU占用(%) |
| ------------ | ------ | ---------- |
| 进程间不使用borrow_loaned_message的非零拷贝通信 | 7440 | 21.0 |
| 进程间不使用borrow_loaned_message的零拷贝通信 | 830 | 2.3 |
| 进程间使用borrow_loaned_message的非零拷贝通信 | 9030 | 24.3 |
| 进程间使用borrow_loaned_message的零拷贝通信 | 85<br />112, 116 | 0.6<br />0.9 |
| **单进程开启进程内通信** | 44<br />57, 68 | 0.3<br />0.3 |
| 单进程关闭进程内通信 | 3100<br />3140, 5400 | 8.3<br />13.7 |

其中后三组测试结果包含一个和两个订阅的情况。

通过对比看到，**进程内通信方式**表现出最优性能，其次是**使用borrow_loaned_message的零拷贝通信方式**，**进程间使用borrow_loaned_message的非零拷贝通信方式**的性能最差。

# 总结

设备内大数据通信的最优方案是使用**进程内通信**。

进程内通信的best practice：

1. 当有多个订阅时，对时延敏感的node优先composition，达到最小latency。
2. 订阅端订阅ConstSharedPtr类型的msg，避免多个sub时产生数据拷贝。
3. 发布端发布`unique_ptr`类型消息，避免在rclcpp层的publish接口产生数据拷贝。
4. 发布端创建消息时，指定消息的初始化方式为`MessageInitialization::SKIP`，即不初始化，避免初始化大块内存造成的性能损失。
5. 使用ros2提供的rclcpp_components组件运行期动态load/unload node，组件的overhead可以忽略，但是具有非常好的灵活性。

# Future Work

测试在高频消息通信下，几种通信方式性能差异。
