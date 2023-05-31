#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <map>
#include <float.h>
#include <memory>
#include <shared_mutex>
#include <mutex>
#include <algorithm>
#include <ctime>
#include <chrono>
#include <stack>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "opencv2/opencv.hpp"
#include "../../include/Json/json.h"

#define OBJHP_NUM (int)8
#define CARPOS_NUM (int)6