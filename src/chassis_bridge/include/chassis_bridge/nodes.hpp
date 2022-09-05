#pragma once

#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <utility>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "chassis_bridge/protocol.hpp"
#include "chassis_bridge/types.hpp"
#include "chassis_bridge/container.hpp"

namespace cb::nodes {
    class publisher : public rclcpp::Node {
        using rx_deque_item = std::unique_ptr<cb::types::underlying::rx::frame>;
    public:
        publisher(
            std::string&                             node_name,
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr
        ) : Node(node_name),
            receive_deque_ptr_(receive_deque_ptr) {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[publishers node] initializing publishers node: " << node_name << std::endl;
            on_publisher_initialize();
        }
        ~publisher() = default;

    protected:
        virtual void on_publisher_initialize() {}
        virtual void publisher_callback() {}

        cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
    };

};
