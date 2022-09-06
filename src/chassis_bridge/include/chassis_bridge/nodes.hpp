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
            publisher_request_stop_(false),
            receive_deque_ptr_(receive_deque_ptr) {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[publishers node] initializing publishers node: " << node_name << std::endl;
            on_publisher_initialize();
        }

        ~publisher() {
            if (!publisher_thread_.joinable()) publisher_request_stop_ = true;
            publisher_thread_.join();
        };

    protected:
        virtual void on_publisher_initialize() { publisher_thread_ = std::thread([this] { publisher_callback(); }); }
        virtual void publisher_callback() {}

        std::thread                              publisher_thread_;
        bool                                     publisher_request_stop_;
        cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
    };
};
