#pragma once

#include <atomic>
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
    class bridge : public rclcpp::Node {
        using rx_deque_item = std::unique_ptr<cb::types::underlying::rx::frame>;
        using tx_deque_item = std::unique_ptr<cb::types::underlying::tx::frame>;
    public:
        bridge(
            std::string&                             node_name,
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr,
            cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr
        ) : Node(node_name),
            receive_deque_ptr_(receive_deque_ptr),
            transmit_deque_ptr_(transmit_deque_ptr),
            publisher_request_stop_(false) {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] initializing bridge node: " << node_name << std::endl;
            on_service_initialize(); 
            on_publisher_initialize();
        }

        ~bridge() {
            if (!publisher_thread_.joinable()) publisher_request_stop_.store(true);
            publisher_thread_.join();
        }

    protected:
        virtual void on_service_initialize() {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] initializing service callbacks" << std::endl;
        }

        virtual void on_publisher_initialize() {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] initializing publisher thread" << std::endl;
            publisher_thread_ = std::thread([this] { publisher_callback(); });
        }

        virtual void publisher_callback() {
            
        }

        cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
        cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;

        std::atomic_bool                         publisher_request_stop_;

    private:
        std::thread                              publisher_thread_;
    };
};
