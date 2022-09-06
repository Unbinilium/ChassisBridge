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
#include "chassis_bridge/utility.hpp"

namespace cb::nodes {
    class bridge : public rclcpp::Node {
        using rx_deque_item = std::shared_ptr<cb::types::underlying::rx::frame>;
        using tx_deque_item = std::shared_ptr<cb::types::underlying::tx::frame>;
    public:
        bridge(
            std::string&                             node_name,
            cb::container::ts::deque<rx_deque_item>* receive_deque_ptr,
            cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr
        ) : Node(node_name),
            receive_deque_ptr_(receive_deque_ptr),
            transmit_deque_ptr_(transmit_deque_ptr),
            publisher_request_stop_(false),
            publisher_helper_(cb::types::helper::publisher()) {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] initializing bridge node: " << node_name << std::endl;
            on_publisher_initialize();
            on_service_initialize(); 
        }

        ~bridge() {
            if (!publisher_thread_.joinable()) publisher_request_stop_.store(true);
            publisher_thread_.join();
        }

    protected:
        virtual void on_publisher_initialize() {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] creating publishers" << std::endl;
            publisher_helper_.volocity     = create_publisher<chassis_interfaces::msg::VelocityInfo>("volocity", 10);
            publisher_helper_.acceleration = create_publisher<chassis_interfaces::msg::AccelerationInfo>("acceleration", 10);
            publisher_helper_.action       = create_publisher<chassis_interfaces::msg::ActionInfo>("action", 10);   
            publisher_thread_ = std::thread([this] {
                std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                          << "[publisher] spawned publisher thread: " << std::this_thread::get_id() << std::endl;
                publisher_callback();
            });
        }

        virtual void publisher_callback() {
            receive_deque_ptr_->wait();
            auto frame{receive_deque_ptr_->front()};
            {
                auto volocity    {cb::utility::tuple_cast_msg(frame->body.chassis_volocity)};
                auto acceleration{cb::utility::tuple_cast_msg(frame->body.chassis_acceleration)};
                auto action      {chassis_interfaces::msg::ActionInfo()};
                action.receive_timestamp                  = cb::utility::get_current_timestamp();
                action.measure_timestamp                  = frame->body.measure_timestamp;
                action.current_action_id                  = frame->body.current_action_id;
                action.distance_moved_since_prev_action_x = frame->body.chassis_diffusion.x;
                action.distance_moved_since_prev_action_x = frame->body.chassis_diffusion.x;
                action.angle_rotated_since_prev_action_z  = frame->body.chassis_diffusion.z;
                publisher_helper_.volocity->publish(std::move(volocity));
                publisher_helper_.acceleration->publish(std::move(acceleration));
                publisher_helper_.action->publish(std::move(action));
            }
            receive_deque_ptr_->pop_front();
            std::this_thread::yield();
            publisher_callback();
        }

        virtual void on_service_initialize() {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] registering service callbacks" << std::endl;

        }

        cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
        cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;

        std::atomic_bool                         publisher_request_stop_;
        cb::types::helper::publisher             publisher_helper_;

    private:
        std::thread                              publisher_thread_;
    };
};
