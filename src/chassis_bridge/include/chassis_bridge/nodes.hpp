#pragma once

#include <atomic>
#include <memory>
#include <chrono>
#include <thread>
#include <string>
#include <utility>
#include <functional>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

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
            publisher_helper_({}),
            service_helper_({}),
            action_id_(1) {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] initializing bridge node: " << node_name << std::endl;
            on_publisher_initialize();
            on_service_initialize(); 
        }

        ~bridge() {
            rclcpp::shutdown();
            publisher_thread_.detach();
            service_thread_.detach();
        }

        void threads_join() {
            publisher_thread_.join();
            service_thread_.join();
        }

    protected:
        virtual void on_publisher_initialize() {
            std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                      << "[bridge node] creating publishers" << std::endl;
            publisher_helper_.volocity     = create_publisher<chassis_interfaces::msg::VelocityInfo>("volocity_info", 10);
            publisher_helper_.acceleration = create_publisher<chassis_interfaces::msg::AccelerationInfo>("acceleration_info", 10);
            publisher_helper_.action       = create_publisher<chassis_interfaces::msg::ActionInfo>("action_info", 10);   
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
                auto volocity    {cb::utility::msg_from_tuple<chassis_interfaces::msg::VelocityInfo>(frame->body.chassis_volocity)};
                auto acceleration{cb::utility::msg_from_tuple<chassis_interfaces::msg::AccelerationInfo>(frame->body.chassis_acceleration)};
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
                      << "[bridge node] registering service callbacks and set action id to: " << action_id_.load() << std::endl;
            service_helper_.volocity = create_service<chassis_interfaces::srv::VolocityControl>("volocity_control", std::bind(
                &bridge::service_callback<chassis_interfaces::srv::VolocityControl::Request, chassis_interfaces::srv::VolocityControl::Response>,
                this, std::placeholders::_1, std::placeholders::_2
            ));
            service_helper_.acceleration = create_service<chassis_interfaces::srv::AccelerationControl>("acceleration_control", std::bind(
                &bridge::service_callback<chassis_interfaces::srv::AccelerationControl::Request, chassis_interfaces::srv::AccelerationControl::Response>,
                this, std::placeholders::_1, std::placeholders::_2
            ));
            service_helper_.diffusion = create_service<chassis_interfaces::srv::DiffusionControl>("diffusion_control", std::bind(
                &bridge::service_callback<chassis_interfaces::srv::DiffusionControl::Request, chassis_interfaces::srv::DiffusionControl::Response>,
                this, std::placeholders::_1, std::placeholders::_2
            ));
            auto self(shared_from_this());
            service_thread_ = std::thread([this, self] {
                std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                          << "[service] spawned service thread: " << std::this_thread::get_id() << std::endl;
                std::cout << std::chrono::system_clock::now().time_since_epoch().count() 
                          << "[service] service thread spin and wait for requests" << std::endl;
                rclcpp::spin(self);
            });
        }

        template <typename Request, typename Response>
        void service_callback(const std::shared_ptr<Request> request, std::shared_ptr<Response> response) {
            auto action_id{action_id_.fetch_add(1)};
            auto frame{std::make_shared<cb::types::underlying::tx::frame>()};
            frame->body.action_id        = action_id;
            frame->body.action_timestamp = cb::utility::get_current_timestamp();
            frame->body.action_type      = cb::protocol::action_types::volocity;
            frame->body.tuple            = cb::utility::request_to_tuple(request);
            transmit_deque_ptr_->push_back(std::move(frame));
            response->action_id = action_id;
        }

        cb::container::ts::deque<rx_deque_item>* receive_deque_ptr_;
        cb::container::ts::deque<tx_deque_item>* transmit_deque_ptr_;

        cb::types::helper::publisher             publisher_helper_;
        cb::types::helper::service               service_helper_;

        std::atomic<uint16_t>                    action_id_;

    private:
        std::thread                              publisher_thread_;
        std::thread                              service_thread_;
    };
}
