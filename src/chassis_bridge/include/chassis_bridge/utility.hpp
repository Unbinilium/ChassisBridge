#pragma once

#include <memory>
#include <chrono>
#include <cstdint>
#include <type_traits>

#include "chassis_bridge/types.hpp"

namespace cb::utility {
    template <typename Type = uint32_t, typename Precision = std::chrono::microseconds>
    constexpr auto get_current_timestamp() {
        return static_cast<Type>(
            std::chrono::duration_cast<Precision>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
    }

    template <typename MsgType>
    constexpr auto msg_from_tuple(const auto& tuple) {
        if constexpr (std::is_same_v<MsgType, chassis_interfaces::msg::VelocityInfo>) {
            auto volocity{MsgType()};
            volocity.move_velocity_x   = tuple.x;
            volocity.move_velocity_y   = tuple.y;
            volocity.rotate_velocity_z = tuple.z;
            return volocity;
        } else if constexpr (std::is_same_v<MsgType, chassis_interfaces::msg::AccelerationInfo>) {
            auto acceleration{MsgType()};
            acceleration.move_acceleration_x   = tuple.x;
            acceleration.move_acceleration_y   = tuple.y;
            acceleration.rotate_acceleration_z = tuple.z;
            return acceleration;
        }
    }

    template <typename RequestType>
    constexpr auto request_to_tuple(const RequestType& request) {
        auto tuple{cb::types::underlying::axis_tuple()};
        if constexpr (std::is_same_v<RequestType, std::shared_ptr<chassis_interfaces::srv::VolocityControl::Request>>) {
            tuple.x = request->target_move_volocity_x;
            tuple.y = request->target_move_volocity_y;
            tuple.z = request->target_rotate_volocity_z;
        } else if constexpr (std::is_same_v<RequestType, std::shared_ptr<chassis_interfaces::srv::AccelerationControl::Request>>) {
            tuple.x = request->target_move_acceleration_x;
            tuple.y = request->target_move_acceleration_y;
            tuple.z = request->target_rotate_acceleration_z;
        } else if constexpr (std::is_same_v<RequestType, std::shared_ptr<chassis_interfaces::srv::DiffusionControl::Request>>) {
            tuple.x = request->move_distance_x;
            tuple.y = request->move_distance_y;
            tuple.z = request->rotate_angle_z;
        }
        return tuple;
    }
}
