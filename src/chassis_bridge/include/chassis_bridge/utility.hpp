#pragma once

#include <cstdint>
#include <chrono>
#include <type_traits>

#include "chassis_bridge/types.hpp"

namespace cb::utility {
    template <typename Type = uint16_t, typename Precision = std::chrono::microseconds>
    constexpr auto get_current_timestamp() {
        return static_cast<Type>(
            std::chrono::duration_cast<Precision>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
    }

    template <typename TupleType>
    constexpr auto tuple_cast_msg(const TupleType& tuple) {
        if constexpr (std::is_same_v<TupleType, cb::types::underlying::volocity>) {
            auto volocity{chassis_interfaces::msg::VelocityInfo()};
            volocity.move_velocity_x   = tuple.x;
            volocity.move_velocity_y   = tuple.y;
            volocity.rotate_velocity_z = tuple.z;
            return volocity;
        }
        if constexpr (std::is_same_v<TupleType, cb::types::underlying::acceleration>) {
            auto acceleration{chassis_interfaces::msg::AccelerationInfo()};
            acceleration.move_acceleration_x   = tuple.x;
            acceleration.move_acceleration_y   = tuple.y;
            acceleration.rotate_acceleration_z = tuple.z;
            return acceleration;
        }
    }
};
