#pragma once

#include <cstdint>
#include <type_traits>

#include "chassis_bridge/protocol.hpp"

namespace cb::types {
    namespace underlying {
#   pragma pack(push, 1)
        using head = char;

        struct axis_tuple {
            float x;
            float y;
            float z;
        };

        using volocity     = axis_tuple;
        using acceleration = axis_tuple;
        using diffusion    = axis_tuple;

        namespace rx {
            struct body {
                uint16_t     current_action_id;
                float        measure_timestamp;
                volocity     chassis_volocity;
                acceleration chassis_acceleration;
            };

            struct frame {
                const head header = cb::protocol::header::receive;
                body       data;
            };
        };

        namespace tx {
            template <typename Axis_Tuple>
            struct body {
                uint16_t   action_id;
                float      action_timestamp;
                const char action_type = [&] {
                    if      constexpr (std::is_same_v<Axis_Tuple, volocity>)     return cb::protocol::action_types::volocity;
                    else if constexpr (std::is_same_v<Axis_Tuple, acceleration>) return cb::protocol::action_types::acceleration;
                    else if constexpr (std::is_same_v<Axis_Tuple, diffusion>)    return cb::protocol::action_types::diffusion;
                };
                Axis_Tuple tuple;
            };

            template <typename T>
            struct frame {
                const head header = cb::protocol::header::transmit;
                body<T>    data;
            };

            struct heartbeat {
                const head header = cb::protocol::header::heartbeat;
            };
        };
#   pragma pack(pop)
    };
};
