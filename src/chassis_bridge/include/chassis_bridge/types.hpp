#pragma once

#include <cstdint>

#include "chassis_interfaces/msg/velocity_info.hpp"
#include "chassis_interfaces/msg/acceleration_info.hpp"
#include "chassis_interfaces/msg/chassis_info.hpp"
#include "chassis_interfaces/msg/action_info.hpp"
#include "chassis_interfaces/srv/volocity_control.hpp"
#include "chassis_interfaces/srv/acceleration_control.hpp"
#include "chassis_interfaces/srv/diffusion_control.hpp"

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
            struct data {
                uint16_t     current_action_id;
                float        measure_timestamp;
                volocity     chassis_volocity;
                acceleration chassis_acceleration;
            };

            struct frame {
                const head header = cb::protocol::header::receive;
                data       body;
            };
        };

        namespace tx {
            struct data {
                uint16_t   action_id;
                float      action_timestamp;
                char       action_type;
                axis_tuple tuple;
            };

            struct frame {
                const head header = cb::protocol::header::transmit;
                data       body;
            };

            struct heartbeat {
                const head header = cb::protocol::header::heartbeat;
            };
        };
#   pragma pack(pop)
    };
};
