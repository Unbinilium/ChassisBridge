#pragma once

#include <cstdint>
#include <type_traits>

namespace ct::types {
    namespace underlying {
#   pragma pack(push, 1)
        struct head {
            char flag;
        };

        struct asix_tuple {
            float x;
            float y;
            float z;
        };

        using volocity     = asix_tuple;
        using acceleration = asix_tuple;
        using diffusion    = asix_tuple;

        namespace rx {
            struct body {
                uint32_t     current_action_id;
                double       measure_timestamp;
                volocity     chassis_volocity;
                acceleration chassis_acceleration;
            };
        };

        namespace tx {
            template <typename Axis_Tuple>
            struct body {
                uint32_t   action_id;
                double     action_timestamp;
                const char action_type = [&] {
                    if      constexpr (std::is_same_v<Axis_Tuple, volocity>)     return 'V';
                    else if constexpr (std::is_same_v<Axis_Tuple, acceleration>) return 'A';
                    else if constexpr (std::is_same_v<Axis_Tuple, diffusion>)    return 'D';
                };
                Axis_Tuple tuple;
            };

            template <typename T>
            struct frame {
                head    header;
                body<T> data;
            };
        };
#   pragma pack(pop)
    };
};
