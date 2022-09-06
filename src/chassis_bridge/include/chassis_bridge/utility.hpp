#pragma once

#include <cstdint>
#include <chrono>

namespace cb::utility {
    template <typename Type = uint16_t, typename Precision = std::chrono::microseconds>
    auto get_current_timestamp() {
        return static_cast<Type>(
            std::chrono::duration_cast<Precision>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );
    }
};
