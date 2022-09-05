#pragma once

namespace cb::protocol {
    enum header : char {
        receive,
        transmit
    };

    enum action_types : char {
        volocity,
        acceleration,
        diffusion
    };
};
