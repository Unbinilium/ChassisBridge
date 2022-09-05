#pragma once

namespace cb::protocol {
    enum header : char {
        receive  = 'S',
        transmit = 'T'
    };

    enum action_types : char {
        volocity     = 'V',
        acceleration = 'A',
        diffusion    = 'D'
    };
};
