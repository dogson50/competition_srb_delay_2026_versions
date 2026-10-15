#pragma once

#include <array>
#include <cstdint>

// Integer ps correction learned from the frozen public training split.  The key
// is endpoint category * 24 + cross-column flag * 12 + |dx|/10 bin.
inline constexpr std::array<std::int8_t, 144> kJointTpCrossAdxCorrection = {
    1,0,0,0,0,0,0,0,1,0,0,0,0,6,4,3,2,1,1,1,1,1,1,1,2,1,1,0,0,0,0,1,1,0,0,0,0,8,5,3,2,2,2,1,1,1,1,1,4,2,2,1,1,1,1,2,2,0,0,0,0,17,10,7,5,3,3,3,3,2,2,2,1,1,0,0,0,0,0,1,1,0,0,0,0,6,4,3,2,2,1,1,1,1,1,1,1,1,0,0,0,0,0,1,1,0,0,0,0,7,4,3,2,2,2,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
