#pragma once

#include "libsg.h"  // NOLINT

namespace sg {
namespace interaction {

// Writes header for interaction features to ofs and returns ofs
ofstream& writeInteractionFeatsHeader(ofstream& ofs);

// Computes and writes interaction feature rows to ofs, returning ofs
ofstream& writeInteractionFeats(ofstream& ofs,
                                const InteractionFactory& ifactory,
                                const vec<Interaction*>& interactions);

}  // namespace interaction
}  // namespace sg
