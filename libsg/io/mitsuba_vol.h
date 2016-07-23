#pragma once

#include "libsg.h"  // NOLINT

namespace sg {

namespace vis { struct IFVisParams; }

namespace io {

//! Write IF to Mitsuba volume (see https://www.mitsuba-renderer.org/releases/current/documentation.pdf 8.7.2)
//! Return 1 if success, 0 if error
int write_mitsuba_vol(const interaction::InteractionFrame& iframe,
                      const vis::IFVisParams& p,
                      const string& file);

}  // namespace io
}  // namespace sg


