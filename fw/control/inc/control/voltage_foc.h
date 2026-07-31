// Compatibility shim — definition lives in foc_ctrl/voltage_foc.h (F12 jumps there).
#pragma once

#include "foc_ctrl/voltage_foc.h"

namespace control
{
using VoltageFoc = ::foc_ctrl::VoltageFoc;
}
