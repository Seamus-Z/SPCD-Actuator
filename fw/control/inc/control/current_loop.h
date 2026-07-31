// Compatibility shim — definition lives in foc_ctrl/current_loop.h (F12 jumps there).
#pragma once

#include "foc_ctrl/current_loop.h"

namespace control
{
using CurrentLoop = ::foc_ctrl::CurrentLoop;
}
