// Compatibility shim — definition lives in telemetry/xt_can.h (F12 jumps there).
#pragma once

#include "telemetry/xt_can.h"

namespace protocol
{
namespace xt_can = ::telemetry::xt_can;
}
