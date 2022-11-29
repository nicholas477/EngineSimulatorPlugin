#include "EngineSimulatorInternals/EngineSimulatorGUI.h"

#if PLATFORM_WINDOWS
__pragma(push_macro("PI"))
__pragma(push_macro("TWO_PI"))

#undef PI

#undef TWO_PI

#include "Windows/AllowWindowsPlatformTypes.h"
#include "Windows/AllowWindowsPlatformAtomics.h"
#include "Windows/PreWindowsApi.h"

#include <mmiscapi.h>

#pragma warning(disable : 4587)
#endif

#include "piston_object.h"
#include "connecting_rod_object.h"
#include "constants.h"
#include "units.h"
#include "crankshaft_object.h"
#include "cylinder_bank_object.h"
#include "cylinder_head_object.h"
#include "ui_button.h"
#include "combustion_chamber_object.h"
#include "csv_io.h"
#include "exhaust_system.h"
#include "feedback_comb_filter.h"
#include "utilities.h"

#include "compiler.h"

#include <chrono>
#include <stdlib.h>
#include <sstream>

#if ATG_ENGINE_SIM_DISCORD_ENABLED
#include "../discord/Discord.h"
#endif

#if PLATFORM_WINDOWS
#include "Windows/PostWindowsApi.h"
#include "Windows/HideWindowsPlatformAtomics.h"
#include "Windows/HideWindowsPlatformTypes.h"

__pragma(pop_macro("PI"))
__pragma(pop_macro("TWO_PI"))

#endif
