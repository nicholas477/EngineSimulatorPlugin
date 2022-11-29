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