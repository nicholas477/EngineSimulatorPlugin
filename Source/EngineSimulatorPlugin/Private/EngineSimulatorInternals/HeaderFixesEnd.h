#if PLATFORM_WINDOWS
#include "Windows/PostWindowsApi.h"
#include "Windows/HideWindowsPlatformAtomics.h"
#include "Windows/HideWindowsPlatformTypes.h"

__pragma(pop_macro("PI"))
__pragma(pop_macro("TWO_PI"))

#endif