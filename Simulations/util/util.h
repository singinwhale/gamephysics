#ifndef __util_h__
#define __util_h__


#include <string>


std::wstring GetExePath();

void UpdateWindowTitle(const std::wstring& appName);

#define sqr(value) value*value
#ifndef USE_VERBOSE
#define USE_VERBOSE 1
#endif

#if USE_VERBOSE
#define VERBOSE(code) code
#else
#define VERBOSE(code) // empty
#endif
#endif
