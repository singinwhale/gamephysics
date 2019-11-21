#ifndef __util_h__
#define __util_h__


#include <string>


std::wstring GetExePath();

void UpdateWindowTitle(const std::wstring& appName);

#define sqr(value) value*value

#define USE_VERBOSE false
#define VERBOSE(code) if(USE_VERBOSE) {code;}

#endif
