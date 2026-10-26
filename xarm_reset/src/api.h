#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define XarmReset_DLLIMPORT __declspec(dllimport)
#  define XarmReset_DLLEXPORT __declspec(dllexport)
#  define XarmReset_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define XarmReset_DLLIMPORT __attribute__((visibility("default")))
#    define XarmReset_DLLEXPORT __attribute__((visibility("default")))
#    define XarmReset_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define XarmReset_DLLIMPORT
#    define XarmReset_DLLEXPORT
#    define XarmReset_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef XarmReset_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define XarmReset_DLLAPI
#  define XarmReset_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef XarmReset_EXPORTS
#    define XarmReset_DLLAPI XarmReset_DLLEXPORT
#  else
#    define XarmReset_DLLAPI XarmReset_DLLIMPORT
#  endif // XarmReset_EXPORTS
#  define XarmReset_LOCAL XarmReset_DLLLOCAL
#endif // XarmReset_STATIC