#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define DualCheck_DLLIMPORT __declspec(dllimport)
#  define DualCheck_DLLEXPORT __declspec(dllexport)
#  define DualCheck_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define DualCheck_DLLIMPORT __attribute__((visibility("default")))
#    define DualCheck_DLLEXPORT __attribute__((visibility("default")))
#    define DualCheck_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define DualCheck_DLLIMPORT
#    define DualCheck_DLLEXPORT
#    define DualCheck_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef DualCheck_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define DualCheck_DLLAPI
#  define DualCheck_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef DualCheck_EXPORTS
#    define DualCheck_DLLAPI DualCheck_DLLEXPORT
#  else
#    define DualCheck_DLLAPI DualCheck_DLLIMPORT
#  endif // DualCheck_EXPORTS
#  define DualCheck_LOCAL DualCheck_DLLLOCAL
#endif // DualCheck_STATIC