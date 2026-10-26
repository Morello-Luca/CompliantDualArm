#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define FSMDualManipulation_DLLIMPORT __declspec(dllimport)
#  define FSMDualManipulation_DLLEXPORT __declspec(dllexport)
#  define FSMDualManipulation_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define FSMDualManipulation_DLLIMPORT __attribute__((visibility("default")))
#    define FSMDualManipulation_DLLEXPORT __attribute__((visibility("default")))
#    define FSMDualManipulation_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define FSMDualManipulation_DLLIMPORT
#    define FSMDualManipulation_DLLEXPORT
#    define FSMDualManipulation_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef FSMDualManipulation_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define FSMDualManipulation_DLLAPI
#  define FSMDualManipulation_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef FSMDualManipulation_EXPORTS
#    define FSMDualManipulation_DLLAPI FSMDualManipulation_DLLEXPORT
#  else
#    define FSMDualManipulation_DLLAPI FSMDualManipulation_DLLIMPORT
#  endif // FSMDualManipulation_EXPORTS
#  define FSMDualManipulation_LOCAL FSMDualManipulation_DLLLOCAL
#endif // FSMDualManipulation_STATIC