#pragma once
/*
#ifdef _WINDLL
#define METIS_API(type) __declspec(dllexport) type __cdecl
#elif defined(__cdecl)
#define METIS_API(type) type __cdecl
#else
#define METIS_API(type) type
#endif
*/

#ifndef METIS_DLL
# ifdef _WIN32
#  if defined( BUILD_DLL )
#    define METIS_DLL __declspec(dllexport)
#  elif defined( USE_DLL )
#    define METIS_DLL __declspec(dllimport)
#  else
#    define METIS_DLL
#  endif
# else
#  define METIS_DLL
# endif
#endif



#if defined(__cdecl)
#define METIS_API(type) METIS_DLL type __cdecl
#else
#define METIS_API(type) METIS_DLL type
#endif