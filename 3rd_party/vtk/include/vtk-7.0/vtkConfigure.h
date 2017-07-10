/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkConfigure.h.in

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#ifndef vtkConfigure_h
#define vtkConfigure_h

/* This header is configured by VTK's build process.  */

/*--------------------------------------------------------------------------*/
/* Platform Features                                                        */

/* Byte order.  */
/* All compilers that support Mac OS X define either __BIG_ENDIAN__ or
   __LITTLE_ENDIAN__ to match the endianness of the architecture being
   compiled for. This is not necessarily the same as the architecture of the
   machine doing the building. In order to support Universal Binaries on
   Mac OS X, we prefer those defines to decide the endianness.
   On other platforms we use the result of the TRY_RUN. */
#if !defined(__APPLE__)
/* #undef VTK_WORDS_BIGENDIAN */
#elif defined(__BIG_ENDIAN__)
# define VTK_WORDS_BIGENDIAN
#endif

/* Threading system.  */
/* #undef VTK_USE_PTHREADS */
/* #undef VTK_USE_SPROC */
/* #undef VTK_HP_PTHREADS */
#define VTK_USE_WIN32_THREADS
# define VTK_MAX_THREADS 64

/* Atomic operations */
/* #undef VTK_HAVE_SYNC_BUILTINS */
#if defined(WIN32)
 #define VTK_HAS_INTERLOCKEDADD
#endif

/* Size of fundamental data types.  */
/* Mac OS X uses two data models, ILP32 (in which integers, long integers,
   and pointers are 32-bit quantities) and LP64 (in which integers are 32-bit
   quantities and long integers and pointers are 64-bit quantities). In order
   to support Universal Binaries on Mac OS X, we rely on this knowledge
   instead of testing the sizes of the building machine.
   On other platforms we use the result of the TRY_RUN. */
#if !defined(__APPLE__)
# define VTK_SIZEOF_CHAR   1
# define VTK_SIZEOF_SHORT  2
# define VTK_SIZEOF_INT    4
# define VTK_SIZEOF_LONG   4
# define VTK_SIZEOF_FLOAT  4
# define VTK_SIZEOF_DOUBLE 8
# define VTK_SIZEOF_VOID_P 8
#else
# define VTK_SIZEOF_CHAR   1
# define VTK_SIZEOF_SHORT  2
# define VTK_SIZEOF_INT    4
# if defined(__LP64__) && __LP64__
#  define VTK_SIZEOF_LONG  8
#  define VTK_SIZEOF_VOID_P 8
# else
#  define VTK_SIZEOF_LONG  4
#  define VTK_SIZEOF_VOID_P 4
# endif
# define VTK_SIZEOF_FLOAT  4
# define VTK_SIZEOF_DOUBLE 8
#endif

/* Define size of long long and/or __int64 bit integer type only if the type
   exists.  */
#if !defined(__APPLE__)
 #define VTK_SIZEOF_LONG_LONG 8
#else
 #define VTK_SIZEOF_LONG_LONG 8
#endif

/* Whether type "long long" is enabled as a unique fundamental type.  */
#define VTK_TYPE_USE_LONG_LONG

/* Whether type "char" is signed (it may be signed or unsigned).  */
#define VTK_TYPE_CHAR_IS_SIGNED 1

/* Compiler features.  */
/* #undef VTK_HAVE_GETSOCKNAME_WITH_SOCKLEN_T */
/* #undef VTK_HAVE_SO_REUSEADDR */

/* Whether we require large files support.  */
/* #undef VTK_REQUIRE_LARGE_FILE_SUPPORT */

/* Whether reverse const iterator's have comparison operators. */
#define VTK_CONST_REVERSE_ITERATOR_COMPARISON

/*--------------------------------------------------------------------------*/
/* VTK Platform Configuration                                               */

/* Whether the target platform supports shared libraries.  */
/* #undef VTK_TARGET_SUPPORTS_SHARED_LIBS */

/* Whether we are building shared libraries.  */
#define VTK_BUILD_SHARED_LIBS

/* Whether vtkIdType is a 64-bit integer type (or a 32-bit integer type).  */
#define VTK_USE_64BIT_IDS

#include "vtkVersionMacros.h" // removed by VTK_LEGACY_REMOVE

/* C++ compiler used.  */
#define VTK_CXX_COMPILER "C:/Program Files (x86)/Microsoft Visual Studio 14.0/VC/bin/x86_amd64/cl.exe"

/* Compatibility settings.  */
/* #undef VTK_LEGACY_REMOVE */
/* #undef VTK_LEGACY_SILENT */

/* Debug leaks support.  */
/* #undef VTK_DEBUG_LEAKS */

/* Should all New methods use the object factory override. */
/* #undef VTK_ALL_NEW_OBJECT_FACTORY */

/*--------------------------------------------------------------------------*/
/* Setup VTK based on platform features and configuration.                  */

/* We now always use standard streams.  */
#ifndef VTK_LEGACY_REMOVE
# define VTK_USE_ANSI_STDLIB
#endif

/* Setup vtkstd, a portable namespace for std.  */
#ifndef VTK_LEGACY_REMOVE
# define vtkstd std
#endif

/* Define a "vtkstd_bool" for backwards compatibility.  Only use bool
   if this file is included by a c++ file. */
#ifndef VTK_LEGACY_REMOVE
# if defined(__cplusplus)
  typedef bool vtkstd_bool;
# else
  typedef int vtkstd_bool;
# endif
#endif

/* The maximum length of a file name in bytes including the
 * terminating null.
 */
#if defined(PATH_MAX) // Usually defined on Windows
# define VTK_MAXPATH PATH_MAX
#elif defined(MAXPATHLEN)  // Usually defined on linux
# define VTK_MAXPATH MAXPATHLEN
#else
# define VTK_MAXPATH 32767 // Possible with Windows "extended paths"
#endif

/* #undef VTK_USE_CXX11_FEATURES */
#ifdef VTK_USE_CXX11_FEATURES
# define VTK_OVERRIDE override
# define VTK_FINAL final
#else
# define VTK_OVERRIDE
# define VTK_FINAL
#endif

#endif
