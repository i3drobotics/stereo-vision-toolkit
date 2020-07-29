
/** This file checks the compiler version and binds to the udshl library and other libraries needed.
 * To use the link feature you must define the UDSHL_LIB_BASEDIR macro.
 * UDSHL_LIB_BASEDIR must contain the relative path to the lib base directory of imaging control.
 *
 * For example this define is used by the samples :
	#define UDSHL_LIB_BASEDIR "..\..\..\ClassLib\"
 * This is automatically expanded to point to the right file for the build.
 *
 * You can prevent linking with anything by specifying UDSHL_LIB_NO_LINK.
 *
 * This file checks the following defines :
	_MSC_VER	// compiler version, defined by the compiler
 *
 * You may pass UDSHL_LIB_NO_COMPILER_CHECK to prevent the compiler checks.
 * 
 */

#ifndef UDSHL_LNK_H_INC_
#define UDSHL_LNK_H_INC_

#pragma once

#if !defined UDSHL_LIB_DIR
	#if defined UDSHL_LIB_BASEDIR
		#ifdef _DEBUG
			#define UDSHL_LIB_DIR UDSHL_LIB_BASEDIR "debug/"
		#else
			#define UDSHL_LIB_DIR UDSHL_LIB_BASEDIR "release/"
		#endif
	#else 
		#define UDSHL_LIB_DIR ""
	#endif
#endif

#ifndef UDSHL_LIB_NO_COMPILER_CHECK

	#if !defined _MSC_VER
		#error Wrong Compiler. This library does only run with Visual C++ 6.0, 7.1, 8.0, 9.0 and 10.0
	#else
		#if _MSC_VER < 1200
			#error Wrong Compiler. This library does not run with Visual C++ prior to version 6.0
		#elif _MSC_VER == 1200
			// VC 6.0
		#elif _MSC_VER == 1300
			// VC7.0 not supported
			#error Wrong Compiler. This library does not run with Visual C++ version 7.0. Use 7.1 instead.
		#elif _MSC_VER == 1310
			// VC71
		#elif _MSC_VER == 1400
			// VC80
		#elif _MSC_VER == 1500
			// VC90
		#elif _MSC_VER == 1600
			// VC100
		#elif _MSC_VER == 1700
			// VC110
		#elif _MSC_VER == 1800
			// vc120
                #elif _MSC_VER == 1900
                    // vc14 / VS2015
                #elif _MSC_VER == 1910
                 // vc14 / VS2017
                #endif
        #endif

	#if defined(_WIN32_WINNT) && (_WIN32_WINNT < 0x0500)
	#	error "This library requires _WIN32_WINNT to be at least 0x0500 (Windows 2000)"
	#endif
#endif

// #define UDSHL_BASE_NAME is in libbasedefs.h
#define __UDSHL_LPREFIX(txt)	L##txt
#define _UDSHL_LPREFIX(txt)		__UDSHL_LPREFIX(txt)

#if defined _DEBUG
#	define UDSHL_SUFFIX_CONFIGURATION "d"
#else 
#	define UDSHL_SUFFIX_CONFIGURATION ""
#endif

#if defined _M_AMD64
#	define UDSHL_SUFFIX_PLATFORM "_x64"
#else 
#	define UDSHL_SUFFIX_PLATFORM ""
#endif

#define UDSHL_NAME		UDSHL_NAME_BASE UDSHL_SUFFIX_CONFIGURATION UDSHL_SUFFIX_PLATFORM
#define UDSHL_NAME_W	_UDSHL_LPREFIX(UDSHL_NAME_BASE) _UDSHL_LPREFIX(UDSHL_SUFFIX_CONFIGURATION) _UDSHL_LPREFIX(UDSHL_SUFFIX_PLATFORM)

#define UDSHL_DLL_NAME		UDSHL_NAME ".dll"
#define UDSHL_LIB_NAME		UDSHL_NAME ".lib"
#define UDSHL_DLL_NAME_W	UDSHL_NAME_W L".dll"
#define UDSHL_LIB_NAME_W	UDSHL_NAME_W L".lib"

#if !defined UDSHL_LIB_NO_LINK
	// other libraries sometimes needed by applications using this library
	#include <comdef.h>		// commsupp.lib
	
	#pragma comment ( lib, UDSHL_LIB_DIR UDSHL_LIB_NAME )
#endif

// remove previous defined macro
#undef UDSHL_LIB_DIR

#endif // UDSHL_LNK_H_INC_
