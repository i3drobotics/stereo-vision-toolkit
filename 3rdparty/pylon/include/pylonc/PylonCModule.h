/*-----------------------------------------------------------------------------
    Basler pylon C SDK
    Copyright (c) 2012-2020 Basler AG
    http://www.baslerweb.com
-----------------------------------------------------------------------------*/
#ifndef PYLON_PYLONMODULE_H_
#define PYLON_PYLONMODULE_H_

/* doxygen comments removed intentionally */

#define PYLONC_TYPELIB_GUID    "{C07E90EF-C385-3BF9-8BB2-A8B0858D3CB4}"
#define PYLONC_TYPELIB_GUID_W L"{C07E90EF-C385-3BF9-8BB2-A8B0858D3CB4}"

#define PYLONC_TYPELIB_VER_MAJOR 1
#define PYLONC_TYPELIB_VER_MINOR 0

#if defined( PYLONC_EXPORTS )

#  define PYLONC_MODULE "PylonC"
#  define GENAPIC_MODULE PYLONC_MODULE  // as long we share the same module as GenApiC, prevent the GenApiC headers from creating its own module the module
[module(name=PYLONC_MODULE, type=unspecified, uuid=PYLONC_TYPELIB_GUID)]
#  ifdef _DEBUG
[idl_module(name=PYLONC_MODULE, dllname="PylonC_v6_1d.dll")];
#  else
[idl_module(name=PYLONC_MODULE, dllname="PylonC_v6_1.dll")];
#  endif
#endif

#endif
