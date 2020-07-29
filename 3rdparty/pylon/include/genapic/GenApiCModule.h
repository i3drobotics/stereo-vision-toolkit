/*-----------------------------------------------------------------------------
    Basler pylon C SDK
    Copyright (c) 2012-2019 Basler AG
    http://www.baslerweb.com
-----------------------------------------------------------------------------*/
#ifndef GENAPI_GENAPICMODULE_H_
#define GENAPI_GENAPICMODULE_H_

/* doxygen comments removed intentionally */

#if defined( GENAPIC_EXPORTS )

#include <pylonc/PylonCModule.h>  // As long as PylonC and GenApiC are hosted within one DLL (module), this
                                  // include ensures that the PylonC module definition is used instead
                                  // of the module definition below.
                                  // When separating the modules, this include must be removed.


#ifndef GENAPIC_MODULE
#  define GENAPIC_MODULE "GenApiC"
[module(name=GENAPIC_MODULE, type=unspecified)]
#  ifdef _DEBUG
[idl_module(name=GENAPIC_MODULE, dllname="GenApiC_v6_0d.dll")];
#  else
[idl_module(name=GENAPIC_MODULE, dllname="GenApiC_v6_0.dll")];
#  endif
#endif

#endif


#endif
