#ifndef TISUDSHL_H
#define TISUDSHL_H

#pragma warning( push )
#pragma warning( disable : 4996 ) // deprecated warnings
#pragma warning( disable : 4100 ) // warning C4100: unreferenced formal parameter
#pragma warning( disable : 4786 ) // warning: identifier was truncated (VC6)

#include <string>
#include <cassert>
#include <vector>
#include <shlwapi.h>

// DLL public interface 
#include "libbasedefs.h"        // namespace definition
#include "udshl_defs.h"

#include "dshow_header.h"

// linker helper, to bind to the UDSHL
#include "udshl_lnk.h"

#include "smart_ptr.h"
#include "smart_com.h"

#include "simplectypes.h"

#include "int_interface_pre.h"

#include "VideoFormatItem.h"
#include "VideoFormatDesc.h"
#include "VideoNormItem.h"
#include "VideoCaptureDeviceItem.h"
#include "AnalogChannelItem.h"

#include "Error.h"
#include "Grabber.h"
#include "MemBufferCollection.h"
#include "MemBuffer.h"
#include "Framegrabbersink.h"
#include "AviSink.h"
#include "OverlayBitmap.h"
#include "GrabberListener.h"

#include "FrameHandlerSink.h"
#include "MediaStreamSink.h"

#include "IVCDProperty.h"
#include "IVCDPropertyInterfaces.h"
#include "VCDPropertyDef.h"

#include "VCDPropertyID.h"
#include "VCDPropertyIDTIS.h"

#include "FilterLoader.h"
#include "FrameFilter.h"
#include "FrameFilterImpl.h"

#pragma warning( pop )

#endif /* TISUDSHL_H */
