//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2020 Basler AG
//  http://www.baslerweb.com
//  Author:  Arne Wischmann
//-----------------------------------------------------------------------------
/*!
\file
\brief Helper functions for different camera configuration classes.

       These helper functions are provided as a header-only file.
       The code can be copied and modified for creating your own configuration classes.
*/

#ifndef INCLUDED_CONFIGURATIONHELPER_H_6526739
#define INCLUDED_CONFIGURATIONHELPER_H_6526739


#include <pylon/Platform.h>

#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/ParameterIncludes.h>

namespace Pylon
{
    /** \addtogroup Pylon_InstantCameraApiGeneric
    * @{
    */

    /*!
    \class  CConfigurationHelper
    \brief Helper functions for different camera configuration classes.
    */
    class CConfigurationHelper
    {
    public:

        /*!
        \brief DisableAllTriggers disables all trigger types that can be
        turned off.
        */
        static void DisableAllTriggers( GENAPI_NAMESPACE::INodeMap& nodemap )
        {
            using namespace GENAPI_NAMESPACE;

            //Disable all trigger types.
            {
                // Get required enumerations.
                CEnumParameter triggerSelector( nodemap, "TriggerSelector" );
                CEnumParameter triggerMode( nodemap, "TriggerMode" );

                if (triggerSelector.IsWritable())
                {
                    // Get all settable enumeration entries of trigger selector.
                    StringList_t triggerSelectorEntries;
                    triggerSelector.GetSettableValues( triggerSelectorEntries );

                    // Turn trigger mode off for all trigger selector entries.
                    for (StringList_t::const_iterator it = triggerSelectorEntries.begin(); it != triggerSelectorEntries.end(); ++it)
                    {
                        // Set trigger mode to off if the trigger is available.
                        triggerSelector.SetValue( *it );
                        triggerMode.SetValue( "Off" );
                    }
                }
            }
        }

        /*!
        \brief DisableCompression disables all compressions modes that can be
               turned off.
        */
        static void DisableCompression( GENAPI_NAMESPACE::INodeMap& nodemap )
        {
            using namespace GENAPI_NAMESPACE;

            //Disable compression mode (if supported by the camera).
            CEnumParameter compressionMode( nodemap, "ImageCompressionMode" );

            if (compressionMode.IsWritable())
            {
                compressionMode.SetValue( "Off" );
            }
        }
    };

    /**
    * @}
    */
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif /* INCLUDED_CONFIGURATIONHELPER_H_6526739 */
