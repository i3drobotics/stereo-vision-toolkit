//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief A parameter class containing all parameters as members that are available for Usb, and GigE
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_UNIVERSALEVENTPARAMS_H
#define BASLER_PYLON_UNIVERSALEVENTPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_UniversalEventParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************

    //! Valid values for Status
    enum StatusEnums
    {
        Status_Closed,  //!< The low level event grabber is closed. - Applies to: GigE
        Status_Open  //!< The low level event grabber is open. - Applies to: GigE
    };



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! A parameter class containing all parameters as members that are available for Usb, and GigE
    class PYLONBASE_API CUniversalEventParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CUniversalEventParams_Params(void);

            //! Destructor
            ~CUniversalEventParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CUniversalEventParams_ParamsData;
        CUniversalEventParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name Root - USB event grabber parameters.
    //@{
    /*!
        \brief Number of buffers to be used - Applies to: Usb and GigE

        Number of Buffers that are going to be used receiving events.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& NumBuffer;
    
    //@}


    //! \name Root - USB event grabber parameters.
    //@{
    /*!
        \brief Maximum number of USB request blocks (URBs) to be enqueued simultaneously - Applies to: Usb

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& NumMaxQueuedUrbs;
    
    //@}


    //! \name Root - USB event grabber parameters.
    //@{
    /*!
        \brief Maximum number of retries - Applies to: GigE

        Number retry attempts by the camera to get an acknowledge for a sent event message.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& RetryCount;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief The count of processed events with an error status - Applies to: Usb

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Failed_Event_Count;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief The status code of the last failed event buffer - Applies to: Usb

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Last_Failed_Event_Buffer_Status;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief The total count of processed events - Applies to: Usb

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Total_Event_Count;
    
    //@}


    //! \name Debug - For internal use only.
    //@{
    /*!
        \brief For internal use only - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IEnumParameterT<StatusEnums>& Status;
    
    //@}


    //! \name Root - USB event grabber parameters.
    //@{
    /*!
        \brief Acknowledge timeout in milliseconds - Applies to: GigE

        Time to wait by the camera if an acknowledge request is configured (RetryCount != 0) to wait until the acknowledge arrives before resending the event message on its own.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Timeout;
    
    //@}


    //! \name Root - USB event grabber parameters.
    //@{
    /*!
        \brief Priority of the thread that handles USB requests from the stream interface - Applies to: Usb

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& TransferLoopThreadPriority;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CUniversalEventParams_Params(CUniversalEventParams_Params&);

            //! not implemented assignment operator
            CUniversalEventParams_Params& operator=(CUniversalEventParams_Params&);

        //! \endcond
    };


} // namespace Basler_UniversalEventParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_UNIVERSALEVENTPARAMS_H
