//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2010-2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief A parameter class containing all parameters as members that are available for Instant Camera
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_INSTANTCAMERAPARAMS_H
#define BASLER_PYLON_INSTANTCAMERAPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_InstantCameraParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! A parameter class containing all parameters as members that are available for Instant Camera
    class CInstantCameraParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CInstantCameraParams_Params(void);

            //! Destructor
            ~CInstantCameraParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CInstantCameraParams_ParamsData;
        CInstantCameraParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief If set, this will automatically execute AcquisitionStart when calling StartGrabbing and AcquisitionStop when calling StopGrabbing  This option is enabled by default - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& AcquisitionStartStopExecutionEnable;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief Enables/disables the use of a chunk node map for each grab result  Grab result chunk node maps can be disabled to save resources - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& ChunkNodeMapsEnable;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief If set, all buffers will be cleared (set to 0) before grabbing an image  Note that this requires additional processing time per frame - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& ClearBufferModeEnable;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief Enables/disables the grabbing of camera events while images are grabbed  Is writable when the camera object is closed - Applies to: Instant Camera

    
        \b Visibility = Expert

    */
    Pylon::IBooleanEx& GrabCameraEvents;
    
    //@}


    //! \name GrabLoopThread - Parameters of the optional grab loop  thread.
    //@{
    /*!
        \brief The grab loop thread priority - Applies to: Instant Camera

        This value sets the absolute thread priority for the grab loop thread.
    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& GrabLoopThreadPriority;
    
    //@}


    //! \name GrabLoopThread - Parameters of the optional grab loop  thread.
    //@{
    /*!
        \brief If enabled, the user can set a custom priority for the grab loop thread  Otherwise, the priority of the newly created thread is not changed - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& GrabLoopThreadPriorityOverride;
    
    //@}


    //! \name GrabLoopThread - Parameters of the optional grab loop  thread.
    //@{
    /*!
        \brief A custom timeout for the grab loop thread's call to RetrieveResult  RetrieveResult is configured to throw an exception on timeout, which will stop the grab session - Applies to: Instant Camera

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& GrabLoopThreadTimeout;
    
    //@}


    //! \name GrabLoopThread - Parameters of the optional grab loop  thread.
    //@{
    /*!
        \brief If enabled, the user can set a custom timeout for the grab loop thread's call to RetrieveResult  RetrieveResult is configured to throw an exception on timeout, which will stop the grab session - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& GrabLoopThreadUseTimeout;
    
    //@}


    //! \name InternalGrabEngineThread - Parameters of the internal grab engine thread.
    //@{
    /*!
        \brief The internal grab engine thread priority - Applies to: Instant Camera

        This value sets the absolute thread priority for the internal grab engine thread operating the stream grabber.
    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& InternalGrabEngineThreadPriority;
    
    //@}


    //! \name InternalGrabEngineThread - Parameters of the internal grab engine thread.
    //@{
    /*!
        \brief If enabled, the user can set a custom priority for the internal grab engine thread operating the stream grabber  Otherwise the priority defaults to 25 - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& InternalGrabEngineThreadPriorityOverride;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The maximum number of buffers that are allocated and used for grabbing - Applies to: Instant Camera

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& MaxNumBuffer;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The maximum number of grab results available at any time during a grab session  This value can be limited to save resources  Furthermore, it can be used to check that the grab results are returned correctly - Applies to: Instant Camera

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& MaxNumGrabResults;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The maximum number of buffers that are queued in the stream grabber input queue - Applies to: Instant Camera

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& MaxNumQueuedBuffer;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The camera object is set to monitor mode when enabled, e g  when using the GigE multicast feature  Is writable when the camera object is closed - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& MonitorModeActive;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The number of empty buffers that are not used for grabbing yet - Applies to: Instant Camera

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& NumEmptyBuffers;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The number of buffers queued at Low Level API stream grabber - Applies to: Instant Camera

        
        
            This is the number of buffers that are queued for grabbing in the stream grabber.
            The number is influenced by the number of available free buffers and the
            maximum number of buffers that can be queued.
            See also the MaxNumBuffer and MaxNumQueuedBuffer parameters.

            This parameter can be used to check whether the number of buffers ready for grabbing
            is stable, which means that the image processing is fast enough to keep up with the
            rate of incoming images when using the GrabStrategy_OneByOne grab strategy.
        
        
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& NumQueuedBuffers;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The number of grab result buffers in the output queue that are ready for retrieval - Applies to: Instant Camera

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& NumReadyBuffers;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief The size of the grab result buffer output queue - Applies to: Instant Camera

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& OutputQueueSize;
    
    //@}


    //! \name Root - Instant camera parameters.
    //@{
    /*!
        \brief If larger than 0, a static chunk node map pool is used instead of dynamic chunk node map creation - Applies to: Instant Camera

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& StaticChunkNodeMapPoolSize;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CInstantCameraParams_Params(CInstantCameraParams_Params&);

            //! not implemented assignment operator
            CInstantCameraParams_Params& operator=(CInstantCameraParams_Params&);

        //! \endcond
    };



    //**************************************************************************************************
    // Parameter class implementation
    //**************************************************************************************************

    //! \cond HIDE_CLASS_METHODS




    ///////////////////////////////////////////////////////////////////////////
    //
    namespace EnumParameterClasses
    {

    }


    class CInstantCameraParams_Params::CInstantCameraParams_ParamsData
    {
    public:
        Pylon::CBooleanParameter AcquisitionStartStopExecutionEnable;
        Pylon::CBooleanParameter ChunkNodeMapsEnable;
        Pylon::CBooleanParameter ClearBufferModeEnable;
        Pylon::CBooleanParameter GrabCameraEvents;
        Pylon::CIntegerParameter GrabLoopThreadPriority;
        Pylon::CBooleanParameter GrabLoopThreadPriorityOverride;
        Pylon::CIntegerParameter GrabLoopThreadTimeout;
        Pylon::CBooleanParameter GrabLoopThreadUseTimeout;
        Pylon::CIntegerParameter InternalGrabEngineThreadPriority;
        Pylon::CBooleanParameter InternalGrabEngineThreadPriorityOverride;
        Pylon::CIntegerParameter MaxNumBuffer;
        Pylon::CIntegerParameter MaxNumGrabResults;
        Pylon::CIntegerParameter MaxNumQueuedBuffer;
        Pylon::CBooleanParameter MonitorModeActive;
        Pylon::CIntegerParameter NumEmptyBuffers;
        Pylon::CIntegerParameter NumQueuedBuffers;
        Pylon::CIntegerParameter NumReadyBuffers;
        Pylon::CIntegerParameter OutputQueueSize;
        Pylon::CIntegerParameter StaticChunkNodeMapPoolSize;
    };


    ///////////////////////////////////////////////////////////////////////////
    //
    inline CInstantCameraParams_Params::CInstantCameraParams_Params(void)
        : m_pData(new CInstantCameraParams_ParamsData())
        , AcquisitionStartStopExecutionEnable(m_pData->AcquisitionStartStopExecutionEnable)
        , ChunkNodeMapsEnable(m_pData->ChunkNodeMapsEnable)
        , ClearBufferModeEnable(m_pData->ClearBufferModeEnable)
        , GrabCameraEvents(m_pData->GrabCameraEvents)
        , GrabLoopThreadPriority(m_pData->GrabLoopThreadPriority)
        , GrabLoopThreadPriorityOverride(m_pData->GrabLoopThreadPriorityOverride)
        , GrabLoopThreadTimeout(m_pData->GrabLoopThreadTimeout)
        , GrabLoopThreadUseTimeout(m_pData->GrabLoopThreadUseTimeout)
        , InternalGrabEngineThreadPriority(m_pData->InternalGrabEngineThreadPriority)
        , InternalGrabEngineThreadPriorityOverride(m_pData->InternalGrabEngineThreadPriorityOverride)
        , MaxNumBuffer(m_pData->MaxNumBuffer)
        , MaxNumGrabResults(m_pData->MaxNumGrabResults)
        , MaxNumQueuedBuffer(m_pData->MaxNumQueuedBuffer)
        , MonitorModeActive(m_pData->MonitorModeActive)
        , NumEmptyBuffers(m_pData->NumEmptyBuffers)
        , NumQueuedBuffers(m_pData->NumQueuedBuffers)
        , NumReadyBuffers(m_pData->NumReadyBuffers)
        , OutputQueueSize(m_pData->OutputQueueSize)
        , StaticChunkNodeMapPoolSize(m_pData->StaticChunkNodeMapPoolSize)
    {
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline CInstantCameraParams_Params::~CInstantCameraParams_Params(void)
    {
        delete m_pData;
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline void CInstantCameraParams_Params::_Initialize(GENAPI_NAMESPACE::INodeMap* _Ptr)
    {
        m_pData->AcquisitionStartStopExecutionEnable.Attach(_Ptr, "AcquisitionStartStopExecutionEnable");
        m_pData->ChunkNodeMapsEnable.Attach(_Ptr, "ChunkNodeMapsEnable");
        m_pData->ClearBufferModeEnable.Attach(_Ptr, "ClearBufferModeEnable");
        m_pData->GrabCameraEvents.Attach(_Ptr, "GrabCameraEvents");
        m_pData->GrabLoopThreadPriority.Attach(_Ptr, "GrabLoopThreadPriority");
        m_pData->GrabLoopThreadPriorityOverride.Attach(_Ptr, "GrabLoopThreadPriorityOverride");
        m_pData->GrabLoopThreadTimeout.Attach(_Ptr, "GrabLoopThreadTimeout");
        m_pData->GrabLoopThreadUseTimeout.Attach(_Ptr, "GrabLoopThreadUseTimeout");
        m_pData->InternalGrabEngineThreadPriority.Attach(_Ptr, "InternalGrabEngineThreadPriority");
        m_pData->InternalGrabEngineThreadPriorityOverride.Attach(_Ptr, "InternalGrabEngineThreadPriorityOverride");
        m_pData->MaxNumBuffer.Attach(_Ptr, "MaxNumBuffer");
        m_pData->MaxNumGrabResults.Attach(_Ptr, "MaxNumGrabResults");
        m_pData->MaxNumQueuedBuffer.Attach(_Ptr, "MaxNumQueuedBuffer");
        m_pData->MonitorModeActive.Attach(_Ptr, "MonitorModeActive");
        m_pData->NumEmptyBuffers.Attach(_Ptr, "NumEmptyBuffers");
        m_pData->NumQueuedBuffers.Attach(_Ptr, "NumQueuedBuffers");
        m_pData->NumReadyBuffers.Attach(_Ptr, "NumReadyBuffers");
        m_pData->OutputQueueSize.Attach(_Ptr, "OutputQueueSize");
        m_pData->StaticChunkNodeMapPoolSize.Attach(_Ptr, "StaticChunkNodeMapPoolSize");
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline const char* CInstantCameraParams_Params::_GetVendorName(void)
    {
        return "Basler";
    }


    ///////////////////////////////////////////////////////////////////////////
    //
    inline const char* CInstantCameraParams_Params::_GetModelName(void)
    {
        return "InstantCameraParams";
    }

    //! \endcond

} // namespace Basler_InstantCameraParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_INSTANTCAMERAPARAMS_H
