//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2019 Basler AG
//  http://www.baslerweb.com
//-----------------------------------------------------------------------------

/*!
\file
\brief A parameter class containing all parameters as members that are available for Usb, GigE, BCON, and CoaXPress
*/

//-----------------------------------------------------------------------------
//  This file is generated automatically
//  Do not modify!
//-----------------------------------------------------------------------------

#ifndef BASLER_PYLON_UNIVERSALTLPARAMS_H
#define BASLER_PYLON_UNIVERSALTLPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_UniversalTLParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************

    //! Valid values for Areatriggermode
    enum AreatriggermodeEnums
    {
        Areatriggermode_Generator,  //!< Applies to: CoaXPress
        Areatriggermode_External,  //!< Applies to: CoaXPress
        Areatriggermode_Software  //!< Applies to: CoaXPress
    };

    //! Valid values for Bitalignment
    enum BitalignmentEnums
    {
        Bitalignment_LeftAligned,  //!< Applies to: CoaXPress
        Bitalignment_RightAligned,  //!< Applies to: CoaXPress
        Bitalignment_CustomBitShift  //!< Applies to: CoaXPress
    };

    //! Valid values for CamerasimulatorEnable
    enum CamerasimulatorEnableEnums
    {
        CamerasimulatorEnable_Camera,  //!< Applies to: CoaXPress
        CamerasimulatorEnable_Simulator  //!< Applies to: CoaXPress
    };

    //! Valid values for CamerasimulatorPattern
    enum CamerasimulatorPatternEnums
    {
        CamerasimulatorPattern_Horizontal,  //!< Applies to: CoaXPress
        CamerasimulatorPattern_Vertical,  //!< Applies to: CoaXPress
        CamerasimulatorPattern_Diagonal  //!< Applies to: CoaXPress
    };

    //! Valid values for CamerasimulatorRoll
    enum CamerasimulatorRollEnums
    {
        CamerasimulatorRoll_On,  //!< Applies to: CoaXPress
        CamerasimulatorRoll_Off  //!< Applies to: CoaXPress
    };

    //! Valid values for CamerasimulatorSelectMode
    enum CamerasimulatorSelectModeEnums
    {
        CamerasimulatorSelectMode_PixelFrequency,  //!< Applies to: CoaXPress
        CamerasimulatorSelectMode_LineRate,  //!< Applies to: CoaXPress
        CamerasimulatorSelectMode_FrameRate  //!< Applies to: CoaXPress
    };

    //! Valid values for CamerasimulatorTriggerMode
    enum CamerasimulatorTriggerModeEnums
    {
        CamerasimulatorTriggerMode_FreeRun,  //!< Applies to: CoaXPress
        CamerasimulatorTriggerMode_RisingEdgeTriggersLine,  //!< Applies to: CoaXPress
        CamerasimulatorTriggerMode_RisingEdgeTriggersFrame  //!< Applies to: CoaXPress
    };

    //! Valid values for CxpDebugPort
    enum CxpDebugPortEnums
    {
        CxpDebugPort_Yes,  //!< Applies to: CoaXPress
        CxpDebugPort_No  //!< Applies to: CoaXPress
    };

    //! Valid values for CxpTriggerPacketMode
    enum CxpTriggerPacketModeEnums
    {
        CxpTriggerPacketMode_Standard,  //!< Applies to: CoaXPress
        CxpTriggerPacketMode_RisingEdgeOnly  //!< Applies to: CoaXPress
    };

    //! Valid values for Debuginenable
    enum DebuginenableEnums
    {
        Debuginenable_On,  //!< Applies to: CoaXPress
        Debuginenable_Off  //!< Applies to: CoaXPress
    };

    //! Valid values for Debuginsert
    enum DebuginsertEnums
    {
        Debuginsert_Apply  //!< Applies to: CoaXPress
    };

    //! Valid values for Debugoutenable
    enum DebugoutenableEnums
    {
        Debugoutenable_On,  //!< Applies to: CoaXPress
        Debugoutenable_Off  //!< Applies to: CoaXPress
    };

    //! Valid values for Debugready
    enum DebugreadyEnums
    {
        Debugready_Yes,  //!< Applies to: CoaXPress
        Debugready_No  //!< Applies to: CoaXPress
    };

    //! Valid values for Debugwriteflag
    enum DebugwriteflagEnums
    {
        Debugwriteflag_Endofline,  //!< Applies to: CoaXPress
        Debugwriteflag_Endofframe  //!< Applies to: CoaXPress
    };

    //! Valid values for DeviceAccessStatus
    enum DeviceAccessStatusEnums
    {
        DeviceAccessStatus_Unknown,  //!< Applies to: CoaXPress
        DeviceAccessStatus_ReadWrite,  //!< Applies to: CoaXPress
        DeviceAccessStatus_ReadOnly,  //!< Applies to: CoaXPress
        DeviceAccessStatus_NoAccess,  //!< Applies to: CoaXPress
        DeviceAccessStatus_Busy,  //!< Applies to: CoaXPress
        DeviceAccessStatus_OpenReadWrite,  //!< Applies to: CoaXPress
        DeviceAccessStatus_OpenReadOnly  //!< Applies to: CoaXPress
    };

    //! Valid values for DeviceType
    enum DeviceTypeEnums
    {
        DeviceType_Mixed,  //!< Applies to: CoaXPress
        DeviceType_Custom,  //!< Applies to: CoaXPress
        DeviceType_GEV,  //!< Applies to: CoaXPress
        DeviceType_CL,  //!< Applies to: CoaXPress
        DeviceType_IIDC,  //!< Applies to: CoaXPress
        DeviceType_UVC,  //!< Applies to: CoaXPress
        DeviceType_CXP,  //!< Applies to: CoaXPress
        DeviceType_CLHS,  //!< Applies to: CoaXPress
        DeviceType_U3V,  //!< Applies to: CoaXPress
        DeviceType_Ethernet  //!< Applies to: CoaXPress
    };

    //! Valid values for Format
    enum FormatEnums
    {
        Format_Gray8bit,  //!< Applies to: CoaXPress
        Format_Gray10bit,  //!< Applies to: CoaXPress
        Format_Gray12bit,  //!< Applies to: CoaXPress
        Format_Gray14bit,  //!< Applies to: CoaXPress
        Format_Gray16bit  //!< Applies to: CoaXPress
    };

    //! Valid values for InterfaceApplet
    enum InterfaceAppletEnums
    {
        InterfaceApplet_Acq_SingleCXP12x1Area,  //!< Applies to: CoaXPress
        InterfaceApplet_FrameGrabberTest  //!< Applies to: CoaXPress
    };

    //! Valid values for MissingCameraFrameResponse
    enum MissingCameraFrameResponseEnums
    {
        MissingCameraFrameResponse_Yes,  //!< Applies to: CoaXPress
        MissingCameraFrameResponse_No  //!< Applies to: CoaXPress
    };

    //! Valid values for MissingCameraFrameResponseClear
    enum MissingCameraFrameResponseClearEnums
    {
        MissingCameraFrameResponseClear_Apply  //!< Applies to: CoaXPress
    };

    //! Valid values for Pixelformat
    enum PixelformatEnums
    {
        Pixelformat_Mono8,  //!< Applies to: CoaXPress
        Pixelformat_Mono10,  //!< Applies to: CoaXPress
        Pixelformat_Mono12,  //!< Applies to: CoaXPress
        Pixelformat_Mono14,  //!< Applies to: CoaXPress
        Pixelformat_Mono16  //!< Applies to: CoaXPress
    };

    //! Valid values for SoftwaretriggerIsBusy
    enum SoftwaretriggerIsBusyEnums
    {
        SoftwaretriggerIsBusy_BusyFlagIsSet,  //!< Applies to: CoaXPress
        SoftwaretriggerIsBusy_BusyFlagIsNotSet  //!< Applies to: CoaXPress
    };

    //! Valid values for SystemmonitorByteAlignment8b10bLocked
    enum SystemmonitorByteAlignment8b10bLockedEnums
    {
        SystemmonitorByteAlignment8b10bLocked_Yes,  //!< Applies to: CoaXPress
        SystemmonitorByteAlignment8b10bLocked_No  //!< Applies to: CoaXPress
    };

    //! Valid values for SystemmonitorExternalPower
    enum SystemmonitorExternalPowerEnums
    {
        SystemmonitorExternalPower_Good,  //!< Applies to: CoaXPress
        SystemmonitorExternalPower_NoPower  //!< Applies to: CoaXPress
    };

    //! Valid values for SystemmonitorPowerOverCxpState
    enum SystemmonitorPowerOverCxpStateEnums
    {
        SystemmonitorPowerOverCxpState_BootingNotInitalized,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_NoCableConnected,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_NoPowerOverCxp,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_PowerOverCxpOk,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_MinimumCurrent,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_MaximumCurrent,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_LowVoltage,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_OverVoltage,  //!< Applies to: CoaXPress
        SystemmonitorPowerOverCxpState_AdcChipError  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerExceededPeriodLimits
    enum TriggerExceededPeriodLimitsEnums
    {
        TriggerExceededPeriodLimits_Yes,  //!< Applies to: CoaXPress
        TriggerExceededPeriodLimits_No  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerExceededPeriodLimitsClear
    enum TriggerExceededPeriodLimitsClearEnums
    {
        TriggerExceededPeriodLimitsClear_Apply  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggercameraOutSelect
    enum TriggercameraOutSelectEnums
    {
        TriggercameraOutSelect_Vcc,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_Gnd,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_CamAPulseGenerator0,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_CamAPulseGenerator1,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_CamAPulseGenerator2,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_CamAPulseGenerator3,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotCamAPulseGenerator0,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotCamAPulseGenerator1,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotCamAPulseGenerator2,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotCamAPulseGenerator3,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_BypassFrontGpi0,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotBypassFrontGpi0,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_BypassFrontGpi1,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotBypassFrontGpi1,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_BypassFrontGpi2,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotBypassFrontGpi2,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_BypassFrontGpi3,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotBypassFrontGpi3,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_PulseGenerator0,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_PulseGenerator1,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_PulseGenerator2,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_PulseGenerator3,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotPulseGenerator0,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotPulseGenerator1,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotPulseGenerator2,  //!< Applies to: CoaXPress
        TriggercameraOutSelect_NotPulseGenerator3  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerinPolarity
    enum TriggerinPolarityEnums
    {
        TriggerinPolarity_LowActive,  //!< Applies to: CoaXPress
        TriggerinPolarity_HighActive  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerinSrc
    enum TriggerinSrcEnums
    {
        TriggerinSrc_FrontGpiTriggerSource0,  //!< Applies to: CoaXPress
        TriggerinSrc_FrontGpiTriggerSource1,  //!< Applies to: CoaXPress
        TriggerinSrc_FrontGpiTriggerSource2,  //!< Applies to: CoaXPress
        TriggerinSrc_FrontGpiTriggerSource3  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerinStatsMinmaxfrequencyClear
    enum TriggerinStatsMinmaxfrequencyClearEnums
    {
        TriggerinStatsMinmaxfrequencyClear_Apply  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerinStatsPolarity
    enum TriggerinStatsPolarityEnums
    {
        TriggerinStatsPolarity_LowActive,  //!< Applies to: CoaXPress
        TriggerinStatsPolarity_HighActive  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerinStatsPulsecountClear
    enum TriggerinStatsPulsecountClearEnums
    {
        TriggerinStatsPulsecountClear_Apply  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerinStatsSource
    enum TriggerinStatsSourceEnums
    {
        TriggerinStatsSource_FrontGpiTriggerSource0,  //!< Applies to: CoaXPress
        TriggerinStatsSource_FrontGpiTriggerSource1,  //!< Applies to: CoaXPress
        TriggerinStatsSource_FrontGpiTriggerSource2,  //!< Applies to: CoaXPress
        TriggerinStatsSource_FrontGpiTriggerSource3  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggeroutSelectFrontGpo0
    enum TriggeroutSelectFrontGpo0Enums
    {
        TriggeroutSelectFrontGpo0_Vcc,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_Gnd,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_CamAPulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_CamAPulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_CamAPulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_CamAPulseGenerator3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotCamAPulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotCamAPulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotCamAPulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotCamAPulseGenerator3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_BypassFrontGpi0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotBypassFrontGpi0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_BypassFrontGpi1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotBypassFrontGpi1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_BypassFrontGpi2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotBypassFrontGpi2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_BypassFrontGpi3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotBypassFrontGpi3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_PulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_PulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_PulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_PulseGenerator3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotPulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotPulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotPulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo0_NotPulseGenerator3  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggeroutSelectFrontGpo1
    enum TriggeroutSelectFrontGpo1Enums
    {
        TriggeroutSelectFrontGpo1_Vcc,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_Gnd,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_CamAPulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_CamAPulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_CamAPulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_CamAPulseGenerator3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotCamAPulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotCamAPulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotCamAPulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotCamAPulseGenerator3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_BypassFrontGpi0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotBypassFrontGpi0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_BypassFrontGpi1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotBypassFrontGpi1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_BypassFrontGpi2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotBypassFrontGpi2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_BypassFrontGpi3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotBypassFrontGpi3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_PulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_PulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_PulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_PulseGenerator3,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotPulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotPulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotPulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutSelectFrontGpo1_NotPulseGenerator3  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggeroutStatsPulsecountClear
    enum TriggeroutStatsPulsecountClearEnums
    {
        TriggeroutStatsPulsecountClear_Apply  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggeroutStatsSource
    enum TriggeroutStatsSourceEnums
    {
        TriggeroutStatsSource_PulseGenerator0,  //!< Applies to: CoaXPress
        TriggeroutStatsSource_PulseGenerator1,  //!< Applies to: CoaXPress
        TriggeroutStatsSource_PulseGenerator2,  //!< Applies to: CoaXPress
        TriggeroutStatsSource_PulseGenerator3  //!< Applies to: CoaXPress
    };

    //! Valid values for TriggerqueueMode
    enum TriggerqueueModeEnums
    {
        TriggerqueueMode_On,  //!< Applies to: CoaXPress
        TriggerqueueMode_Off  //!< Applies to: CoaXPress
    };

    //! Valid values for Triggerstate
    enum TriggerstateEnums
    {
        Triggerstate_Active,  //!< Applies to: CoaXPress
        Triggerstate_AsyncStop,  //!< Applies to: CoaXPress
        Triggerstate_SyncStop  //!< Applies to: CoaXPress
    };



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! A parameter class containing all parameters as members that are available for Usb, GigE, BCON, and CoaXPress
    class PYLONBASE_API CUniversalTLParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CUniversalTLParams_Params(void);

            //! Destructor
            ~CUniversalTLParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CUniversalTLParams_ParamsData;
        CUniversalTLParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the timestamp, the hardware applet file has been generated - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& AppletBuildTime;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the applet id - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& AppletId;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the applet revision - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& AppletRevision;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the applet version - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& AppletVersion;
    
    //@}


    //! \name Synchronisation - 
    //@{
    /*!
        \brief Trigger mode for the area trigger signal generation - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<AreatriggermodeEnums>& Areatriggermode;
    
    //@}


    //! \name OutputFormat - 
    //@{
    /*!
        \brief Defines if the output data are right aligned or left aligned - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<BitalignmentEnums>& Bitalignment;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Active parts for frequency generation - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorActive;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Use internal frame generator or Camera input  Only 8 bit as pixel format is supported correctly - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CamerasimulatorEnableEnums>& CamerasimulatorEnable;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Line Gap between frames of camera simulator - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorFrameGap;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Camera simulator framerate - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& CamerasimulatorFramerate;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Image height of camera simulator - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorHeight;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Gap between lines of camera simulator  Value is automatically rounded up to internal processing granularity  Reading the parameter will return the rounded value - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorLineGap;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Defines the line rate of the camera simulator - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& CamerasimulatorLinerate;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Passive part for frequency generation - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorPassive;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Camera simulator pattern selector - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CamerasimulatorPatternEnums>& CamerasimulatorPattern;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Adds the defined value as an offset to the generated patterns - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorPatternOffset;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Defines the pixel frequency of the camera simulator - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& CamerasimulatorPixelFrequency;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Select rolling or static pattern - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CamerasimulatorRollEnums>& CamerasimulatorRoll;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Select input parameter for speed control - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CamerasimulatorSelectModeEnums>& CamerasimulatorSelectMode;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Select the trigger module to trigger the camera simulator - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CamerasimulatorTriggerModeEnums>& CamerasimulatorTriggerMode;
    
    //@}


    //! \name CameraSimulator - 
    //@{
    /*!
        \brief Image width of camera simulator  Width is automatically rounded up to internal processing granularity  Reading the parameter will return the rounded value - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamerasimulatorWidth;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the status of a connected CameraLink camera  For CoaXPress this is not of interest - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Camstatus;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the extended status of the camera  Bit = meaning: 0 = CameraPixelClk, 1 = CameraLval, 2 = CameraFval, 3 = Camera CC1 Signal, 4 = ExTrg / external trigger, 5 = BufferOverflow, 6 = BufferStatus-LSB, 7 = BufferStatus-MSB - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CamstatusExtended;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Enables sending all commands and receiving all acknowledges twice - Applies to: GigE

        Enables sending all commands and receiving all acknowledges twice. This option should only be enabled in case of network problems.
    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& CommandDuplicationEnable;
    
    //@}


    //! \name OutputFormat - 
    //@{
    /*!
        \brief Define a custom right bit shift  Set FG_BITALIGNEMENT to custom - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CustomBitShiftRight;
    
    //@}


    //! \name Debug - 
    //@{
    /*!
        \brief IInternal Use Only - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CxpDebugPortEnums>& CxpDebugPort;
    
    //@}


    //! \name Coaxpress - 
    //@{
    /*!
        \brief Reflect the current status of the camera - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& CxpStatus;
    
    //@}


    //! \name Coaxpress - 
    //@{
    /*!
        \brief Allows to send trigger rising edges only  Used for extreme trigger rates  This parameter can violate the CXP standard - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<CxpTriggerPacketModeEnums>& CxpTriggerPacketMode;
    
    //@}


    //! \name DebugInput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& Debugfile;
    
    //@}


    //! \name DebugInput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DebuginenableEnums>& Debuginenable;
    
    //@}


    //! \name DebugInput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DebuginsertEnums>& Debuginsert;
    
    //@}


    //! \name DebugOutput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DebugoutenableEnums>& Debugoutenable;
    
    //@}


    //! \name DebugOutput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Debugoutpixel;
    
    //@}


    //! \name DebugOutput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Debugoutxpos;
    
    //@}


    //! \name DebugOutput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Debugoutypos;
    
    //@}


    //! \name DebugInput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DebugreadyEnums>& Debugready;
    
    //@}


    //! \name Debug - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& Debugsaveconfig;
    
    //@}


    //! \name Debug - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Debugsource;
    
    //@}


    //! \name Debug - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& Debugsourcename;
    
    //@}


    //! \name DebugInput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DebugwriteflagEnums>& Debugwriteflag;
    
    //@}


    //! \name DebugInput - 
    //@{
    /*!
        \brief Debug-Interface, Internal Use ONLY - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Debugwritepixel;
    
    //@}


    //! \name DeviceInformation - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DeviceAccessStatusEnums>& DeviceAccessStatus;
    
    //@}


    //! \name DeviceInformation - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& DeviceDisplayName;
    
    //@}


    //! \name DeviceInformation - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& DeviceID;
    
    //@}


    //! \name DeviceInformation - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& DeviceModelName;
    
    //@}


    //! \name DeviceInformation - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<DeviceTypeEnums>& DeviceType;
    
    //@}


    //! \name DeviceInformation - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& DeviceVendorName;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the status of the DMA transmission - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Dmastatus;
    
    //@}


    //! \name Bufferstatus - 
    //@{
    /*!
        \brief Fill level of the buffers - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Filllevel;
    
    //@}


    //! \name OutputFormat - 
    //@{
    /*!
        \brief Pixel format of the output image - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<FormatEnums>& Format;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Current input signal levels of all Front GPI inputs (Bitmask) - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& FrontGpi;
    
    //@}


    //! \name Gentl - 
    //@{
    /*!
        \brief Ignore the FG output format and manage it internally in the GenTL producer - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& GentlInfoIgnorefgformat;
    
    //@}


    //! \name Gentl - 
    //@{
    /*!
        \brief Version of the GenTL description Interface in the applet - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& GentlInfoVersion;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Current input signal levels of all GPI inputs (Bitmask) - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Gpi;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief The name of the Hardware-Applet (HAP) file on which this AcquisitionApplets is based - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& HapFile;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Heartbeat timeout value on the host side in milliseconds - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& HeartbeatTimeout;
    
    //@}


    //! \name Roi - 
    //@{
    /*!
        \brief Image height of the acquisition ROI  If VantagePoint is set to BottomLeft or BottomRight the Height + YOffset must be smaller than the SensorHeight - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Height;
    
    //@}


    //! \name InterfaceApplets - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<InterfaceAppletEnums>& InterfaceApplet;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Maximum number of retries for read operations after a read operation has timed out - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& MaxRetryCountRead;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Maximum number of retries for write operations after a write operation has timed out - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& MaxRetryCountWrite;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Enables mapping of certain SFNC 1 x node names to SFNC 2 x node names - Applies to: Usb and GigE

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& MigrationModeEnable;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief Frame grabber received no frame for trigger pulse - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<MissingCameraFrameResponseEnums>& MissingCameraFrameResponse;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief Clear Missing Camera Frame Response Register - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<MissingCameraFrameResponseClearEnums>& MissingCameraFrameResponseClear;
    
    //@}


    //! \name ImageFormatControl - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IBooleanEx& OutputPackedFormats;
    
    //@}


    //! \name Bufferstatus - 
    //@{
    /*!
        \brief Signals if the image buffer is in overflow state - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Overflow;
    
    //@}


    //! \name OutputFormat - 
    //@{
    /*!
        \brief Internal processing bit depth of the applet - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Pixeldepth;
    
    //@}


    //! \name Coaxpress - 
    //@{
    /*!
        \brief Camera pixel format according to Pixel Format Naming Convention (PFNC) - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<PixelformatEnums>& Pixelformat;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Read access timeout value in milliseconds - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& ReadTimeout;
    
    //@}


    //! \name TriggerInputSoftwareTrigger - 
    //@{
    /*!
        \brief Add pulses to trigger queue - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Sendsoftwaretrigger;
    
    //@}


    //! \name TriggerInputSoftwareTrigger - 
    //@{
    /*!
        \brief Check if the trigger system is still processing software trigger pulses - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<SoftwaretriggerIsBusyEnums>& SoftwaretriggerIsBusy;
    
    //@}


    //! \name TriggerInputSoftwareTrigger - 
    //@{
    /*!
        \brief Number of pulses in queue which have to be processed - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SoftwaretriggerQueueFilllevel;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Number of timeouts during read and write operations when waiting for a response from the device - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& StatisticReadWriteTimeoutCount;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief Last error status of a read or write operation - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Last_Error_Status;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief Last error status of a read or write operation - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& Statistic_Last_Error_Status_Text;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief Number of failed read operations - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Read_Operations_Failed_Count;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief Number of read pipe resets - Applies to: Usb

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Read_Pipe_Reset_Count;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief Number of failed write operations - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Write_Operations_Failed_Count;
    
    //@}


    //! \name Statistic - Statistical data.
    //@{
    /*!
        \brief Number of write pipe resets - Applies to: Usb

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Write_Pipe_Reset_Count;
    
    //@}


    //! \name StreamEnumeration - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& StreamDisplayName;
    
    //@}


    //! \name StreamEnumeration - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& StreamID;
    
    //@}


    //! \name StreamEnumeration - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamSelector;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Byte Alignment 8b10b locked - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<SystemmonitorByteAlignment8b10bLockedEnums>& SystemmonitorByteAlignment8b10bLocked;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the Channel Current - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorChannelCurrent;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorChannelCurrentSelector;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the Channel Voltage - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorChannelVoltage;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorChannelVoltageSelector;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the current PCIe link speed in Gibibyte (2^30 byte) - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorCurrentLinkSpeed;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Disparity 8b 10b errors - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorDisparity8b10bError;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Shows the external power state of the board - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<SystemmonitorExternalPowerEnums>& SystemmonitorExternalPower;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the high FPGA DNA - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorFpgaDnaHigh;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the low FPGA DNA - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorFpgaDnaLow;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the current FPGA die temperature - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorFpgaTemperature;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the current FPGA auxiliary Vcc - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorFpgaVccAux;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the current FPGA BRAM Vcc - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorFpgaVccBram;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the current FPGA internal Vcc - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorFpgaVccInt;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Not in table 8b 10b errors - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorNotInTable8b10bError;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the PCIe trained payload size - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorPcieTrainedPayloadSize;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the PCIe trained request size - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& SystemmonitorPcieTrainedRequestSize;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Returns the port bit rate - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& SystemmonitorPortBitRate;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Shows the current power over CXP state - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<SystemmonitorPowerOverCxpStateEnums>& SystemmonitorPowerOverCxpState;
    
    //@}


    //! \name Miscellaneous - 
    //@{
    /*!
        \brief Internal framegrabber image timeout  This is independend of the runtime timeout value - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Timeout;
    
    //@}


    //! \name Coaxpress - 
    //@{
    /*!
        \brief Counts how many trigger acknowledgement packets sent by the camera (in answer to the trigger edge packets sent before) have been received by the frame grabber - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerAcknowledgementCount;
    
    //@}


    //! \name Coaxpress - 
    //@{
    /*!
        \brief Counts how many trigger events have been sent to the camera - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerEventCount;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief The trigger input signal frequency exceeds the allowed output frequency - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerExceededPeriodLimitsEnums>& TriggerExceededPeriodLimits;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief Clear the exceeded period limits register - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerExceededPeriodLimitsClearEnums>& TriggerExceededPeriodLimitsClear;
    
    //@}


    //! \name Synchronisation - 
    //@{
    /*!
        \brief Base frequency (=frame rate) of the trigger output - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerFramespersecond;
    
    //@}


    //! \name Sequencer - 
    //@{
    /*!
        \brief Upscale i e  duplicate each external or software generated trigger pulse with a period specified by FG_TRIGGER_FRAMESPERSEC - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerMultiplyPulses;
    
    //@}


    //! \name PulseFormGenerator0 - 
    //@{
    /*!
        \brief Delay between the input and the output - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen0Delay;
    
    //@}


    //! \name PulseFormGenerator0 - 
    //@{
    /*!
        \brief Downscale factor of the pulse form generator input - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen0Downscale;
    
    //@}


    //! \name PulseFormGenerator0 - 
    //@{
    /*!
        \brief Select the pulse index for each downscale series - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen0DownscalePhase;
    
    //@}


    //! \name PulseFormGenerator0 - 
    //@{
    /*!
        \brief Signal Output Width - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen0Width;
    
    //@}


    //! \name PulseFormGenerator1 - 
    //@{
    /*!
        \brief Delay between the input and the output - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen1Delay;
    
    //@}


    //! \name PulseFormGenerator1 - 
    //@{
    /*!
        \brief Downscale factor of the pulse form generator input - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen1Downscale;
    
    //@}


    //! \name PulseFormGenerator1 - 
    //@{
    /*!
        \brief Select the pulse index for each downscale series - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen1DownscalePhase;
    
    //@}


    //! \name PulseFormGenerator1 - 
    //@{
    /*!
        \brief Signal Output Width - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen1Width;
    
    //@}


    //! \name PulseFormGenerator2 - 
    //@{
    /*!
        \brief Delay between the input and the output - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen2Delay;
    
    //@}


    //! \name PulseFormGenerator2 - 
    //@{
    /*!
        \brief Downscale factor of the pulse form generator input - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen2Downscale;
    
    //@}


    //! \name PulseFormGenerator2 - 
    //@{
    /*!
        \brief Select the pulse index for each downscale series - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen2DownscalePhase;
    
    //@}


    //! \name PulseFormGenerator2 - 
    //@{
    /*!
        \brief Signal Output Width - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen2Width;
    
    //@}


    //! \name PulseFormGenerator3 - 
    //@{
    /*!
        \brief Delay between the input and the output - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen3Delay;
    
    //@}


    //! \name PulseFormGenerator3 - 
    //@{
    /*!
        \brief Downscale factor of the pulse form generator input - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen3Downscale;
    
    //@}


    //! \name PulseFormGenerator3 - 
    //@{
    /*!
        \brief Select the pulse index for each downscale series - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerPulseformgen3DownscalePhase;
    
    //@}


    //! \name PulseFormGenerator3 - 
    //@{
    /*!
        \brief Signal Output Width - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerPulseformgen3Width;
    
    //@}


    //! \name Coaxpress - 
    //@{
    /*!
        \brief Indicates a distance of two trigger edges violating the minimum edge frequency - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerWaveViolation;
    
    //@}


    //! \name CameraOutSignalMapping - 
    //@{
    /*!
        \brief Select the output source - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggercameraOutSelectEnums>& TriggercameraOutSelect;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Set the debounce time for trigger input signals - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerinDebounce;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Downscale of the trigger input signals - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerinDownscale;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Select the pulse index for each downscale series - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerinDownscalePhase;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Polarity of the trigger input signal - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerinPolarityEnums>& TriggerinPolarity;
    
    //@}


    //! \name TriggerInputExternal - 
    //@{
    /*!
        \brief Select the trigger input source for external trigger mode - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerinSrcEnums>& TriggerinSrc;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief Current Frequency of the input pulses - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerinStatsFrequency;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief The detected maximum input frequency - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerinStatsMaxfrequency;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief The detected minimum input frequency - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IFloatEx& TriggerinStatsMinfrequency;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief Clear minimum and maximum frequency measurement - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerinStatsMinmaxfrequencyClearEnums>& TriggerinStatsMinmaxfrequencyClear;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief Polarity of the trigger input signal for statistics - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerinStatsPolarityEnums>& TriggerinStatsPolarity;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief Pulse count of input - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerinStatsPulsecount;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief Clear the input signal pulse counter - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerinStatsPulsecountClearEnums>& TriggerinStatsPulsecountClear;
    
    //@}


    //! \name TriggerInputStatistics - 
    //@{
    /*!
        \brief Select the trigger input source for statistics - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerinStatsSourceEnums>& TriggerinStatsSource;
    
    //@}


    //! \name DigitalOutput - 
    //@{
    /*!
        \brief Select the output source for this Front GPO - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggeroutSelectFrontGpo0Enums>& TriggeroutSelectFrontGpo0;
    
    //@}


    //! \name DigitalOutput - 
    //@{
    /*!
        \brief Select the output source for this Front GPO - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggeroutSelectFrontGpo1Enums>& TriggeroutSelectFrontGpo1;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief Pulse count - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggeroutStatsPulsecount;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief Clear pulse counter - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggeroutStatsPulsecountClearEnums>& TriggeroutStatsPulsecountClear;
    
    //@}


    //! \name DigitalOutputStatistics - 
    //@{
    /*!
        \brief Select the pulse form generator which has to be analyzed - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggeroutStatsSourceEnums>& TriggeroutStatsSource;
    
    //@}


    //! \name Queue - 
    //@{
    /*!
        \brief Filllevel of trigger queue - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& TriggerqueueFilllevel;
    
    //@}


    //! \name Queue - 
    //@{
    /*!
        \brief Enable or disable the trigger queue and output frequency limitation specified by FG_TRIGGER_FRAMESPERSEC - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerqueueModeEnums>& TriggerqueueMode;
    
    //@}


    //! \name Synchronisation - 
    //@{
    /*!
        \brief State of the trigger system - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TriggerstateEnums>& Triggerstate;
    
    //@}


    //! \name Roi - 
    //@{
    /*!
        \brief Image width of the acquisition ROI  If VantagePoint is set to TopRight or BottomRight the With + XOffset must be smaller than the SensorWidth - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Width;
    
    //@}


    //! \name Root - Transport layer-specific parameters.
    //@{
    /*!
        \brief Write access timeout in milliseconds - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IIntegerEx& WriteTimeout;
    
    //@}


    //! \name Roi - 
    //@{
    /*!
        \brief X-offset of the acquisition ROI  If VantagePoint is set to TopRight or BottomRight the With + XOffset must be smaller than the SensorWidth - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Xoffset;
    
    //@}


    //! \name Roi - 
    //@{
    /*!
        \brief Y-offset of the acquisition ROI If VantagePoint is set to BottomLeft or BottomRight the Height + YOffset must be smaller than the SensorHeight - Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Yoffset;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CUniversalTLParams_Params(CUniversalTLParams_Params&);

            //! not implemented assignment operator
            CUniversalTLParams_Params& operator=(CUniversalTLParams_Params&);

        //! \endcond
    };


} // namespace Basler_UniversalTLParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_UNIVERSALTLPARAMS_H
