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

#ifndef BASLER_PYLON_UNIVERSALSTREAMPARAMS_H
#define BASLER_PYLON_UNIVERSALSTREAMPARAMS_H

#pragma once

// common parameter types
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>

#ifdef _MSC_VER
#pragma warning( push )
#pragma warning( disable : 4250 ) // warning C4250: 'Pylon::CXYZParameter': inherits 'Pylon::CParameter::Pylon::CParameter::ZYX' via dominance
#endif

//! The namespace containing the device's control interface and related enumeration types
namespace Basler_UniversalStreamParams
{

    //**************************************************************************************************
    // Enumerations
    //**************************************************************************************************

    //! Valid values for AccessMode
    enum AccessModeEnums
    {
        AccessMode_NotInitialized,  //!< Applies to: GigE
        AccessMode_Monitor,  //!< Applies to: GigE
        AccessMode_Control,  //!< Applies to: GigE
        AccessMode_Exclusive  //!< Applies to: GigE
    };

    //! Valid values for PixelPerClockCycle
    enum PixelPerClockCycleEnums
    {
        PixelPerClockCycle_One,  //!< One pixel per clock cycle is transmitted. - Applies to: BCON
        PixelPerClockCycle_Two  //!< Two pixels per clock cycle are transmitted. - Applies to: BCON
    };

    //! Valid values for Status
    enum StatusEnums
    {
        Status_NotInitialized,  //!< The stream grabber is not initialized. - Applies to: GigE
        Status_Closed,  //!< The stream grabber is closed. - Applies to: GigE
        Status_Open,  //!< The stream grabber is open. - Applies to: GigE
        Status_Locked  //!< The stream grabber is locked. - Applies to: GigE
    };

    //! Valid values for StreamBufferHandlingMode
    enum StreamBufferHandlingModeEnums
    {
        StreamBufferHandlingMode_NewestOnly,  //!< Applies to: CoaXPress
        StreamBufferHandlingMode_OldestFirst,  //!< Applies to: CoaXPress
        StreamBufferHandlingMode_OldestFirstOverwrite  //!< Applies to: CoaXPress
    };

    //! Valid values for StreamType
    enum StreamTypeEnums
    {
        StreamType_CL,  //!< Applies to: CoaXPress
        StreamType_CLHS,  //!< Applies to: CoaXPress
        StreamType_CXP,  //!< Applies to: CoaXPress
        StreamType_Custom,  //!< Applies to: CoaXPress
        StreamType_Ethernet,  //!< Applies to: CoaXPress
        StreamType_GEV,  //!< Applies to: CoaXPress
        StreamType_IIDC,  //!< Applies to: CoaXPress
        StreamType_Mixed,  //!< Applies to: CoaXPress
        StreamType_U3V,  //!< Applies to: CoaXPress
        StreamType_UVC  //!< Applies to: CoaXPress
    };

    //! Valid values for TransmissionType
    enum TransmissionTypeEnums
    {
        TransmissionType_UseCameraConfig,  //!< The stream transmission configuration is read from the camera. - Applies to: GigE
        TransmissionType_Unicast,  //!< The stream data is sent to a single device in the local network. - Applies to: GigE
        TransmissionType_Multicast,  //!< The stream data is sent to selected devices in the local network.  - Applies to: GigE
        TransmissionType_LimitedBroadcast,  //!< The stream data is sent to all devices in the local area network (255.255.255.255). - Applies to: GigE
        TransmissionType_SubnetDirectedBroadcast  //!< The stream data is sent to all devices in the same subnet as the camera. - Applies to: GigE
    };

    //! Valid values for Type
    enum TypeEnums
    {
        Type_WindowsFilterDriver,  //!< The pylon GigE Vision Filter Driver is used. - Applies to: GigE
        Type_WindowsIntelPerformanceDriver,  //!< The pylon GigE Vision Performance Driver is used. - Applies to: GigE
        Type_SocketDriver,  //!< The socket driver is used. - Applies to: GigE
        Type_NoDriverAvailable  //!< No suitable driver is installed. - Applies to: GigE
    };



    //**************************************************************************************************
    // Parameter class
    //**************************************************************************************************
    

    //! A parameter class containing all parameters as members that are available for Usb, GigE, BCON, and CoaXPress
    class PYLONBASE_API CUniversalStreamParams_Params
    {
    //----------------------------------------------------------------------------------------------------------------
    // Implementation
    //----------------------------------------------------------------------------------------------------------------
    protected:
        // If you want to show the following methods in the help file
        // add the string HIDE_CLASS_METHODS to the ENABLED_SECTIONS tag in the doxygen file
        //! \cond HIDE_CLASS_METHODS
        
            //! Constructor
            CUniversalStreamParams_Params(void);

            //! Destructor
            ~CUniversalStreamParams_Params(void);

            //! Initializes the references
            void _Initialize(GENAPI_NAMESPACE::INodeMap*);

            //! Return the vendor of the camera
            const char* _GetVendorName(void);

            //! Returns the camera model name
            const char* _GetModelName(void);
        
        //! \endcond

    private:
        class CUniversalStreamParams_ParamsData;
        CUniversalStreamParams_ParamsData* m_pData;


    //----------------------------------------------------------------------------------------------------------------
    // References to features
    //----------------------------------------------------------------------------------------------------------------
    public:
        
    //! \name Category Debug
    //@{
    /*!
        \brief Indicates the mode of access the current application has to the device - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IEnumParameterT<AccessModeEnums>& AccessMode;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Enables negotiation of the packet size to find the largest possible packet size - Applies to: GigE

        
        
        Enables or disables probing of a working large packet size before grabbing.
        Using large packets reduces the overhead for transferring images but
        whether a large packet can be transported depends on the used network hardware
        and its configuration.
        
        
    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& AutoPacketSize;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Clock frequency on the host side in Hertz - Applies to: BCON

        Clock frequency on the host side in Hertz. This parameter is linked with the BconClockFrequency camera parameter. If the camera parameter changes, the host parameter changes accordingly, but not vice versa. The host parameter should not be set to a value different from the camera parameter.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& ClockFrequency;
    
    //@}


    //! \name Category IP Configuration
    //@{
    /*!
        \brief IP address to which the stream grabber sends all stream data - Applies to: GigE

    
        \b Visibility = Expert

        \b Selected by : TransmissionType

    */
    Pylon::IStringEx& DestinationAddr;
    
    //@}


    //! \name Category IP Configuration
    //@{
    /*!
        \brief Port to which the stream grabber sends all stream data - Applies to: GigE

        
        
            The camera will send all stream data to this port. If you manually enter a port
            number, it is highly recommended not to choose a port from the ephemeral port
            range of the operating system (https://en.wikipedia.org/wiki/Ephemeral_port)
            in order to avoid port collisions.

            <b>Port configuration:</b>
            <ol>
            <li>
                <b>Unicast</b><br>
                The port is determined automatically.
                Manually choosing a port number might be useful for certain firewall configurations.
            </li>
            <br>
            <li>
                <b>Broadcast & Multicast</b><br>
                For each device reachable by a specific network interface, a unique, unused port number
                must be assigned. Be aware that the suggested default value might be already in use.
                Choose an unused port or 0=autoselect in that case. The controlling application and all 
                monitor applications must use the same port number. There is no autoselect feature
                availbale for monitor applications, i.e., monitor applications must not use the 0 value.
                For monitor applications it is convenient to use the 'UseCameraConfig' value for the 
                'TransmissionType' feature instead. For more details see the 'TransmissionType' feature.
            </li>
            </ol>
        
        
    
        \b Visibility = Expert

        \b Selected by : TransmissionType

    */
    Pylon::IIntegerEx& DestinationPort;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Enables the packet resend mechanism - Applies to: GigE

        
        
        An image frame consists of n numbers of packets. Each packet has a header consisting of a 24-bit packet ID. 
        This packet ID increases with each packet sent, and makes it possible for the receiving end to know if a 
        particular packet has been lost during the transfer. If 'ResendPacketMechanism' is enabled, and the receiving
        end notices a lost packet, it will request the other side (e.g. the camera) to send the lost packet again.
        If enabled, the 'ResendPacketMechanism' can cause delays in the timing because the sending end will resend 
        the lost packet. If disabled, image data packet(s) can get lost which results in an incomplete received frame.
        You have to weigh the disadvantages and advantages for your special application to decide whether to enable 
        or disable this command.<br><br>
        
        Default setting: <i>Enabled</i>
        
        
    
        \b Visibility = Expert

    */
    Pylon::IBooleanEx& EnableResend;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Maximum time in milliseconds to receive all packets of a frame - Applies to: GigE

        
      
      An image frame consists of n numbers of packets. The 'FrameRetention' always starts from the
      point in time the receiving end notices that a packet has been received for a particular frame. 
      If the transmission of packets of a frame is not completed within the 'FrameRetention' time, 
      the corresponding frame is delivered with status 'Failed'.
      
      
    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& FrameRetention;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Height of the image ROI on the host side - Applies to: BCON

        Height of the image ROI on the host side. This parameter is linked with the Height camera parameter. If the camera parameter changes, the host parameter changes accordingly, but not vice versa. The host parameter should not be set to a value different from the camera parameter.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Height;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Maximum size (in bytes) of a buffer used for grabbing images - Applies to: Usb, GigE and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& MaxBufferSize;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Maximum number of buffers that can be used simultaneously for grabbing images - Applies to: Usb, GigE and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& MaxNumBuffer;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Maximum USB data transfer size in bytes - Applies to: Usb

        
        
        The default value is appropriate for most applications. 
        Reducing the value may cause a higher CPU load. USB host adapter drivers may require 
        to decrease the value in case the application fails to receive the image stream. The maximum value 
        for the Maximum Transfer Size depends on the operating system version and may be limited by the host adapter drivers.
        
        
    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& MaxTransferSize;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Maximum number of resend requests per missing packet - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& MaximumNumberResendRequests;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Maximum number of USB request blocks (URBs) to be enqueued simultaneously - Applies to: Usb

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& NumMaxQueuedUrbs;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Timeout period in milliseconds between two packets within one frame - Applies to: GigE

        
        
        An image frame consists of n numbers of packets. The packet timeout counting is (re)started
        each time a packet is received. If the timeout expires (e.g. no packet was received 
        during the last 'PacketTimeout' period), the 'Resend Packet Mechanism' is started. 
        For information, see the 'EnableResend' feature.
        
        
    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& PacketTimeout;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Size of the payload in bytes - Applies to: BCON and CoaXPress

        Size of the payload in bytes. This is the total number of bytes of the image data payload produced by the frame grabber. This value is computed by the frame grabber implementation. The pylon stream grabber needs to be open to be able to compute the payload size.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& PayloadSize;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Pixel format on the host side - Applies to: BCON

        Pixel format on the host side. This parameter is linked with the Pixel Format camera parameter. If the camera parameter changes, the host parameter changes accordingly, but not vice versa. The host parameter should not be set to a value different from the camera parameter.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& PixelFormatValue;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Sets the number of pixels per clock cycle on the host side - Applies to: BCON

        Sets the number of pixels per clock cycle on the host side. This parameter is linked with the Pixels per Clock Cycle camera parameter. If the camera parameter changes, the host parameter changes accordingly, but not vice versa. The host parameter should not be set to a value different from the camera parameter.
    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<PixelPerClockCycleEnums>& PixelPerClockCycle;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Thread priority of the receive thread - Applies to: GigE

        Thread priority of the receive thread. Only available if the socket driver is used. To assign the priority, the Receive Thread Priority Override parameter must be set to true.
    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& ReceiveThreadPriority;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Enables assigning a custom priority to the thread which receives incoming stream packets - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IBooleanEx& ReceiveThreadPriorityOverride;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Size (in frames) of the receive window in which the stream grabber looks for missing packets - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& ReceiveWindowSize;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Amount of packet resend requests to be batched, i e , sent together - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& ResendRequestBatching;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Time to wait (in milliseconds) between sending a resend request and considering the request as lost - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& ResendRequestResponseTimeout;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Threshold after which resend requests are initiated - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& ResendRequestThreshold;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Time to wait (in milliseconds) between detecting a missing packet and sending a resend request - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& ResendTimeout;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Socket buffer size in kilobytes - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& SocketBufferSize;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of frames lost because there were no buffers in the queue - Applies to: GigE

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Buffer_Underrun_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief GigE cameras: Number of buffers with at least one failed packet  A packet is considered failed if its status is not 'success'  Other cameras: Number of buffers that returned with an error status   - Applies to: Usb, GigE and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Failed_Buffer_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of failed packets - Applies to: GigE

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Failed_Packet_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Last grabbed block ID - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Last_Block_Id;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Status code of the last failed buffer - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Last_Failed_Buffer_Status;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Status code of the last failed buffer - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& Statistic_Last_Failed_Buffer_Status_Text;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of corrupt or lost frames between successfully grabbed images - Applies to: Usb and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Missed_Frame_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of packets requested by packet resend commands - Applies to: GigE

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Resend_Packet_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of emitted packet resend commands sent - Applies to: GigE

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Resend_Request_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of stream resynchronizations - Applies to: Usb

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Resynchronization_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief GigE cameras: Number of frames received  Other cameras: Number of buffers processed - Applies to: Usb, GigE and BCON

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Total_Buffer_Count;
    
    //@}


    //! \name Category Statistic
    //@{
    /*!
        \brief Number of packets received - Applies to: GigE

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Statistic_Total_Packet_Count;
    
    //@}


    //! \name Category Debug
    //@{
    /*!
        \brief Indicates the current status of the stream grabber - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IEnumParameterT<StatusEnums>& Status;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamAnnounceBufferMinimum;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamAnnouncedBufferCount;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamBufferAlignment;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<StreamBufferHandlingModeEnums>& StreamBufferHandlingMode;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamChunkCountMaximum;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamDeliveredFrameCount;
    
    //@}


    //! \name Category Stream Information
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IStringEx& StreamID;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamInputBufferCount;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Beginner

    */
    Pylon::IBooleanEx& StreamIsGrabbing;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamLostFrameCount;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamOutputBufferCount;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& StreamStartedFrameCount;
    
    //@}


    //! \name Category Stream Information
    //@{
    /*!
        \brief Applies to: CoaXPress

    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<StreamTypeEnums>& StreamType;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Priority of the thread that handles USB requests from the stream interface - Applies to: Usb

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& TransferLoopThreadPriority;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Timeout for payload and trailer transfers in milliseconds - Applies to: Usb

    
        \b Visibility = Invisible

    */
    Pylon::IIntegerEx& TransferTimeout;
    
    //@}


    //! \name Category IP Configuration
    //@{
    /*!
        \brief Sets how stream data is transferred within the network - Applies to: GigE

        
        
        <ul>
        <li>
            <b>Default (Unicast)</b><br>
            The camera sends stream data to a single controlling application. Other devices cannot
            receive the stream data.
        </li>
        <br>
        <li>
            <b>Broadcast</b><br>
            The camera sends the stream data to all devices on the network. The application which 
            starts/stops the acquisition is called the controlling application. Other applications
            can receive the stream data. These applications are called monitor applications, because 
            they open the camera in read-only mode. This implies that monitor applications cannot 
            change the camera configuration and they cannot start/stop the image acquisition.
            However, monitor applications can request resend requests for lost stream data packets.
            <br><br>
            Attention: Broadcasting the stream data packets uses a high amount of network bandwidth 
                       because the stream data packets are forwarded to all devices attached to the 
                       network, even if they are not interested in receiving stream data.
        </li>
        <br>
        <li>
            <b>Multicast</b><br>
            Multicasting is very similar to broadcasting. The main advantage of multicasting is that 
            multicasting saves network bandwidth, because the image data stream is only sent to those 
            devices that are interested in receiving the data. To achieve this, the camera sends image 
            data streams to members of a multicast group only. A multicast group is defined by an IP 
            address taken from the multicast address range (224.0.0.0 to 239.255.255.255). 
            <br><br>
            Every device that wants to receive a multicast data stream has to be a member of a multicast 
            group. A member of a specific multicast group only receives data destinated for this group.
            Data for other groups is not received. Usually network adapters and switches are able to filter
            the data efficently on hardware level (layer-2 packet filtering). 
            <br><br>
            When multicasting is enabled for pylon, pylon automatically takes care of joining and leaving 
            the multicast groups defined by the destination IP address. Keep in mind that some addresses
            from the multicast address range are reserved for general purposes. The address range from
            239.255.0.0 to 239.255.255.255 is assigned by RFC 2365 as a locally administered address space.
            Use one of these addresses if you are not sure.
            <br><br>
            On protocol level multicasting involves a so-called IGMP message (IGMP = Internet Group Management Protocol).
            To benefit from multicasting, managed network switches should be used. These managed network 
            switches support the IGMP protocol. Managed network switches supporting the IGMP protocol
            will forward multicast packets only if there is a connected device that has joined the
            corresponding multicast group. If the switch does not support the IGMP protocol, multicast
            is equivalent to broadcasting.
            <br><br>
            Recommendation:<br>
            <ul>
                <li>
                    Each camera should stream to a different multicast group.
                </li>
                <li>
                    Streaming to different multicast groups reduces the CPU load and saves network bandwidth 
                    if the network switches supports the IGMP protocol.
                </li>
            </ul>
        </li>
        <br>        
        <li>
            <b>Use camera configuration</b><br>
            This setting is only available if the application has opened the device in monitor mode. If 
            the controlling application has already configured the camera stream channel and has possibly 
            started the acquisition, the monitor application can read all required configuration data 
            from the camera. Additional configuration is not required. This setting can only be used if 
            the controlling application has established the stream channel (by opening a pylon stream 
            grabber object), and the monitor application is started afterwards.
            <br><br>
            Note, when using broadcast and multicast configurations: When there is more than one camera 
            device reachable by one network interface, make sure that for each camera a different port
            number must be assigned. For assigning port numbers, see the 'DestinationPort' feature.
        </li>
        </ul>
        
        
    
        \b Visibility = Expert

    */
    Pylon::IEnumParameterT<TransmissionTypeEnums>& TransmissionType;
    
    //@}


    //! \name Category Root
    //@{
    /*!
        \brief Sets the driver type to be used by the stream grabber - Applies to: GigE

    
        \b Visibility = Beginner

    */
    Pylon::IEnumParameterT<TypeEnums>& Type;
    
    //@}


    //! \name Category Debug
    //@{
    /*!
        \brief Indicates whether the socket driver is currently available - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& TypeIsSocketDriverAvailable;
    
    //@}


    //! \name Category Debug
    //@{
    /*!
        \brief Indicates whether the pylon GigE Vision Filter Driver is currently available - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& TypeIsWindowsFilterDriverAvailable;
    
    //@}


    //! \name Category Debug
    //@{
    /*!
        \brief Indicates whether the pylon GigE Vision Performance Driver is currently available - Applies to: GigE

    
        \b Visibility = Guru

    */
    Pylon::IIntegerEx& TypeIsWindowsIntelPerformanceDriverAvailable;
    
    //@}


    //! \name Categories Buffer Handling Control and GrabberSettings
    //@{
    /*!
        \brief Width of the image ROI on the host side - Applies to: BCON

        Width of the image ROI on the host side. This parameter is linked with the Width camera parameter. If the camera parameter changes, the host parameter changes accordingly, but not vice versa. The host parameter should not be set to a value different from the camera parameter.
    
        \b Visibility = Expert

    */
    Pylon::IIntegerEx& Width;
    
    //@}



    private:
        //! \cond HIDE_CLASS_METHODS

            //! not implemented copy constructor
            CUniversalStreamParams_Params(CUniversalStreamParams_Params&);

            //! not implemented assignment operator
            CUniversalStreamParams_Params& operator=(CUniversalStreamParams_Params&);

        //! \endcond
    };


} // namespace Basler_UniversalStreamParams

#ifdef _MSC_VER
#pragma warning( pop )
#endif

#endif // BASLER_PYLON_UNIVERSALSTREAMPARAMS_H
