//-----------------------------------------------------------------------------
//  Basler pylon SDK
//  Copyright (c) 2006-2019 Basler AG
//  http://www.baslerweb.com
//  Author:  Hartmut Nebelung
//-----------------------------------------------------------------------------
/*!
\file
\brief    Low Level API: Definition of classes that contain grabbed data.
*/
#pragma once

#ifndef RESULT_H__
#define RESULT_H__


#include <limits.h>
#include <Base/GCTypes.h>
#include <Base/GCException.h>

#include <pylon/Platform.h>
#ifdef _MSC_VER
#   pragma pack(push, PYLON_PACKING)
#endif /* _MSC_VER */

#include <pylon/Platform.h>
#include <pylon/StreamGrabber.h>
#include <pylon/PixelType.h>
#include <pylon/PayloadType.h>
#include <pylon/ResultImage.h>

// Microsoft Visual Studio defines SIZE_MAX (see limits.h)
// But VS 2003 does not define SIZE_MAX. In this case, we have to define SIZE_MAX
// Because of this, the following lines must be AFTER #include limits.h
#ifndef SIZE_MAX
#define SIZE_MAX UINT_MAX
#endif

namespace Pylon
{
    /*!
    \brief Low Level API: Adapts a reference of a grab result to pylon image.

    \attention The referenced grab result must not be destroyed and the result's buffer must not be queued for grabbing again during the lifetime of this object.
    \ingroup  Pylon_LowLevelApi
    */
    class GrabResult;
    typedef CGrabResultImageT<const GrabResult&> CGrabResultImageRef;

    // -------------------------------------------------------------------------
    // enum GrabStatus
    // -------------------------------------------------------------------------
    /*!
    \brief Low Level API: Possible grab status values
    \ingroup Pylon_LowLevelApi
    */
    enum EGrabStatus
    {
        _UndefinedGrabStatus=-1,
        Idle,       ///< Currently not used.
        Queued,     ///< Grab request is in the input queue.
        Grabbed,    ///< Grab request terminated successfully. Buffer is filled with data.
        Canceled,   ///< Grab request was canceled. Buffer doesn't contain valid data.
        Failed      ///< Grab request failed. Buffer doesn't contain valid data.
    };

    // pylon 2.x compatibility.
    typedef EGrabStatus GrabStatus;

    /// Retrieve the number of bits per pixel for a given pixel type
#define BIT_PER_PIXEL( pixelType ) ( ( (pixelType) >> 16 ) & 0xff )

    // -------------------------------------------------------------------------
    // class GrabResult
    // -------------------------------------------------------------------------
    /*!
    \brief Low Level API: A grab result that combines the used image buffer and status information.

    \ingroup  Pylon_LowLevelApi

    Note that depending on the used interface technology, the specific camera and
    the situation some of the attributes are not meaningful, e. g. timestamp in case
    of an canceled grab.
    */
    class GrabResult
    {
    public:
        /// Default constructor
        GrabResult()
            : m_pContext( NULL ), m_hBuffer( NULL ), m_pBuffer( NULL )
            , m_Status( _UndefinedGrabStatus )
            , m_FrameNr( 0 )// 0 as N/A value for uint
            , m_PayloadType( PayloadType_Undefined )
            , m_PixelType( PixelType_Undefined )
            , m_TimeStamp( 0 ), m_SizeX( -1 ), m_SizeY( -1 ), m_OffsetX( -1 )
            , m_OffsetY( -1 ), m_PaddingX( -1 ), m_PaddingY (  -1 )
            , m_PayloadSize( (uint64_t) -1 )
            , m_ErrorCode( 0 ), m_ErrorDescription( "" )
            , m_BlockID(GC_UINT64_MAX)
        {
        }
        ~GrabResult(){}
        /// True if status is grabbed.
        bool Succeeded() const
        {
            return m_Status == Grabbed;
        }
        /// Get the buffer handle.
        StreamBufferHandle Handle() const
        {
            return m_hBuffer;
        }
        /// Get the pointer to the buffer.
        void* Buffer() const
        {
            return const_cast<void*>(m_pBuffer);
        }
        /// Get the grab status.
        EGrabStatus Status() const
        {
            return m_Status;
        }
        /// Get the pointer the user provided context.
        const void* Context() const
        {
            return m_pContext;
        }

        /// Deprecated: GetBlockID() should be used instead. Get the index of the grabbed frame.
        PYLON_DEPRECATED("FrameNr() has been deprecated. Use GetBlockID() instead.")  uint32_t FrameNr() const
        {
            return m_FrameNr;
        }
        //
        // Get functions for reporting frame info
        //

        /// Get the actual payload type.
        EPayloadType GetPayloadType() const
        {
            return m_PayloadType;
        }
        /// Get the actual pixel type
        /*!
        This is only defined in case of image data.
        */
        EPixelType GetPixelType() const
        {
            return m_PixelType;
        }
        /// Get the camera specific tick count
        /*!
        In case of GigE-Vision this describes when the image exposure was started.
        Cameras that do not support this feature return zero. If supported this
        may be used to determine which ROIs were acquired simultaneously.

        In case of FireWire this value describes the cycle time when the first
        packet arrives.
        */
        uint64_t GetTimeStamp() const
        {
            return m_TimeStamp;
        }
        /// Get the actual number of columns in pixel
        /*! This is only defined in case of image data.*/
        int32_t GetSizeX() const
        {
            return m_SizeX;
        }
        /// Get the actual number of rows in pixel
        /*! This is only defined in case of image data.*/
        int32_t GetSizeY() const
        {
            return m_SizeY;
        }
        /// Get the actual starting column
        /*! This is only defined in case of image data.*/
        int32_t GetOffsetX() const
        {
            return m_OffsetX;
        }
        /// Get the actual starting row
        /*! This is only defined in case of image data.*/
        int32_t GetOffsetY() const
        {
            return m_OffsetY;
        }
        /// Get the number of extra data at the end of each row in bytes
        /*! This is only defined in case of image data.*/
        int32_t GetPaddingX() const
        {
            return m_PaddingX;
        }
        /// Get the number of extra data at the end of the image data in bytes
        /*! This is only defined in case of image data.*/
        int32_t GetPaddingY() const
        {
            return m_PaddingY;
        }
        /// Get the actual payload size in bytes.
        int64_t GetPayloadSize() const
        {
            return m_PayloadSize;
        }
        /// Get the actual payload size in bytes as size_t.
        size_t GetPayloadSize_t() const
        {
#if SIZE_MAX >= 0xffffffffffffffffULL
            return static_cast<size_t>(GetPayloadSize());
#else
            if (m_PayloadSize > SIZE_MAX)
            {
                throw OUT_OF_RANGE_EXCEPTION("PayloadSize too big");
            }
            return static_cast<size_t>(m_PayloadSize & SIZE_MAX);
#endif
        }

        /// Get a description of the current error.
        String_t GetErrorDescription() const
        {
            return m_ErrorDescription;
        }
        /// Get the current error code
        uint32_t GetErrorCode() const
        {
            return m_ErrorCode;
        }


        /*!
        \brief Provides an adapter from the grab result to Pylon::IImage interface.

        This returned adapter allows passing the grab result to saving functions or image format converter.

        \attention The returned reference is only valid as long the grab result is not destroyed.
        */
        CGrabResultImageRef GetImage() const
        {
            return CGrabResultImageRef(*this, false);
        }

        /// Get the block ID of the grabbed frame (camera device specific).
        /*!
        \par IEEE 1394 Camera Devices
        The value of block ID is always UINT64_MAX.

        \par GigE Camera Devices
        The sequence number starts with 1 and
        wraps at 65535. The value 0 has a special meaning and indicates
        that this feature is not supported by the camera.

        \par USB Camera Devices
        The sequence number starts with 0 and uses the full 64 Bit range.

        \attention A block ID of value UINT64_MAX indicates that the Block ID is invalid and must not be used.
        */
        uint64_t GetBlockID() const
        {
            return m_BlockID;
        }
    protected:
        const void *m_pContext;
        StreamBufferHandle m_hBuffer;
        const void *m_pBuffer;
        EGrabStatus m_Status;

        // private members for reporting frame info values
        uint32_t m_FrameNr;
        EPayloadType m_PayloadType;
        EPixelType m_PixelType;
        uint64_t m_TimeStamp;
        int32_t m_SizeX;
        int32_t m_SizeY;
        int32_t m_OffsetX;
        int32_t m_OffsetY;
        int32_t m_PaddingX;
        int32_t m_PaddingY;
        uint64_t m_PayloadSize;
        uint32_t m_ErrorCode;
        String_t m_ErrorDescription;
        uint64_t m_BlockID;
    };


    // -------------------------------------------------------------------------
    // class EventResult
    // -------------------------------------------------------------------------
    /*!
    \brief Low Level API: An event result
    \ingroup Pylon_LowLevelApi
    */
    class EventResult
    {
    protected:
        unsigned long m_ReturnCode;
        String_t m_Message;
    public:
        EventResult();
        ~EventResult();
        bool Succeeded() const;
        String_t ErrorDescription() const;
        unsigned long ErrorCode() const;
        unsigned char Buffer[576];
    };
    inline EventResult::EventResult()
        : m_ReturnCode(0)
    {};

    inline EventResult::~EventResult()
    {};

    inline bool EventResult::Succeeded() const
    {
        return 0 == m_ReturnCode;
    }
    inline String_t EventResult::ErrorDescription() const
    {
        return m_Message;
    }
    inline unsigned long EventResult::ErrorCode() const
    {
        return m_ReturnCode;
    }

    /*!
    \brief Low Level API: Adapts a copy of a grab result to pylon image.

    \attention The the result's buffer must not be queued for grabbing again during the lifetime of this object.

    \ingroup  Pylon_LowLevelApi
    */
    typedef CGrabResultImageT<GrabResult> CGrabResultImage;
}

#ifdef _MSC_VER
#   pragma pack(pop)
#endif /* _MSC_VER */

#endif // RESULT_H__
