
#pragma once

#include <vector>

#include "udshl_defs.h"
#include "simplectypes.h"
#include "dvector.h"


namespace _DSHOWLIB_NAMESPACE
{
    struct FrameTypeInfo;

	/** An object implementing the IFrame interface permits access to its image data and the
	 * frame type of the image.
	 *
	 * The pointer returned from getPtr may only be valid in the scope of the function in which get a reference
	 * to the IFrame passed in.
	 */
	class IFrame
	{
	public:
		/** Returns the frame type of this frame. */
		virtual const FrameTypeInfo&		getFrameType() const = 0;

		/** Get pointer to the frame data.
		 * @return pointer to the frame buffer. This may be 0.
		 **/
		virtual BYTE*						getPtr() const = 0;

		/** Returns the sample description. */
		virtual const tsMediaSampleDesc&	getSampleDesc() const = 0;

        /* Retrieves the extended sample info
         * You have to supply a copy target. When the buffer is too small, no data is copied.
         *
         * Internal function, use media_sample_info::fill to fill the according structure and then perform the necessary operations on that.
         *
         * @param data_available    contains the count of bytes that are available.
         * @return Returns false when no data is available or the buffer passed in is too small,
         *          true otherwise.
         */
        virtual bool                        getMediaSampleInfo_( void* buffer, int buffer_len, int& data_available ) const = 0;

        virtual unsigned int                getActualDataSize() const = 0;
	};

    struct MediaSampleInfo;

    struct MediaSampleInfoDataStore
    {
        dvector<uint8_t>       data;

        bool    update( const IFrame& frame );

        /*
         * Finds the chunk with the according id.
         * REturns either the pointer to the specific chunk or NULL
         */

        UDSHL_EXP_API_ const uint8_t*             findChunk( int& data_len, uint32_t chunk_id ) const;
        UDSHL_EXP_API_ const MediaSampleInfo*     findSampleInfo() const;
        /*
         * Enumerates the chunks present in data.
         * The passed in function object must have the signature of 
                bool (* func)( uint32_t chunk_id, const uint8_t* chun_data, uint32_t chunk_data_len )
         *
         */
        template<class TFunc>
        void    enumChunkItems( TFunc func ) const;

        template<class TStruct>
        const TStruct*  findChunk( uint32_t chunk_id ) const;

        template<class TStruct>
        const TStruct*  findChunk() const;
    };

#pragma pack(push,1)
    struct MediaSampleInfo
	{
        enum { static_chunk_id = 'info' };

        uint64_t    driver_frame_number;
        uint64_t    driver_frame_first_packet_time;     // in us
        uint64_t    driver_frame_last_packet_time;      // in us

        uint64_t	device_time_stamp;	                // this is the timestamp as supplied by the device, maybe 0 if not available
        uint64_t    device_frame_number;                // this is the frame number as supplied by the device, maybe 0 if not available.
                                                        // note that this is device dependent. eg. GigECam wraps to 1 after hitting 2 ^ 16

        uint32_t    packet_resends_this_frame;          // only applicable for devices with packet resend
        uint32_t    bytes_missing_this_frame;           // number of bytes missing this frame, most likely 0, otherwise this was delivered with DropIncompleteFrames disabled
                                                        // note that the bytes missing can be at the end or in the center depending on the protocol used

        uint32_t    frame_flags;                        // 
    };

	struct MediaSampleInfoMultiFrameSetFrameId
	{
		// This is the multi frame set frame id chunk sent by cameras with IMX174, IMX250, IMX252 sensors
        enum { static_chunk_id = 0x13370002 };

		uint32_t     reserved0	: 28;
		uint32_t     frame_id	: 2;
		uint32_t     frame_set	: 1;
		uint32_t     reserved1	: 1;
    };

#pragma pack(pop)

    template<class TFunc>
    inline void    MediaSampleInfoDataStore::enumChunkItems( TFunc func ) const
    {
        const uint8_t* ptr = &data[0];
        const uint8_t* ptr_end = &data[0] + data.size();

        while( (ptr + 8) < ptr_end )
        {
            uint32_t chunk_data_len = *reinterpret_cast<const uint32_t*>(ptr + 0);
            uint32_t chunk_id = *reinterpret_cast<const uint32_t*>(ptr + 4);
            ptr += 8;

            const uint8_t* chunk_data = ptr;

            if( (ptr + chunk_data_len) > ptr_end )
                return;

            bool should_exit = func( chunk_id, chunk_data, chunk_data_len );
            if( should_exit )
                return;

            ptr += chunk_data_len;
        }
    }

    template<class TStruct>
    const TStruct*  MediaSampleInfoDataStore::findChunk( uint32_t chunk_id ) const
    {
        int blob_len = 0;
        const uint8_t* ptr = findChunk( blob_len, chunk_id );
        if( ptr == 0 )
            return 0;
        if( sizeof( TStruct ) > blob_len )
            return 0;

        return reinterpret_cast<const TStruct*>(ptr);
    }
    template<class TStruct>
    const TStruct*  MediaSampleInfoDataStore::findChunk() const
    {
        return findChunk<TStruct>( TStruct::static_chunk_id );
    }


    inline bool     MediaSampleInfoDataStore::update( const IFrame& frame )
    {
        if( data.size() < 1024 )      // optimization, don't shrink size if enough space would be available
        {
            data.clear();             // clear so that the resize does not need to copy
            data.resize( 1024 );      // allocate 1024 for the first try
        }

        int data_available = 0;
        bool rval = frame.getMediaSampleInfo_( &data[0], (int)data.size(), data_available );
        if( data_available == 0 )   // there is no data available
        {
            data.resize( 0 );
            return false;
        }
        if( !rval )
        {
            data.resize( data_available );        // resize to accommodate more data

            data_available = 0;
            rval = frame.getMediaSampleInfo_( &data[0], (int)data.size(), data_available );
            if( !rval )     // some other error
                data_available = 0;
        }

        data.resize( data_available );
        return rval;
    }

}

