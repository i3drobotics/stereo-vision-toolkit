// MemBuffer.h: interface for the MemBuffer class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MEMBUFFER_H__E562743E_3382_4481_8532_C2A2E936FF10__INCLUDED_)
#define AFX_MEMBUFFER_H__E562743E_3382_4481_8532_C2A2E936FF10__INCLUDED_

#pragma once

#include <string>
#include "udshl_defs.h"
#include "smart_ptr.h"

#include "simplectypes.h"
#include "FrameType.h"
#include "dstring.h"

struct IMediaSample;

namespace _DSHOWLIB_NAMESPACE
{
	struct MemBufferPImpl;
	struct MemBufferCollectionPImpl;
	class MemBufferCollection;
	class Error;
    class GrabberPImpl;

	/** This class saves a pointer to a MemBuffer and some necessary information for this buffer.
	 * The information saved alongside the buffer is the dimension of the frame, the color format and the
	 * size of the buffer. If this buffer is initialized without user mode data pointer, the memory
	 * is automatically allocated on initialization and freed on destruction.
	 *
	 * Remember that Windows images are usually bottom up.
	 **/
	class MemBuffer : public IFrame
	{
		friend MemBufferCollection;
		friend MemBufferCollectionPImpl;
	public:
		typedef smart_ptr< BITMAPINFOHEADER > tBmInfHPtr;

	public:
		/** creates a new MemBuffer by copying an existing one
		 * removed due to inconsistencies in the meaning of this operation
		 * @param op MemBuffer to copy
		 **/
//		MemBuffer( const MemBuffer& op );
		_UDSHL_EXP_API ~MemBuffer();

		/** get internal color format
		 * @return internal color format
		 **/
		_UDSHL_EXP_API tColorformatEnum	getColorformat() const;

		/** get internal frame size
		 * @return internal frame size
		 **/
		_UDSHL_EXP_API const SIZE&			getSize() const;

		/** get size of internal buffer
		 * @return size of internal buffer
		 **/
		_UDSHL_EXP_API DWORD				getBufferSize() const;

		/** get bits per pixel
		 * @return number of bits per pixel for the current color format
		 **/
		_UDSHL_EXP_API int					getBitsPerPixel() const;

		/** get pointer to internal buffer
		 * @return pointer to internal buffer
		 **/
		_UDSHL_EXP_API BYTE*				getPtr() const;

		/** get position in collection
		 * @return position in collection
		 **/
		_UDSHL_EXP_API DWORD				getCollectionPos() const;


		/** get collection of buffer
		 * @return the collection of the buffer
		 **/
		_UDSHL_EXP_API const MemBufferCollection& getCollection() const;

		/** writes buffer to disk in a bmp-file.
		 * <bold>Attention:</bold> existing file are overwritten.
		 * @param filename is a path and filename relative to the current directory.
		 * @return Error that contains the error if on occurred
		 **/
        UDSHL_DEPRECATE_FUNCTION_T_( "saveToFileBMP() function" )
		_UDSHL_EXP_API Error					save( const dstringa& filename ) const;
        UDSHL_DEPRECATE_FUNCTION_T_( "saveToFileBMP() function" )
		_UDSHL_EXP_API Error					save( const dstringw& filename ) const;
        UDSHL_DEPRECATE_FUNCTION_T_( "saveToFileBMP() function" )
		_UDSHL_EXP_API Error					save( const wchar_t* pFilename ) const;

		/** return pointer to BitmapInfoHeader struct according to the buffer
		 * The size of the returned buffer is sizeof( BITMAPINFOHEADER ) + sizeof( RGBQUAD ) * tBmInfHPtr->biClrUsed
		 * @return smart_ptr< BITMAPINFOHEADER > with BitmapInforHeader to this buffer
		 **/
        UDSHL_DEPRECATE_FUNCTION_T_( "createBitmapInfoHeader() function" )
		_UDSHL_EXP_API tBmInfHPtr				getBitmapInfoHeader() const;

		/** increases the lock count by one */
		_UDSHL_EXP_API void					lock();
		/** decreases the lock count by one */
		_UDSHL_EXP_API void					unlock();
		/** forces a complete unlock of the buffer */
		_UDSHL_EXP_API void					forceUnlock();
		/** returns if the buffer is locked */
		_UDSHL_EXP_API bool					isLocked() const;

		/** returns the WasLockedFlag.
		 * the WasLockedFlag is set by the grabber, when it cannot copy to this buffer, because the buffer is locked.
		 */
		_UDSHL_EXP_API bool					getWasLockedFlag() const;
		/** sets the WasLockedFlag */
		_UDSHL_EXP_API void					setWasLockedFlag( bool f );

		// since 1.4
		/** returns the sample description. */
		_UDSHL_EXP_API const tsMediaSampleDesc&	getSampleDesc() const;

		// since 3.0
		_UDSHL_EXP_API virtual const FrameTypeInfo&		getFrameType() const;

        // since 3.4
        virtual bool            getMediaSampleInfo_( void* buffer, int buffer_len, int& data_available  ) const;
        virtual unsigned int    getActualDataSize() const;

        /*
         *	Do not use, internal function.
         */
        void            set_sample_info( IMediaSample& sample );
        void            set_sample_info( IMediaSample& sample, GrabberPImpl& parent );
	private:
		/** Creates a MemBuffer from the collection parent
        * Note: May throw bad_alloc
		* @param parent         The collection to which this MemBuffer belongs
		* @param buffer         Optional. == nullptr => Allocate own buffer, != nullptr => a User allocated buffer
		* @param collectionpos  Position of MemBuffer in MemBufferCollection
		**/
		MemBuffer( const MemBufferCollection& parent, void* buffer, DWORD collectionpos, const FrameTypeInfo& type );
	private:
		MemBufferPImpl* m_pP;
	};

	/** This method saves the IFrame data into a bitmap file.
	 * The format of the bitmap file (BITMAPINFO) is specified by the IFrame. When the IFrame does have an
	 * unknown FrameType, then eINVALID_PARAM_VAL is returned. The IFrame data is not converted to another type,
	 * this means that the bitmap may contain a odd looking image (e.g. UYVY is treated as RGB565).
	 * @param buf		The IFrame to save.
	 * @param filename	The filename of the target file. This file must not exist.
	 * @return When the frame could not be saved, then eINVALID_PARAM_VAL is returned. Otherwise eNOERROR
	 */
	Error	_UDSHL_EXP_API saveToFileBMP( const IFrame& buf, const dstringa& filename );
	Error	_UDSHL_EXP_API saveToFileBMP( const IFrame& buf, const dstringw& filename );
	Error	_UDSHL_EXP_API saveToFileBMP( const IFrame& buf, const wchar_t* pFilename );
	/** This method saves the IFrame data into a bitmap file.
	 * The format of the bitmap file (BITMAPINFO) is specified by the file_fmt parameter.
	 * When you specify another color format as the IFrame FrameType, then the data is converted on the fly to the
	 * desired file format.
	 * @param buf		The IFrame to save.
	 * @param filename	The filename of the target file. This file must not exist.
	 * @param file_fmt	The color format of the target file.
	 * @param bTreatAs	If false, then the IFrame data should be converted to the file_fmt, otherwise
	 *						it should be treated as this frame type (this may lead to odd looking images or even
	 *						bitmaps which do have odd width/height).
	 * @return When the conversion failed or the frame could not be saved, then eINVALID_PARAM_VAL is returned. 
	 *				Otherwise eNOERROR.
	 */
	Error	_UDSHL_EXP_API saveToFileBMP( const IFrame& buf, const dstringa& filename, tColorformatEnum file_fmt, bool bTreatAs = false );
	Error	_UDSHL_EXP_API saveToFileBMP( const IFrame& buf, const dstringw& filename, tColorformatEnum file_fmt, bool bTreatAs = false );
	Error	_UDSHL_EXP_API saveToFileBMP( const IFrame& buf, const wchar_t* pFilename, tColorformatEnum file_fmt, bool bTreatAs = false );

	/** This method saves the IFrame data into a JPEG file.
	 * The IFrame data is internally converted into a format which the JPEG library then converts to a JPEG file.
	 * When the input IFrame data cannot be converted, then the method fails.
	 * @param buf		The IFrame to save.
	 * @param filename	The filename of the target file. This file must not exist.
	 * @param quality	The quality of the created JPEG file. Must be in the range [0,100], with 100 being the highest.
	 * @return When the conversion failed, then eINVALID_PARAM_VAL is returned. Otherwise eNOERROR.
	 */
	Error	_UDSHL_EXP_API saveToFileJPEG( const IFrame& buf, const dstringa& filename, int quality = 75 );
	Error	_UDSHL_EXP_API saveToFileJPEG( const IFrame& buf, const dstringw& filename, int quality = 75 );
	Error	_UDSHL_EXP_API saveToFileJPEG( const IFrame& buf, const wchar_t* pFilename, int quality = 75 );

	/** This method saves the IFrame data into a TIFF file.
	 * When the input IFrame data cannot be converted into a format that can be stored in a TIFF file,
	 * the method fails.
	 * @param buf      The IFrame to be saved
	 * @param fileName The name of the file to be written
	 * @return When the buffer cannot be saved as a TIFF file, then eINVALID_PARAM_VAL is returned, otherwise eNOERROR.
	 */
	Error	_UDSHL_EXP_API saveToFileTIFF( const IFrame& buf, const dstringa& fileName );
	Error	_UDSHL_EXP_API saveToFileTIFF( const IFrame& buf, const dstringw& fileName );
	Error	_UDSHL_EXP_API saveToFileTIFF( const IFrame& buf, const wchar_t* pFileName );
};
#endif // !defined(AFX_MEMBUFFER_H__E562743E_3382_4481_8532_C2A2E936FF10__INCLUDED_)
