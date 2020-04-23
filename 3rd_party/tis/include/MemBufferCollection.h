// MemBufferCollection.h: interface for the MemBuffer class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MEMBUFFERCOLLECTION_H__9DAE8989_42EC_4A63_8328_AD5829AD2D11__INCLUDED_)
#define AFX_MEMBUFFERCOLLECTION_H__9DAE8989_42EC_4A63_8328_AD5829AD2D11__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <string>

#include "udshl_defs.h"
#include "simplectypes.h"
#include "smart_ptr.h"

#include "Error.h"
#include "FrameType.h"

namespace _DSHOWLIB_NAMESPACE
{
	class MemBuffer;
	class MemBufferCollection;

	typedef smart_ptr<MemBufferCollection> tMemBufferCollectionPtr;

	struct MemBufferCollectionPImpl;

	/** this class implements a memory buffer handling for image buffers.
	 * it can collect multiple buffers of the same type, i. e. with the same size and colorformat.
	 * The class provides an init() method to initialize the internal buffers. Any Errors can be
	 * queried by getLastError(). MemBufferCollection can handle user allocated buffers or allocates
	 * memory for you.
	 **/
	class MemBufferCollection
	{
	public:
		_UDSHL_EXP_API ~MemBufferCollection();

		typedef smart_ptr<MemBuffer> tMemBufferPtr;
		typedef smart_ptr<BITMAPINFOHEADER> tBmInfHPtr;

		/** get frame Size
		 * @return size of each Frame in the buffer
		 **/
		_UDSHL_EXP_API const SIZE&					getFrameSize() const;

		/** get colorformat
		 * @return colorformat for that the buffer was initialized.
		 * If the buffer is uninitialized, the return value is undefined.
		 **/
		_UDSHL_EXP_API tColorformatEnum			getColorformat() const;

		/** get bit per pixel
		 * @return number of bits per pixel for the current colorformat
		 **/
		_UDSHL_EXP_API int							getBitsPerPixel() const;

		/** get buffer length
		 * @return length of the buffer in frames
		 **/
		_UDSHL_EXP_API DWORD						getBufferCount() const; // in frames

		/** get a MemBuffer with a certain index
		 * @param pos frame number (starting from 0)
		 * @return the queried MemBuffer
		 **/
		_UDSHL_EXP_API tMemBufferPtr				operator [] ( DWORD pos ) const;

		/** get a MemBuffer with a certain index
		 * @param pos frame number (starting from 0)
		 * @return the queried MemBuffer
		 **/
		_UDSHL_EXP_API tMemBufferPtr				getBuffer( DWORD pos ) const;

		/** get the size of one frame buffer
		 * @return the size of one frame buffer in bytes
		 **/
		_UDSHL_EXP_API DWORD						getBufferSize() const; // in bytes

		/** test whether the buffer is initialized. Deprecated, the collection is always valid.
		 * @return true, if it is initialized
		 **/
		UDSHL_DEPRECATE_FUNCTION_
		_UDSHL_EXP_API bool							isInit() const;

		/// @return the last occurred error
		_UDSHL_EXP_API Error						getLastError() const;


		/** fills given buffer with a certain pattern.
		 * @param buffno number of buffer to fill with pattern
		 * @return true on success
		 **/
		_UDSHL_EXP_API bool						fillWithPattern( DWORD buffno, tPatternEnum pattern );

		/** fills every buffer with a certain pattern.
		 * @return true on success
		 **/
		_UDSHL_EXP_API bool						fillWithPattern( tPatternEnum pattern );

		/** writes buffers to disk in a bmp-file for each buffer.
		 * <bold>Attention:</bold> existing file are overwritten.
		 * @param filename is a path and filename relative to the current directory. The name must
		 * contain one '*' character which will be replaced by the collection number of the
		 * buffers.
		 * @return true on success, else false
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "saveToFileBMP functions in a loop" )
		_UDSHL_EXP_API bool						save( const dstringa& filename ) const;
        UDSHL_DEPRECATE_FUNCTION_T_( "saveToFileBMP functions in a loop" )
		_UDSHL_EXP_API bool						save( const dstringw& filename ) const;

		/** return pointer to BitmapInfoHeader struct according to the buffers. 
		 * The size of the returned buffer is sizeof( BITMAPINFOHEADER ) + sizeof( RGBQUAD ) * tBmInfHPtr->biClrUsed
		 * @return smart_ptr< BITMAPINFOHEADER > with BitmapInfoHeader to this collection
		 **/
		_UDSHL_EXP_API tBmInfHPtr					getBitmapInfoHeader() const;


		// since 2.0
		/** get the size of the valid data in the buffer itself.
		 * User allocated buffer may be larger than the actual data which is defined through
		 * (getFrameSize().cx * getFrameSize().cy * getBitsPerPixel()) / 8.
		 */
		_UDSHL_EXP_API DWORD						getBufferDataSize() const; // in bytes

		/** internal method, do not use.
		 */
		_UDSHL_EXP_API GUID						getMediaSubtype() const;

		// since 3.0
		/* Returns the frame type of the collection. */
		_UDSHL_EXP_API const FrameTypeInfo&		getFrameType() const;

		/** Creates a MemBufferCollection for use with the FrameHandlerSink or the Grabber.
		 * @param frame_type	Must contain a fully specified frame type (no partial frame type allowed).
		 * @param count			The number of buffers the collection should contain. Must be > 0.
		 * @return 0 when one of the parameters is not valid, otherwise a MemBufferCollection instance.
		 */
		_UDSHL_EXP_API static tMemBufferCollectionPtr		create( const FrameTypeInfo& frame_type, DWORD count );

		/** Creates a MemBufferCollection for use with the FrameHandlerSink or the Grabber.
		 * @param frame_type	Must contain a fully specified frame type (no partial frame type allowed).
		 * @param count			The number of buffers the collection should contain. Must be > 0.
		 * @param buffers		The user specified image data pointers. These will be used by the MemBuffers for the
		 *							image data. The array must contain <count> pointers, which must not be 0.
		 * @return 0 when one of the parameters is not valid, otherwise a MemBufferCollection instance.
		 */
		_UDSHL_EXP_API static tMemBufferCollectionPtr		create( const FrameTypeInfo& frame_type, DWORD count, BYTE* buffers[] );

		/** Creates a new MemBufferCollection from  a colorformat and image dimensions.
		 * @param colorformat	The colorformat of the MemBuffers. Must be a valid colorformat.
		 * @param dim			The dimensions of the MemBuffers created. Must be > 0
		 * @param count			The number of buffers the collection should contain. Must be > 0.
		 * @return 0 when one of the parameters is not valid, otherwise a MemBufferCollection instance.
		 **/
		_UDSHL_EXP_API static tMemBufferCollectionPtr		create( tColorformatEnum colorformat, SIZE dim, DWORD count );

		/** Creates a new MemBufferCollection from a colorformat and image dimensions.
		 * The MemBuffers in the collection get the pointers you specified as image data pointers.
		 * @param colorformat	The colorformat of the MemBuffers. Must be a valid colorformat.
		 * @param dim			The dimensions of the MemBuffers created. Must be > 0
		 * @param count			The number of buffers the collection should contain. Must be > 0.
		 * @param buffers		The user specified image data pointers. These will be used by the MemBuffers for the
		 *							image data. The array must contain <count> pointers, which must not be 0.
		 * @return 0 when one of the parameters is not valid, otherwise a MemBufferCollection instance.
		 **/
		_UDSHL_EXP_API static tMemBufferCollectionPtr		create( tColorformatEnum colorformat, SIZE dim, DWORD count, BYTE* buffers[] );
	protected:
		MemBufferCollection( const FrameTypeInfo& type, DWORD buffercount, BYTE* buffers[] );
	private:
		/** Copying of MemBufferCollection objects is prohibited. */
		MemBufferCollection( const MemBufferCollection& op );
		void	operator=( const MemBufferCollection& op2 );

		MemBufferCollectionPImpl*	m_pP;
	};
};

#endif // !defined(AFX_MEMBUFFERCOLLECTION_H__9DAE8989_42EC_4A63_8328_AD5829AD2D11__INCLUDED_)
