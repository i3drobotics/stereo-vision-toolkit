// VideoFormatItem.h: interface for the VideoFormatItem class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VIDEOFORMATITEM_H__AD23522D_3281_4115_8943_C22F95EE0957__INCLUDED_)
#define AFX_VIDEOFORMATITEM_H__AD23522D_3281_4115_8943_C22F95EE0957__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <string>
#include "udshl_defs.h"

#include "int_interface_pre.h"
#include "dstring.h"

namespace dshowlib_dll
{
	struct sTISScanningModeInfo;
}

namespace _DSHOWLIB_NAMESPACE
{
	class Grabber;
	class GrabberPImpl;
	class VideoFormatDesc;
    class CSourceFilterType;

	/** Represents a certain Video Format with its size and MEDIASUBTYPE/Colorformat.
	 * All members will deliver valid values only if isValid() returns true
	 **/
	class VideoFormatItem
	{
		friend Grabber;
		friend GrabberPImpl;
		friend VideoFormatDesc;
        friend CSourceFilterType;
    public:
		/** constructs an empty VideoFormatItem
		 * the constructed item is invalid (see isValid())
		 * @see isValid()
		 **/
		_UDSHL_EXP_API VideoFormatItem();
		/** copies a VideoFormatItem
		 * @param op the VideoFormatItem to copy
		 **/
		_UDSHL_EXP_API VideoFormatItem( const VideoFormatItem& op );
		/** dtor */
		_UDSHL_EXP_API ~VideoFormatItem();

		/** assignment operator
		 * @param op2 op2
		 */
		_UDSHL_EXP_API VideoFormatItem& operator=( const VideoFormatItem& op2 );

		/** test if instance is a valid format
		 * @return true if <code>this</code> is valid
		 **/
		_UDSHL_EXP_API bool			isValid() const;

		/** get size of format
		 * @return the size of a frame with this format in a SIZE struct
		 **/
		_UDSHL_EXP_API SIZE			getSize() const;

		/** get the bit count per pixel.
		 */
		_UDSHL_EXP_API DWORD		getBitsPerPixel() const;

		/** get color format
		 * @return the GUID of the color format of this format
		 **/
		_UDSHL_EXP_API GUID			getColorformat() const;
		_UDSHL_EXP_API GUID			getSubtype() const;

		/** get binning factor
		 *
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "VideoFormatItem::getBinningFactorHorizontal() and VideoFormatItem::getBinningFactorVertical() methods" )
		_UDSHL_EXP_API int			getBinningFactor() const;

		_UDSHL_EXP_API int			getBinningFactorHorizontal() const;
		_UDSHL_EXP_API int			getBinningFactorVertical() const;
		_UDSHL_EXP_API int			getSkippingFactorHorizontal() const;
		_UDSHL_EXP_API int			getSkippingFactorVertical() const;

		/**
		 * Internal use only
		 **/
		int							getBinningModeId() const { return m_binningModeId; }

		/** get string representing the Format
		 * @return a string representation describing the format
		 * @see operator std::string ()
		 **/
        UDSHL_DEPRECATE_FUNCTION_T_( "VideoFormatItem::toString() method" )
		_UDSHL_EXP_API const char*		c_str() const;
	
		/** Creates a textual representation for this VideoFormatItem
		 * @return The textual representation.
		 */
		std::string		toString() const
		{
			return wstoas( toString_() );
		}
		std::wstring	toStringW() const
		{
			return toString_();
		}

		/** get string representing the dimensions of this format
		 * @return a string representing the size of a frame with this format
		 **/
		std::string		getSizeString() const
		{
			return getSizeString_();
		}


		/** get string representing the color format of this format
		 * @return string representing the color format of this format
		 **/
		std::string		getColorformatString() const
		{
			return wstoas( getColorformatString_() );
		}
		std::wstring	getColorformatStringW() const
		{
			return getColorformatString_();
		}

		_UDSHL_EXP_API bool			operator<( const VideoFormatItem& op ) const;

		/** test if two formats are equal
		 * @param op format to compare to this
		 * @return true if this and op are equal, else false
		 **/
		_UDSHL_EXP_API bool			operator==( const VideoFormatItem& op ) const;

		/** test if two formats are not equal
		 * @param op format to compare to this
		 * @return false if this and op are equal, else true
		 **/
        _UDSHL_EXP_API bool            operator!=( const VideoFormatItem& op ) const;

		/** test if this is equal to the string passed in op
		 * @param op item to compare to this
		 * @return true if this and op are equal, else false
		 **/
        _UDSHL_EXP_API bool            operator==( const dstringa& op ) const;
		_UDSHL_EXP_API bool            operator==( const dstringw& op ) const;

		/** generates an invalid item
		 * @return an invalid item
		 * @see isValid()
		 **/
		_UDSHL_EXP_API static VideoFormatItem createInvalid();


		/** get internal VideoFormat. should not be used
		 * @deprecated this function should only be used in the library, but not outside, because you can't do anything with 
		 *		a VideoFormat outside
		 */
		_UDSHL_EXP_API win32_utils::CVideoFormat		getVideoFormat() const;
	private:
		_UDSHL_EXP_API dstringa		getSizeString_() const;
		_UDSHL_EXP_API dstringw		toString_() const;
		_UDSHL_EXP_API dstringw		getColorformatString_() const;

		win32_utils::CVideoFormat*			get()		{ return m_pInternalData; }
		const win32_utils::CVideoFormat*	get() const	{ return m_pInternalData; }

		/** constructs a VideoFormatItem from a VideoFormat
		 * @param op2
		 **/
		VideoFormatItem( const win32_utils::CVideoFormat& op2, const dshowlib_dll::sTISScanningModeInfo& smi );
		VideoFormatItem( const win32_utils::CVideoFormat& op2, int binId, int binH, int binV, int skipH, int skipV );

		win32_utils::CVideoFormat*	m_pInternalData;
		int							m_binningModeId;
		int							m_binningHorizontal;
		int							m_binningVertical;
		int							m_skippingHorizontal;
		int							m_skippingVertical;
		
		void			initVideoFormatStringMember();

		dstringa		m_VideoFormatString;		
	};

};

#endif // !defined(AFX_VIDEOFORMATITEM_H__AD23522D_3281_4115_8943_C22F95EE0957__INCLUDED_)
