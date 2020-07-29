// VideoNormItem.h: interface for the VideoNormItem class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VIDEONORMITEM_H__978E5F24_E299_4CA7_BFD2_F5700DC0444D__INCLUDED_)
#define AFX_VIDEONORMITEM_H__978E5F24_E299_4CA7_BFD2_F5700DC0444D__INCLUDED_

#pragma once

#include <string>
#include "udshl_defs.h"
#include "dstring.h"

namespace _DSHOWLIB_NAMESPACE
{
	class Grabber;

	/** Class representing a video norm.
	 * All members will deliver valid values when isValid() returns true
	 **/
	class VideoNormItem
	{
		friend Grabber;
    private:
		VideoNormItem( AnalogVideoStandard t );
	public:
        /// constructs an invalid VideoNormItem();
		_UDSHL_EXP_API VideoNormItem();
        /// copies an VideoNormItem();
		_UDSHL_EXP_API VideoNormItem( const VideoNormItem& op );
		_UDSHL_EXP_API ~VideoNormItem();

		/** get the saved video norm
		 * @return saved video norm
		 **/
		_UDSHL_EXP_API AnalogVideoStandard	getVideoNorm()	const { return m_VideoNorm; }


		/** get a string representation of the saved video norm
		 * @return a pointer to the string representation of the saved video norm
		 **/
		_UDSHL_EXP_API const char*		c_str() const;

		/** Creates a textual representation for this VideoNormItem
		 * @return The textual representation.
		 */
		std::string		toString() const
		{
			return c_str();
		}
		std::wstring	toStringW() const
		{
			return astows( toString() );
		}

		/** assignment operator
		 * @return *this
		 */
		_UDSHL_EXP_API VideoNormItem&	operator=( const VideoNormItem& op2 );

		/** returns the result of a lexicographical compare of the names
		 * @return true, if this is before op
		 * @param op the VideoNormItem to compare with
		 **/
		_UDSHL_EXP_API bool operator < (const VideoNormItem& op) const;

        /** test if <code>this</code> is valid
         * @return true if <code>this</code> is valid
         **/
        _UDSHL_EXP_API bool            isValid() const;

		/** test if two norms are equal
		 * @param op norm to compare to this
		 * @return true if this and op are equal, else false
		 **/
		_UDSHL_EXP_API bool			operator==( const VideoNormItem& op ) const;

		/** test if two norms are not equal
		 * @param op norm to compare to this
		 * @return false if this and op are equal, else true
		 **/
		_UDSHL_EXP_API bool			operator!=( const VideoNormItem& op ) const;

		/** test if two items are equal
		 * @param op item to compare to this
		 * @return true if this and op are equal, else false
		 **/
		_UDSHL_EXP_API bool			operator==( const dstringa& op ) const;
		_UDSHL_EXP_API bool			operator==( const dstringw& op ) const;

		/** generates an invalid item
		 * @return an invalid item
		 * @see isValid()
		 **/
		_UDSHL_EXP_API static VideoNormItem createInvalid();
	private:
		AnalogVideoStandard	m_VideoNorm;
	};
};

#endif // !defined(AFX_VIDEONORMITEM_H__978E5F24_E299_4CA7_BFD2_F5700DC0444D__INCLUDED_)
