// AnalogChannelItem.h: interface for the AnalogChannelItem class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ANALOGCHANNELITEM_H__31CEA1C4_246E_411C_800F_CF94EE0DBE8D__INCLUDED_)
#define AFX_ANALOGCHANNELITEM_H__31CEA1C4_246E_411C_800F_CF94EE0DBE8D__INCLUDED_

#pragma once

#include <string>
#include "udshl_defs.h"

#include "simplectypes.h"
#include "dstring.h"

namespace _DSHOWLIB_NAMESPACE
{
	class Grabber;

	/** Represents an input of a video capture device. **/
	class AnalogChannelItem
	{
		friend Grabber;
	public:
		///< constructs an invalid AnalogChannelItem()
		_UDSHL_EXP_API AnalogChannelItem();
        /** copies an analog channel item.
         * @param op item to copy
         **/
        _UDSHL_EXP_API AnalogChannelItem( const AnalogChannelItem& op );

		/** dtor */
		_UDSHL_EXP_API ~AnalogChannelItem();

		/** assignment operator
		 * @param op2
		 * @return this
		 */
		_UDSHL_EXP_API AnalogChannelItem&	operator=( const AnalogChannelItem& op2 );

		/** return internal PhysicalConnectorType
		 * @return internal PhysicalConnectorType, which is valid only if isValid() returns true
		 **/
		_UDSHL_EXP_API PhysicalConnectorType getPhysicalConnectorType() const { return m_PhysicalConnectorType; };

		/** return index of input
		 * @return index of input or -1, which is valid only if isValid() returns true
		 **/
		_UDSHL_EXP_API int getIndex() const { return m_Index; };

		/** test if the input is valid
		 * @return true, if the input is valid, else false
		 **/
		_UDSHL_EXP_API bool isValid() const;

		/** get a string representation of the input
		 * @return a pointer to the string representation of the input
		 **/
        UDSHL_DEPRECATE_FUNCTION_T_( "AnalogChannelItem::toString() method" )
		_UDSHL_EXP_API const char*		c_str() const;

		/** returns the result of a lexicographical compare of the index
		 * @return true, if this is before op
		 * @param op the VideoNormItem to compare with
		 **/
		_UDSHL_EXP_API bool			operator<(const AnalogChannelItem& op) const;

		/** test if two channels are equal
		 * @param op channel to compare to this
		 * @return true if this and op are equal, else false
		 **/
		_UDSHL_EXP_API bool			operator==( const AnalogChannelItem& op ) const;

		/** test if two channels are not equal
		 * @param op channel to compare to this
		 * @return false if this and op are equal, else true
		 **/
		_UDSHL_EXP_API bool			operator!=( const AnalogChannelItem& op ) const;

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
		static AnalogChannelItem createInvalid();

		/** Creates a textual representation for this AnalogChannelItem
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
	private:
		_UDSHL_EXP_API	dstringw	toString_() const;

		/** constructs an AnalogChannelItem */
		AnalogChannelItem( int index, PhysicalConnectorType type );

		/** the PhysicalConnectorType of the wrapped type */
		PhysicalConnectorType m_PhysicalConnectorType;
		/** the index of this Channel */
		int				m_Index;

		// this is only needed because we handout a string buffer in c_str()
		dstringa		m_String;
	};

};
#endif // !defined(AFX_ANALOGCHANNELITEM_H__31CEA1C4_246E_411C_800F_CF94EE0DBE8D__INCLUDED_)
