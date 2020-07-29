// Error.h: interface for the Error class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ERROR_H__6EB4029E_C2A2_4353_8938_FF398FE42826__INCLUDED_)
#define AFX_ERROR_H__6EB4029E_C2A2_4353_8938_FF398FE42826__INCLUDED_

#pragma once

#include <string>
#include "udshl_defs.h"
#include "simplectypes.h"

#include "int_interface_pre.h"
#include "dstring.h"

namespace _DSHOWLIB_NAMESPACE
{
	/** The Error class gives information about an error occurred during a call to the class library.
	 *
	 * Example usage :
	 * <br><code>
		Grabber g;
		// ...
		if( !g.startLive() )
		{
			Error e = g.getLastError();
			if( e.getVal() == eDEVICE_INVALID )
			{
				MessageBox( "Device has become invalid.", ... );
				// reopen device
			}
			else
			{
				MessageBox( e.toString(), ... );
				exit( -1 );
			}
		}
	   </code>
	 **/
	class Error 
	{
	public:
		/** Constructs an unknown Error **/
		_UDSHL_EXP_API Error();

		/** Constructs an unknown error with given message
		 * @param errordesc description of the error
		 **/
		_UDSHL_EXP_API Error( const dstringa& errordesc );
		_UDSHL_EXP_API Error( const dstringw& errordesc );

		/** constructs an error with given error code and uses the string from the resources
		 * @param e error code
		 **/
		_UDSHL_EXP_API Error( tErrorEnum e );

        /** constructs an error with given exception from DShowLib
         * @param e DShowLibException to handle
         **/
        _UDSHL_EXP_API Error( const icbase::IDShowError& e );

		/** constructs an error as a copy of the given one
		 * @param e Error to copy
		 **/
		_UDSHL_EXP_API Error( const Error& e );

		/** dtor */
		_UDSHL_EXP_API ~Error();

		/** return a string representation for the error
		 * @return a string representation for the error
		 **/
		std::string		getString() const
		{
			return toString();
		}
		std::wstring	getStringW() const
		{
			return toStringW();
		}

		std::string		toString() const
		{
			return m_String;
		}
		std::wstring	toStringW() const
		{
			return astows( m_String );
		}

		/** return a const char* to a string representing the error
		 * @return a const char* to a string representation of the error
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "Error::toString() method" )
		const char*	c_str() const
		{
			return m_String.c_str();
		}

		/** return the error code of the error
		 * @return the error code of the error
		 **/
		tErrorEnum	getVal() const
		{
			return m_Enum;
		}

		/** return a string representation for the error
		 * @return a string representation for the error
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "Error::toString() method" )
		operator std::string  () const
		{
			return m_String;
		}

		/** return a const char* to a string representing the error
		 * @return a const char* to a string representation of the error
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "Error::toString() method" )
		operator const char*  () const
		{
			return m_String.c_str();
		}

		/** return the error code of the error
		 * @return the error code of the error
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "Error::getVal() method" )
		operator tErrorEnum  () const
		{
			return m_Enum;
		}

		/** return true, if the internal error value equals not eNOERROR
		 * @return true, if the internal error value equals not eNOERROR
		 **/
		bool	isError() const
		{
			return eNOERROR != m_Enum;
		}
    private: // members
		tErrorEnum	m_Enum;
		dstringa	m_String;
	};
}

#endif // !defined(AFX_ERROR_H__6EB4029E_C2A2_4353_8938_FF398FE42826__INCLUDED_)
