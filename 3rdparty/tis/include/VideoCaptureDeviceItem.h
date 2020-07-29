// VideoCaptureDeviceItem.h: interface for the VideoCaptureDeviceItem class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_VIDEOCAPTUREDEVICEITEM_H__7ADEF0BC_B575_4806_8ED2_BE6961FE68DE__INCLUDED_)
#define AFX_VIDEOCAPTUREDEVICEITEM_H__7ADEF0BC_B575_4806_8ED2_BE6961FE68DE__INCLUDED_

#pragma once

#include <string>
#include "udshl_defs.h"

#include "smart_com.h"
#include "int_interface_pre.h"
#include "dstring.h"

namespace DriverInfoReq
{
	struct DeviceInfo;
}

namespace _DSHOWLIB_NAMESPACE
{
	class Grabber;
	class CSourceFilterType;
	class CVideoCaptureDeviceEnum;	

	/** En capsules the data to create a VideoCaptureDevice.
	 */
	class VideoCaptureDeviceItem  
	{
		friend Grabber;
		friend CVideoCaptureDeviceEnum;
	public:
		/** creates an invalid VideoCaptureDeviceItem. */
		_UDSHL_EXP_API VideoCaptureDeviceItem();

		/** copy constructor
		 * @param op2 VideoCaptureDeviceItem to copy info new one
		 **/
		_UDSHL_EXP_API VideoCaptureDeviceItem( const VideoCaptureDeviceItem& op2 );
		/** dtor */
		_UDSHL_EXP_API ~VideoCaptureDeviceItem();

		/** assignment operator.
		 * @return this
		 * @param op2
		 */
		_UDSHL_EXP_API VideoCaptureDeviceItem& operator=( const VideoCaptureDeviceItem& op2 );

		/** compares two VideoCaptureDeviceItems
		 * @param op2 VideoCaptureDeviceItem to compare with this
		 * @return true, if the device ID and the device name are equal, else false
		 **/
		_UDSHL_EXP_API bool				operator==( const VideoCaptureDeviceItem& op2 ) const;

		/** compares this to a string. This does toString() == op2
		 * @param op2 string
		 * @return true/false
		 */
		bool				operator==( const std::string& op2 ) const
		{
			return *this == astows( op2 );
		}
		_UDSHL_EXP_API bool				operator==( const dstringw& op2 ) const;

		/** test if instance is a valid device
		 * @return true if <code>this</code> is valid
		 **/
		_UDSHL_EXP_API bool				isValid() const;

		/** returns the name of the capture device, as returned from its driver
		 * @return the name of the capture device
		 **/
		std::string		getName() const
		{
			return toString();
		}

		/** returns the serial number
		 * @return true on success, false otherwise
		 */
		_UDSHL_EXP_API bool				getSerialNumber( __int64& val ) const;

		/** returns a unique name for this device. This is only available devices which return
		 * a serial number.
		 * @return When the device has a SerialNumber then a unique name, otherwise
		 *				a empty string is returned (getUniqueName().empty() == true)
		 */
		std::string		getUniqueName() const
		{
			return wstoas( getUniqueName_() );
		}
		std::wstring		getUniqueNameW() const
		{
			return getUniqueName_();
		}


		/**	returns the unique name of the device in this system.
		 * USB devices keep the same name as long as you do not change the USB port at which this
		 * device resides.
		 * IEEE 1394 devices keep their names, but may have a different name on a different machine.
		 * @return The display name of the device.
		 */
		std::string		getDisplayName() const
		{
			return wstoas( getDisplayNameW() );
		}
		std::wstring	getDisplayNameW() const
		{
			return getDisplayName_();
		}

		/** returns the base name of the device without the identification number appended to the device name.
		 * @return The base name of the device.
		 */
		std::string		getBaseName() const
		{
			return wstoas( getBaseNameW() );
		}
		/** returns the base name of the device without the identification number appended to the device name.
		 * @return The base name of the device.
		 */
		std::wstring	getBaseNameW() const
		{
			return getBaseName_();
		}


		/** Creates a textual representation for this VideoCaptureDeviceItem
		 * @return The textual representation.
		 */
		std::string		toString() const
		{
			return wstoas( toStringW() );
		}
		/** Creates a textual representation for this VideoCaptureDeviceItem
		 * @return The textual representation.
		 */
		std::wstring	toStringW() const
		{
			return getFriendlyName_();
		}


		/** returns the name of the capture device, as returned from its driver
		 * @return the name of the capture device
		 **/
		UDSHL_DEPRECATE_FUNCTION_T_( "VideoCaptureDeviceItem::toString() method" )
		_UDSHL_EXP_API const char*		c_str() const;


		/** returns the result of a lexicographical compare of the name
		 * @return true, if this is before op
		 * @param op the VideoCaptureDeviceItem to compare with
		 **/
		_UDSHL_EXP_API bool				operator<(const VideoCaptureDeviceItem& op) const;

		/** generates an invalid device
		 * @return an invalid device
		 * @see isValid()
		 **/
		_UDSHL_EXP_API static	VideoCaptureDeviceItem	createInvalid();

		/** Compares the device's serial number with a given serial number
		 */
		_UDSHL_EXP_API bool				operator==( const __int64& serial ) const;

		/**	Returns a reference to an internal interface.
		 * With this function you can fetch an custom interface from the codec.
		 * To use this function your compiler must support the __uuidof operator and the interface must
		 * be assigned an iid with the __declspec( uuid( "iid" ) ) compiler option. When this option is
		 * not available then you should use the other function.
		 * \param pItf
		 * \return A reference to the interface requested or 0 if the interface is not supported.
		 */
		template<class TItf>
		smart_com<TItf>		getInternalInterface( smart_com<TItf>& pItf ) 
		{
			return getInternalInterface( __uuidof( TItf ), pItf );
		}

		/**	Returns a reference to an internal interface.
		 * \par usage
		 *
		 *	smart_com<ICodecInterface> pItf;
		 *	if( pCodec->getInternalInterface( pItf ) == 0 )
		 *	{
		 *		... // interface is not supported, so error handling
		 *	}
		 *	else
		 *	{
		 *		...	// use the interface
		 *	}
		 *
		 * \param pItf A smart_com to a interface reference.
		 * \param riid An interface ID.
		 * \return A reference to the interface requested or 0 if the interface is not supported.
		 */
		template<class TItf>
		smart_com<TItf>		getInternalInterface( REFIID riid, smart_com<TItf>& pItf ) 
		{
			pItf = 0;
			getInternalInterface_( riid, (void**) &pItf.get() );
			return pItf;
		}

        bool         execKsPropQuerySupported( REFGUID guidPropSet, DWORD dwPropID )
        {
            DWORD type_supported = 0;
            HRESULT hr = execKsPropQuerySupported( guidPropSet, dwPropID, &type_supported );
            return SUCCEEDED( hr ) && type_supported != 0;
        }
        HRESULT      execKsPropSet( REFGUID guidPropSet, DWORD dwPropID, LPVOID pPropData, DWORD cbPropData )
        {
            return execKsPropSet( guidPropSet, dwPropID, pPropData, cbPropData, pPropData, cbPropData );
        }
        HRESULT      execKsPropGet( REFGUID guidPropSet, DWORD dwPropID, LPVOID pPropData, DWORD cbPropData, DWORD& cbReturned )
        {
            return execKsPropGet( guidPropSet, dwPropID, pPropData, cbPropData, pPropData, cbPropData, &cbReturned );
        }

        _UDSHL_EXP_API HRESULT      execKsPropQuerySupported( REFGUID guidPropSet, DWORD dwPropID, DWORD *pTypeSupport );
        _UDSHL_EXP_API HRESULT      execKsPropSet( REFGUID guidPropSet, DWORD dwPropID, LPVOID pInstanceData, DWORD cbInstanceData, LPVOID pPropData, DWORD cbPropData );
        _UDSHL_EXP_API HRESULT      execKsPropGet( REFGUID guidPropSet, DWORD dwPropID, LPVOID pInstanceData, DWORD cbInstanceData, LPVOID pPropData, DWORD cbPropData, DWORD *pcbReturned );

        _UDSHL_EXP_API  HRESULT     getDriverVersion( int& major, int& minor, int& spnum, int& build );
        _UDSHL_EXP_API  HRESULT     getDeviceVersion( unsigned __int64& version );

		_UDSHL_EXP_API	HRESULT		getDeviceInfo( DriverInfoReq::DeviceInfo& info );

	private:
		_UDSHL_EXP_API dstringw		getUniqueName_() const;
		_UDSHL_EXP_API dstringw		getDisplayName_() const;
		_UDSHL_EXP_API dstringw		getBaseName_() const;
		_UDSHL_EXP_API dstringw		getFriendlyName_() const;


        void    assign_index( int num );
        int     get_index() const       { return index_; }

		/// internal ctor
		explicit VideoCaptureDeviceItem( icbase::IDShowFactoryObjectInfo* pInfo );

		_UDSHL_EXP_API HRESULT			getInternalInterface_( REFIID riid, void** ppv );

        HRESULT                         internal_bind() const;

		smart_com<CSourceFilterType>	getSourceFilterType() const;

		mutable smart_com<CSourceFilterType>	m_pSourceFilter;
		
		/// internal data
		smart_com<icbase::IDShowFactoryObjectInfo>		m_pInternalData;

		dstringw						friendly_name_;
		dstringa						friendly_name_a_;

        int                             index_;
	};
};

#endif // !defined(AFX_VIDEOCAPTUREDEVICEITEM_H__7ADEF0BC_B575_4806_8ED2_BE6961FE68DE__INCLUDED_)
