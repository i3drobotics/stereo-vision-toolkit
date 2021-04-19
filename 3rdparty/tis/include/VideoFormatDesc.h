
#ifndef _VIDEOFORMATDESC_H_
#define _VIDEOFORMATDESC_H_

#include "dstring.h"

#include "VideoFormatItem.h"

#include "int_interface_pre.h"

namespace dshowlib_dll
{
	struct sTISScanningModeInfo;
}

namespace _DSHOWLIB_NAMESPACE
{
	class Grabber;
    class CSourceFilterType;

	class VideoFormatDesc
	{
		friend class Grabber;
        friend class CSourceFilterType;
	public:
		_UDSHL_EXP_API ~VideoFormatDesc();
	public:
		_UDSHL_EXP_API VideoFormatItem	createVideoFormat( SIZE size ) const;
		_UDSHL_EXP_API VideoFormatItem	createVideoFormat( int width, int height ) const;
	public:
		_UDSHL_EXP_API GUID			getSubtype() const;
		_UDSHL_EXP_API SIZE			getMinSize() const;
		_UDSHL_EXP_API SIZE			getMaxSize() const;
		_UDSHL_EXP_API SIZE			getStepSize() const;

		UDSHL_DEPRECATE_FUNCTION_T_( "VideoFormatDesc::getBinningFactorHorizontal() and VideoFormatDesc::getBinningFactorVertical() methods" )
		_UDSHL_EXP_API int			getBinningFactor() const;

		// Internal use only
		_UDSHL_EXP_API int			getBinningModeId() const;

		_UDSHL_EXP_API int			getBinningFactorHorizontal() const;
		_UDSHL_EXP_API int			getBinningFactorVertical() const;
		_UDSHL_EXP_API int			getSkippingFactorHorizontal() const;
		_UDSHL_EXP_API int			getSkippingFactorVertical() const;

		_UDSHL_EXP_API bool			isROIFormat() const;

		_UDSHL_EXP_API bool			isValidSize( const SIZE& sz ) const;

		/** get string representing this format desc
		 **/
		std::string		toString() const
		{
			return wstoas( toString_() );
		}
		std::wstring	toStringW() const
		{
			return toString_();
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

	private:
		_UDSHL_EXP_API	dstringw		toString_() const;
		_UDSHL_EXP_API	dstringw		getColorformatString_() const;

		VideoFormatDesc( const VideoFormatDesc& op2 );
		//VideoFormatDesc&	operator=( const VideoFormatDesc& op2 );

		VideoFormatDesc( const win32_utils::CVideoFormatDesc& desc, const dshowlib_dll::sTISScanningModeInfo& smi, bool isROIFormat );

		smart_ptr<win32_utils::CVideoFormatDesc> m_pDesc;

		int		m_binningModeId;
		int		m_binningHorizontal;
		int		m_binningVertical;
		int		m_skippingHorizontal;
		int		m_skippingVertical;
		bool	m_isROIFormat;
		bool	m_dontAllowROI;
	};

}

#endif // _VIDEOFORMATDESC_H_
