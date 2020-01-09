
#ifndef _FILTERLOADER_H_INC_
#define _FILTERLOADER_H_INC_

#include "FrameFilter.h"
#include "dstring.h"
#include "dvector.h"

namespace _DSHOWLIB_NAMESPACE
{
	class IFrameFilter_param;

	/**
	 *	The FilterLoader methods allows you to load frame filters from external DLLs.
	 *
	 *	The FilterLoader loads the frame filters from the ".ftf" filter DLLs in the following directories :
			1) The path where this DLL resides. (tis_udshl_*.dll)
			2) The Application Path or if you called <link>FilterLoader::setLoadPath<link> in the specified
				path.
	 *
	 *	The loaded frame filter factories can be queried by calling getAvailableFrameFilters.
	 *	You can then create a specific filter by calling createFilter with a FilterInfo or
	 *	by specifying the according name.
	 *
	 *	As all methods of this class are static, you can not create instances of this class.
	 */
	class FilterLoader
	{
	public:
		/**
		 *	Internal use only.
		 */
		_UDSHL_EXP_API static void		setLoadPath( const dstringw& path );

		_UDSHL_EXP_API static void		loadDirectory( const dstringw& path );
		_UDSHL_EXP_API static void		loadDirectory( const dstringw& path, const dstringw& search_pattern );

		/**
		 *	Retrieves a list with the names of the available frame filters.
		 *	To instantiate a filter from this list, call
		 *	<link>createFilter</link>.
		 *	@return A list with FilterInfo objects which describe the available filters.
		 */
		static std::vector<FilterInfo>	getAvailableFrameFilters( tFilterClass filterClass = eFC_ALL )
		{
			dvector<FilterInfo> tmp = getAvailableFrameFilters_( filterClass );
			return std::vector<FilterInfo>( tmp.begin(), tmp.end() );
		}

		/** Creates an instance of a specified frame filter.
		 * @return 0 when no according filter could be created, otherwise a reference to the newly created filter.
		 */
		_UDSHL_EXP_API
		static smart_com<IFrameFilter>	createFilter( const FilterInfo& fi );
		/** Creates an instance of a specified frame filter.
		 * When module == std::string(), then the loader creates the first filter with the specified name
		 *	ignoring the module.
		 * The look up of the filter name is case-sensitive.
		 * @return 0 when no according filter could be created, otherwise a reference to the newly created filter.
		 */
		_UDSHL_EXP_API
		static smart_com<IFrameFilter>	createFilter( const dstringa& filter_name, const dstringa& module = dstringa() );
		_UDSHL_EXP_API
		static smart_com<IFrameFilter>	createFilter( const dstringw& filter_name, const dstringw& module = dstringw() );
	private:
		_UDSHL_EXP_API
		static dvector<FilterInfo>		getAvailableFrameFilters_( tFilterClass filterClass = eFC_ALL );

		/**
		 *	Do not create any instances of this class.
		 */
		FilterLoader() {}
	};
}

#endif