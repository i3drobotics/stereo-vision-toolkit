
#ifndef DSTRING_H_INC_
#define DSTRING_H_INC_

#include <string>
#include <string.h>

#if _MSC_VER >= 1300 && _MSC_VER < 1400
// Required for StringCchPrintf
#include <Strsafe.h>
#endif

#include "udshl_defs.h"

#include "libutil_string.h"

namespace _DSHOWLIB_NAMESPACE 
{
	class dstringa
	{
	public:
		dstringa() 
			: buf_( 0 )
		{
			char str = 0;
			buf_ = init( &str, 1 );
		}

		dstringa( const char* str, size_t len )
			: buf_( 0 )
		{
			buf_ = init( str, len );
		}

		dstringa( const char* str )
			: buf_( 0 )
		{
			assert( str );
			buf_ = init( str, calc_len( str ) );
		}
		dstringa( const std::basic_string<char>& str )
			: buf_( 0 )
		{
			buf_ = init( str.c_str(), str.size() );
		}

		dstringa( const dstringa& str )
			: buf_( 0 )
		{
			buf_ = init( str.buf_, calc_len( str.buf_ ) );
		}
#if _MSC_VER >= 1600
		dstringa( dstringa&& str )
			: buf_( 0 )
		{
			swap( str );
		}
#endif
		~dstringa()
		{
			free();
		}

		dstringa& operator=( const dstringa& str )
		{
			dstringa temp( str );
			swap( temp );
			return *this;
		}
		dstringa& operator=( const char* str )
		{
			dstringa temp( str );
			swap( temp );
			return *this;
		}
		void swap( dstringa& str )
		{
			std::swap( buf_, str.buf_ );
		}

		operator std::basic_string<char>()	const	{ return  std::basic_string<char>( buf_ ); }

		const char*		c_str() const			{ return buf_; }

		bool	empty() const						{ return buf_[0] == 0; }

		std::basic_string<char>	to_string() const	{ return  std::basic_string<char>( buf_ ); }


		bool	operator==( const std::basic_string<char>& rhs ) const
		{
			return to_string() == rhs;
		}
		bool	operator!=( const std::basic_string<char>& rhs ) const
		{
			return to_string() != rhs;
		}
	private:
		static size_t		calc_len( const char* str )
		{
			return strlen( str );
		}
		static char*		init( const char* str, size_t str_len )
		{
			char* buf = static_cast<char*>( CoTaskMemAlloc( (str_len + 1) * sizeof( str[0] ) ) );
	#if _MSC_VER >= 1400
			strcpy_s( buf, str_len + 1, str );
	#elif _MSC_VER >= 1300
			StringCchPrintfA( buf, str_len + 1, str );
	#else
			strcpy( buf, str );
	#endif
			buf[str_len] = 0;
			return buf;
		}
		void		free()
		{
			CoTaskMemFree( buf_ );
		}
		char*		buf_;
	};

	inline bool	operator<( const dstringa& lhs, const dstringa& rhs )
	{
		return lhs.to_string() < rhs.to_string();
	}

	class dstringw
	{
	public:
		dstringw() 
			: buf_( 0 )
		{
			wchar_t str = 0;
			buf_ = init( &str, 1 );
		}

		dstringw( const wchar_t* str, size_t len )
			: buf_( 0 )
		{
			buf_ = init( str, len );
		}

		dstringw( const wchar_t* str )
			: buf_( 0 )
		{
			assert( str );
			buf_ = init( str, calc_len( str ) );
		}
		dstringw( const std::basic_string<wchar_t>& str )
			: buf_( 0 )
		{
			buf_ = init( str.c_str(), str.size() );
		}

		dstringw( const dstringw& str )
			: buf_( 0 )
		{
			buf_ = init( str.buf_, calc_len( str.buf_ ) );
		}
#if _MSC_VER >= 1600
		dstringw( dstringw&& str )
			: buf_( 0 )
		{
			swap( str );
		}
#endif
		~dstringw()
		{
			free();
		}

		dstringw& operator=( const dstringw& str )
		{
			dstringw temp( str );
			swap( temp );
			return *this;
		}
		dstringw& operator=( const wchar_t* str )
		{
			dstringw temp( str );
			swap( temp );
			return *this;
		}
		void swap( dstringw& str )
		{
			std::swap( buf_, str.buf_ );
		}

		operator std::basic_string<wchar_t>()	const	{ return  std::basic_string<wchar_t>( buf_ ); }

		const wchar_t*		c_str() const			{ return buf_; }

		bool	empty() const						{ return buf_[0] == 0; }

		std::basic_string<wchar_t>	to_string() const	{ return  std::basic_string<wchar_t>( buf_ ); }


		bool	operator==( const std::basic_string<wchar_t>& rhs ) const
		{
			return std::basic_string<wchar_t>( *this ) == rhs;
		}
		bool	operator!=( const std::basic_string<wchar_t>& rhs ) const
		{
			return std::basic_string<wchar_t>( *this ) != rhs;
		}

	private:
		static size_t		calc_len( const wchar_t* str )
		{
			return wcslen( str );
		}
		static wchar_t*		init( const wchar_t* str, size_t str_len )
		{
			wchar_t* buf = static_cast<wchar_t*>( CoTaskMemAlloc( (str_len + 1) * sizeof( wchar_t ) ) );
#if _MSC_VER >= 1400
			wcscpy_s( buf, str_len + 1, str );
#elif _MSC_VER >= 1300
			StringCchPrintfW( buf, str_len + 1, str );
#else
			wcscpy( buf, str );
#endif
			buf[str_len] = 0;
			return buf;
		}
		void		free()
		{
			CoTaskMemFree( buf_ );
		}
		wchar_t*		buf_;
	};

	inline bool	operator<( const dstringw& lhs, const dstringw& rhs )
	{
		return lhs.to_string() < rhs.to_string();
	}
};

#endif // DSTRING_H_INC_
