
#ifndef DVECTOR_H_INC_
#define DVECTOR_H_INC_

#include "udshl_defs.h"

namespace _DSHOWLIB_NAMESPACE
{
	template<class TObj>
	class dvector
	{
	public:
		typedef TObj*		pointer;
		typedef TObj		value_type;
		typedef TObj&		reference;
		typedef const TObj&	const_reference;

		typedef pointer		iterator;
	public:
		dvector()
			: _pData( 0 ), _size( 0 ), _reserved( 0 )
		{
		}
		explicit dvector( size_t size, size_t reserved_size = 0 )
			: _pData( 0 ), _size( 0 ), _reserved( 0 )
		{
			if( size > reserved_size )
				reserved_size = size;
			uninitialized_alloc( reserved_size );
			uninitialized_construct( _pData, size, TObj() );
		}
		template<class TIter>
		dvector( TIter assign_beg, TIter assign_end )
			: _pData( 0 ), _size( 0 ), _reserved( 0 )
		{
			size_t diff = assign_end - assign_beg;
			uninitialized_alloc( diff );
			uninitialized_assign( assign_beg, diff );
		}
		dvector( pointer assign_beg, pointer assign_end )
			: _pData( 0 ), _size( 0 ), _reserved( 0 )
		{
			size_t diff = assign_end - assign_beg;
			uninitialized_alloc( diff );
			uninitialized_assign( assign_beg, diff );
		}
		dvector( size_t size, pointer assign_beg, pointer assign_end )
			: _pData( 0 ), _size( 0 ), _reserved( 0 )
		{
			uninitialized_alloc( size );

			size_t diff = min_( size_t(assign_end - assign_beg), size );
			uninitialized_assign( assign_beg, diff );
			if( diff < size )
			{
				uninitialized_construct( _pData + diff, size - diff, TObj() );
			}
		}
		dvector( const dvector& op2 )
			: _pData( 0 ), _size( 0 ), _reserved( 0 )
		{
			assign( op2.begin(), op2.end() );
		}

		~dvector()
		{
			pointer pDataEnd = _pData + _size;
			pointer pData = _pData;
			while( pData < pDataEnd )
			{
				destroy( pData++ );
			}
			deallocate( _pData, _size );
		}

		template<class TIter>
		void	assign( TIter beg, TIter end )
		{
			dvector vTmp( beg, end );
			swap( vTmp );
		}


		void	resize( size_t newSize )
		{
			dvector vTmp( newSize, _pData, _pData + _size );

			swap( vTmp );
		}

		void	reserve( size_t newSize )
		{
			if( _reserved >= newSize )	return;
			newSize = (newSize & (~0x1)) + 2;

			dvector reserve_vec( 0, newSize );
			if( _size > 0 )
				reserve_vec.uninitialized_assign( _pData, _size );
			swap( reserve_vec );
		}

		void	push_back( const_reference obj )
		{
			reserve( _size + 1 );
			if( capacity() > _size )
			{
				construct( _pData + _size, obj );
				_size += 1;
			}
		}

		void	clear()
		{
			pointer pDataEnd = _pData + _size;
			pointer pData = _pData;
			while( pData < pDataEnd )
			{
				destroy( pData++ );
			}
			deallocate( _pData, _size );

			_pData = 0;
			_size = 0;
			_reserved = 0;
		}

		iterator				begin()	const			{ return _pData + 0; }
		iterator				end()	const			{ return _pData + _size; }

		reference				operator[]( size_t i )			{ return _pData[i]; }
		const_reference			operator[]( size_t i ) const	{ return _pData[i]; }

		reference				front()					{ return _pData[0]; }
		const_reference			front()	const			{ return _pData[0]; }

		reference				back()					{ return _pData[_size-1]; }
		const_reference			back()	const			{ return _pData[_size-1]; }

		size_t		size() const						{ return _size; }
		bool		empty() const						{ return _size == 0; }
		size_t		capacity() const					{ return _reserved; }

		pointer			data()								{ return _pData; }
		const pointer	data()	const						{ return _pData; }

		void	swap( dvector& op2 )
		{
			std::swap( _pData, op2._pData );
			std::swap( _size, op2._size );
			std::swap( _reserved, op2._reserved );
		}

		dvector&		operator=( const dvector& op2 )
		{
			dvector vTmp( op2 );
			swap( vTmp );
			return *this;
		}

		bool			operator==( const dvector& op2 ) const
		{
			if( size() == op2.size() )
			{
				iterator j = op2.begin();
				for( iterator i = begin(); i != end(); ++i )
				{
					if( !(*i == *j) )
						return false;
				}
				return true;
			}
			return false;
		}
		bool			operator!=( const dvector& op2 ) const
		{
			return !( *this == op2);
		}
	private:
		size_t min_( size_t a, size_t b )
		{
			return (a < b) ? a : b;
		}
		void	uninitialized_alloc( size_t s )
		{
			_size = 0;
			_reserved = 0;
			if( s == 0 )
			{
				_pData = 0;
			}
			else
			{
				_pData = static_cast<pointer>(CoTaskMemAlloc( s * sizeof( value_type ) ));
				if( _pData != 0 )
					_reserved = s;
			}
		}
		template<class TIter>
		void	uninitialized_assign( TIter iter, size_t cnt )
		{
			while( _size < cnt )
			{
				construct( _pData + _size, *iter );
				++iter;
				++_size;
			}
		}
		void	uninitialized_construct( pointer beg, size_t cnt, const TObj& ctor )
		{
			while( _size < cnt )
				construct( beg + _size++, ctor );
		}
		static void	construct( pointer p, const value_type& value )
		{
			new((void*)p)value_type(value);
		}
		static void	destroy( pointer p )
		{
			p->~TObj();
		}
		void	deallocate( pointer p, size_t /* num */ )
		{
			if( p != 0 )
				CoTaskMemFree( p );
		}

		pointer	_pData;
		size_t	_size;
		size_t	_reserved;
	};
};

#endif // DVECTOR_H_INC_