#ifndef _SMARTPTR_H_INC_
#define _SMARTPTR_H_INC_

#include <windows.h>


// forward declarations
template<typename TType> class smart_ptr;

class refcount_ptr_base
{
public:
	virtual ~refcount_ptr_base() {};

	virtual void		AddRef() = 0;
	virtual void		Release() = 0;

	virtual void*		GetPtr() = 0;
};

template<class T>
class refcount_ptr : public refcount_ptr_base
{
public:
	refcount_ptr( T* p )
		: ref_count_( 1 ), data_( p )
	{
	}

	~refcount_ptr()
	{
		delete data_;
	}

	void		AddRef()
	{
		InterlockedIncrement( &ref_count_ );
	}
	void		Release()
	{
		if( ::InterlockedDecrement( &ref_count_ ) == 0 )
		{
			delete this;
		}
	}
	virtual void* GetPtr() 
	{
		return data_;
	}
private:
	LONG			ref_count_;
	T*				data_;
};

template<class T, class TTarget>
class refcount_ptr_wrapper : public refcount_ptr_base
{
public:
	refcount_ptr_wrapper( refcount_ptr_base* target )
		: ref_count_( 1 ), target_( target )
	{
		target_->AddRef();
	}

	~refcount_ptr_wrapper()
	{
		target_->Release();
	}

	void		AddRef()
	{
		InterlockedIncrement( &ref_count_ );
	}
	void		Release()
	{
		if( ::InterlockedDecrement( &ref_count_ ) == 0 )
		{
			delete this;
		}
	}
	virtual void* GetPtr() 
	{
		return static_cast<T*>( static_cast<TTarget*>(target_->GetPtr()) );
	}
private:
	refcount_ptr_base*	target_;
	LONG				ref_count_;
};

template<typename TType>
class smart_ptr
{
public:
	typedef TType			value_type;
	typedef TType*			pointer;
	typedef const TType*	const_pointer;
	typedef TType&			reference;
	typedef const TType&	const_reference;
	
	typedef smart_ptr<TType>	object_type;

	typedef void (* deleter_type)( void* );
public:
	/** ctor */
	smart_ptr( pointer p = 0 )
		: ptr_( new refcount_ptr<TType>( p ) )
	{

	}

	template<typename TTOp2>
	smart_ptr( const smart_ptr<TTOp2>& op2 )
	{
		const_pointer p = static_cast<const_pointer>(op2.get());		// compile time test ..
		UNREFERENCED_PARAMETER( p );

		ptr_ = new refcount_ptr_wrapper<TType, TTOp2>( op2.get_refcount_ptr() );
	}

	/** copy constructor */
	smart_ptr( const object_type& op2 )
	{
		ptr_ = op2.ptr_;
		ptr_->AddRef();
	}
	/** dtor */
	~smart_ptr()
	{
		ptr_->Release();
	}

	/** assignment operator*/
	smart_ptr&	operator=( const smart_ptr& op2 )
	{
		smart_ptr tmp( op2 );
		tmp.swap( *this );
		return *this;
	}

	/** member selection operator
	 * may fail when the pointer is empty
	 */
	reference			operator*()				{ return *get(); }
	/** member selection operator
	 * may fail when the pointer is empty
	 */
	const_reference		operator*() const		{ return *get(); }
	/** member selection operator
	 * may fail when the pointer is empty
	 */
	pointer				operator->()			{ return get(); }
	/** member selection operator
	 * may fail when the pointer is empty
	 */
	const_pointer		operator->() const		{ return get(); }

	/** compare operator== */
	bool		operator==( const smart_ptr& op2 ) const
	{
		return get() == op2.get();
	}
	/** compare operator!= */
	bool		operator!=( const smart_ptr& op2 ) const
	{
		return get() != op2.get();
	}
	
	/** compare operator==.
	 * implemented cause of performance reasons
	 */
	bool		operator==( const pointer p ) const
	{
		return get() == p;
	}
	/** compare operator!=.
	 * implemented cause of performance reasons
	 */
	bool		operator!=( const pointer p ) const
	{
		return get() != p;
	}

	/** lets you get the internal pointer representation
	 * <strong>do not delete the internal rep, use destroy</strong>
	 */
	pointer			get()
	{
		return static_cast<pointer>( ptr_->GetPtr() );
	}

	/** lets you get the internal pointer representation
	 * <strong>do not delete the internal rep, use destroy</strong>
	 */
	const_pointer	get() const
	{
		return static_cast<pointer>( ptr_->GetPtr() );
	}

	/** destroys the internal reference and sets this to zero */
	void			destroy()
	{
		smart_ptr tmp;
		tmp.swap( *this );
	}

	/** swaps this with op2 */
	void			swap( smart_ptr& op2 )
	{
		std::swap( ptr_, op2.ptr_ );
	}

	refcount_ptr_base*	get_refcount_ptr() const { return ptr_; }

private:
	mutable refcount_ptr_base*		ptr_;
};


template<typename TType>
inline bool		operator==( const void* p, const smart_ptr<TType>& op2 )
{
	return op2.get() == p;
}

template<typename TType>
inline bool		operator!=( const void* p, const smart_ptr<TType>& op2 )
{
	return op2.get() != p;
}

template<typename TType>
inline void	swap( smart_ptr<TType>& op1, smart_ptr<TType>& op2 )	{ op1.swap( op2 ); }

#endif