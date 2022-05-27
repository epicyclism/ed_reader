//
// basic_file.h
//
// Windows/Linux minimal file wrapper.
//
// Copyright (c) 2004-2022 Paul Ranson. paul@epicyclism.com
//
// Refer to licence in repository.
//

#pragma once

#include <string>
#include <string_view>

#if defined (_MSC_VER)
#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

template<DWORD access, DWORD share, DWORD disposition, DWORD attributes> class basic_file
{
private :
	mutable HANDLE	hFile_ ;

public :
	basic_file() : hFile_ ( INVALID_HANDLE_VALUE )
	{
	}
	basic_file( char const* name ) : hFile_ ( INVALID_HANDLE_VALUE )
	{
		open ( name ) ;
	}
	basic_file( std::string const& name) : hFile_ ( INVALID_HANDLE_VALUE )
	{
		open ( name ) ;
	}
	basic_file( const basic_file& other )
	{
		hFile_ = other.hFile_ ;
		other.hFile_ = INVALID_HANDLE_VALUE ;
	}
	~basic_file()
	{
		if ( hFile_ != INVALID_HANDLE_VALUE )
		{
			::CloseHandle ( hFile_ ) ;
		}
	}
	basic_file& operator= ( const basic_file& other )
	{
		if ( this == &other )
		{
			return *this ;
		}
		hFile_ = other.hFile_ ;
		other.hFile_ = INVALID_HANDLE_VALUE ;
		return *this ;
	}
	bool	open ( char const* name )
	{
		if ( hFile_ != INVALID_HANDLE_VALUE )
		{
			::CloseHandle ( hFile_ ) ;
		}
		hFile_ = ::CreateFileA ( name,
									access,
									share,
									0,
									disposition,
									attributes,
									0 ) ;
		return good () ;
	}
	bool	open ( std::string const& name )
	{
		return open(name.c_str());
	}
	void close ()
	{
		if ( hFile_ != INVALID_HANDLE_VALUE )
		{
			::CloseHandle ( hFile_ ) ;
			hFile_ = INVALID_HANDLE_VALUE ;
		}
	}
	bool	good () const
	{
		return hFile_ != INVALID_HANDLE_VALUE ;
	}
	bool	write ( const void * pBuf, size_t nLen ) const
	{
		while ( nLen )
		{
			DWORD dwWritten ;
			if ( !::WriteFile ( hFile_,
								pBuf,
								static_cast<DWORD>( nLen ), // in the theoretical 64 bit world while nLen is > 0xffffffff it will cast to 0xffffffff, so this should be fine.
								&dwWritten,
								0 ))
			{
				return false ;
			}
			nLen -= dwWritten ;
		}
		return true ;
	}
	bool	read ( void * pBuf, size_t nLen ) const
	{
		while ( nLen )
		{
			DWORD dwRead ;
			if ( !::ReadFile ( hFile_,
								pBuf,
								static_cast<DWORD>( nLen ),
								&dwRead,
								0 ))
			{
				return false ;
			}
			nLen -= dwRead ;
		}
		return true ;
	}
	HANDLE  handle () const
	{
		return hFile_ ;
	}
	operator HANDLE () const
	{
		return hFile_ ;
	}
	operator bool() const
	{
		return good();
	}
};
using out_file_t = basic_file<GENERIC_WRITE, 0, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL> ;
#else
#include <fcntl.h>
#include <unistd.h>

template<int access, int flags> class basic_file
{
private:
	mutable int	h_;

public:
	basic_file() : h_{ -1 }
	{
	}
	basic_file(char const* name) : h_{-1}
	{
		open(name);
	}
	basic_file(std::string const& name) : h_{-1}
	{
		open(name);
	}
	basic_file(const basic_file& other)
	{
		h_ = other.h_;
		other.h_ = -1;
	}
	~basic_file()
	{
		if (h_ != -1)
		{
			::close(h_);
		}
	}
	basic_file& operator= (const basic_file& other)
	{
		if (this == &other)
		{
			return *this;
		}
		h_ = h_;
		other.h_ = -1;
		return *this;
	}
	bool	open(char const* name)
	{
		if (h_ != -1)
		{
			::close(h_);
		}
		h_ = ::open(name, flags | access, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IWOTH | S_IROTH);
		return good();
	}
	bool	open(std::string const& name)
	{
		return open(name.c_str());
	}
	void close()
	{
		if (h_ != -1)
		{
			::close(h_);
			h_ = -1;
		}
	}
	bool	good() const
	{
		return h_ != -1;
	}
	bool	write(const void* pBuf, size_t nLen) const
	{
		return ::write(h_, pBuf, nLen) != -1;
	}
	bool	read(void* pBuf, size_t nLen) const
	{
		return false;
	}
	int  handle() const
	{
		return h_;
	}
	operator bool() const
	{
		return good();
	}

};
using out_file_t = basic_file<O_WRONLY, O_CREAT | O_TRUNC>;
#endif