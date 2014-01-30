/*
 *   LinuxNetwork.cpp
 *
 *   Author: ROBOTIS
 *
 */
#include <iostream>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sstream>
#include "LinuxNetwork.h"
#include <stdio.h>

using namespace Robot;
using namespace std;


LinuxSocket::LinuxSocket() : m_sock ( -1 ), non_blocking(false)
{
	memset ( &m_addr, 0, sizeof ( m_addr ) );
}

LinuxSocket::~LinuxSocket()
{
	if ( is_valid() )
		::close ( m_sock );
}

bool LinuxSocket::create()
{
	m_sock = socket ( AF_INET,
			SOCK_STREAM,
			0 );

	if ( ! is_valid() )
		return false;

	// TIME_WAIT - argh
	int on = 1;
	if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 )
		return false;

	return true;
}

bool LinuxSocket::bind ( const char* hostname, const int port ) 
{
	if ( ! is_valid() )
	{
		return false;
	}

	m_addr.sin_family = AF_INET;
	//m_addr.sin_addr.s_addr = INADDR_ANY;
	inet_aton(hostname, &m_addr.sin_addr);
	m_addr.sin_port = htons ( port );

	int bind_return = ::bind ( m_sock,
			( struct sockaddr * ) &m_addr,
			sizeof ( m_addr ) );
	if ( bind_return == -1 )
	{
		return false;
	}

	return true;
}

bool LinuxSocket::listen() const
{
	if ( ! is_valid() )
	{
		return false;
	}

	int listen_return = ::listen ( m_sock, MAXCONNECTIONS );

	if ( listen_return == -1 )
	{
		return false;
	}
	return true;
}

bool LinuxSocket::accept ( LinuxSocket& new_socket ) const
{
	int addr_length = sizeof ( m_addr );
	new_socket.m_sock = ::accept ( m_sock, ( sockaddr * ) &m_addr, ( socklen_t * ) &addr_length );

	if ( new_socket.m_sock <= 0 )
		return false;
	else
	{
		int on = 1;
		//if (setsockopt(new_socket.m_sock, SOL_SOCKET, SO_KEEPALIVE, (const char*) &on, sizeof(on)) == -1)
		//	return false;
		if (setsockopt(new_socket.m_sock, IPPROTO_TCP, TCP_NODELAY, (const char*) &on, sizeof(on)) == -1)
			return false;
		return true;
	}
}

bool LinuxSocket::send ( const std::string s ) const
{
	int status = ::send ( m_sock, s.c_str(), s.size(), MSG_NOSIGNAL );
	if ( status == -1 )
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool LinuxSocket::send ( void* data, int length ) const
{
	if (non_blocking) {
		int ndone = 0;
		char* buf = (char*)data;

		while (ndone < length) {

			if (selectWrite(-1)) {
				int status = ::send(m_sock, (void*)(buf+ndone), length-ndone, MSG_NOSIGNAL);
				if ( status == -1 ) {
					cout<<"status == -1   errno == "<<errno<<"  in Socket::send\n";
					return false;
				}
				else if ( status == 0 ) {
					// connection has been broken
					return false;
				}

				// all is good
				ndone += status;
			}
		}
		return true;
	}
	else {
		int status = ::send(m_sock, data, length, MSG_NOSIGNAL);
		if ( status == -1 )
		{
			return false;
		}
		else
		{
			return true;
		}
	}
}

int LinuxSocket::recv ( std::string& s ) const
{
	if (non_blocking) {
		return 0;
	}
	else {
		char buf[MAXRECV+1];
		s = "";

		memset(buf, 0, MAXRECV+1);

		int status = ::recv(m_sock, buf, MAXRECV, 0);

		if ( status == -1 ) {
			cout<<"status == -1   errno == "<<errno<<"  in Socket::recv\n";
			return 0;
		}
		else if ( status == 0 ) {
			return 0;
		}
		else {
			s = buf;
			return status;
		}
	}
}

int LinuxSocket::recv ( void* data, int length ) const
{
	if (non_blocking) {
		int ndone=0;
		char* buf = (char*)data;

		while (ndone < length) {
			if (selectRead(-1)) {
				int status = ::recv ( m_sock, (void*)(buf+ndone), length-ndone, 0 );
				if ( status == -1 ) {
					cout<<"status == -1   errno == "<<errno<<"  in Socket::recv\n";
					return 0;
				}
				else if ( status == 0 ) {
					// connection has been broken
					return 0;
				}

				// all is good
				ndone += status;
			}
		}
		return ndone;
	}
	else {
		// blocking code will read everything at once
		int status = ::recv ( m_sock, data, length, 0 );

		if ( status == -1 ) {
			cout<<"status == -1   errno == "<<errno<<"  in Socket::recv\n";
			return 0;
		}
		else if ( status == 0 ) {
			return 0;
		}

		return status;
	}
}

bool LinuxSocket::selectRead(int tmout) const
{
	fd_set set;
	FD_ZERO(&set);
	FD_SET(m_sock, &set);

	// timeout structure
	struct timeval tm;
	tm.tv_sec = tmout/1000;
	tm.tv_usec = ((long)(tmout%1000)) * 1000;

	int result = select((int)m_sock+1, &set, 0, 0, tmout>=0 ? &tm : 0);

	// check for error
	if( result==-1 )
		return false;

	return true;
}

bool LinuxSocket::selectWrite(int tmout) const
{
	fd_set set;
	FD_ZERO(&set);
	FD_SET(m_sock, &set);

	// timeout structure
	struct timeval tm;
	tm.tv_sec = tmout/1000;
	tm.tv_usec = ((long)(tmout%1000)) * 1000;

	int result = select((int)m_sock+1, 0, &set, 0, tmout>=0 ? &tm : 0);

	// check for error
	if( result==-1)
		return false;

	return true;
}

bool LinuxSocket::connect(const std::string host, const int port)
{
	if ( ! is_valid() ) return false;

	m_addr.sin_family = AF_INET;
	m_addr.sin_port = htons ( port );

	int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );

	if ( errno == EAFNOSUPPORT ) return false;

	status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );

	if ( status == 0 )
		return true;
	else
		return false;
}

void LinuxSocket::set_non_blocking(const bool b)
{
	int opts;

	opts = fcntl(m_sock, F_GETFL);

	if (opts < 0) {
		return;
	}

	if (b)
		opts = (opts | O_NONBLOCK);
	else
		opts = (opts & ~O_NONBLOCK);

	fcntl(m_sock, F_SETFL, opts);

	non_blocking = b;
	printf("Socket is %s\n", (non_blocking)?"Non-Blocking":"Blocking");
}

// ----------------------------------------------------------------------

LinuxServer::LinuxServer ( const char* hostname, int port )
{
	if ( ! LinuxSocket::create() )
	{
		throw LinuxSocketException ( "Could not create server socket." );
	}

	if ( ! LinuxSocket::bind ( hostname, port ) )
	{
		throw LinuxSocketException ( "Could not bind to port." );
	}

	if ( ! LinuxSocket::listen() )
	{
		throw LinuxSocketException ( "Could not listen to socket." );
	}
}

LinuxServer::~LinuxServer()
{
}

void LinuxServer::set_non_blocking(const bool b)
{
	LinuxSocket::set_non_blocking(b);
}

const LinuxServer& LinuxServer::operator << ( const std::string& s ) const
{	
	if ( ! LinuxSocket::send ( s ) )
	{
		throw LinuxSocketException ( "Could not write to socket." );
	}

	return *this;
}

const LinuxServer& LinuxServer::operator << ( const int& i ) const
{
	std::stringstream ss;
	ss << i;

	if ( ! LinuxSocket::send ( ss.str() ) )
	{
		throw LinuxSocketException ( "Could not write to socket." );
	}

	return *this;
}

const LinuxServer& LinuxServer::operator >> ( std::string& s ) const
{
	if ( ! LinuxSocket::recv ( s ) )
	{
		throw LinuxSocketException ( "Could not read from socket." );
	}

	return *this;
}

void LinuxServer::accept ( LinuxServer& sock )
{
	if ( ! LinuxSocket::accept ( sock ) )
	{
		throw LinuxSocketException ( "Could not accept socket." );
	}
}

bool LinuxServer::send ( unsigned char* data, int length )
{
	return LinuxSocket::send(data, length);
}

int LinuxServer::recv ( unsigned char* data, int length )
{
	int ret = LinuxSocket::recv(data, length);
	return ret;
}
