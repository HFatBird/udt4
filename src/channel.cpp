/*****************************************************************************
Copyright (c) 2001 - 2011, The Board of Trustees of the University of Illinois.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.

* Redistributions in binary form must reproduce the
  above copyright notice, this list of conditions
  and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the University of Illinois
  nor the names of its contributors may be used to
  endorse or promote products derived from this
  software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/****************************************************************************
written by
   Yunhong Gu, last updated 01/27/2011
*****************************************************************************/

#ifndef WIN32
   #include <netdb.h>
   #include <arpa/inet.h>
   #include <unistd.h>
   #include <fcntl.h>
   #include <cstring>
   #include <cstdio>
   #include <cerrno>
#else
   #include <winsock2.h>
   #include <ws2tcpip.h>
   #ifdef LEGACY_WIN32
      #include <wspiapi.h>
   #endif
#endif
#include "channel.h"
#include "packet.h"

#ifdef WIN32
   #define socklen_t int
#endif

#ifndef WIN32
   #define NET_ERROR errno
#else
   #define NET_ERROR WSAGetLastError()
#endif

#ifdef UDT_COMPACT_NO_TIMESTAMP
   static const int UDT_WIRE_HEADER_SIZE = 12;
#else
   static const int UDT_WIRE_HEADER_SIZE = CPacket::m_iPktHdrSize;
#endif


CChannel::CChannel():
m_iIPversion(AF_INET),
m_iSockAddrSize(sizeof(sockaddr_in)),
m_iSocket(),
m_iSndBufSize(65536),
m_iRcvBufSize(65536)
{
}

CChannel::CChannel(int version):
m_iIPversion(version),
m_iSocket(),
m_iSndBufSize(65536),
m_iRcvBufSize(65536)
{
   m_iSockAddrSize = (AF_INET == m_iIPversion) ? sizeof(sockaddr_in) : sizeof(sockaddr_in6);
}

CChannel::~CChannel()
{
}

void CChannel::open(const sockaddr* addr)
{
   // construct an socket
   m_iSocket = ::socket(m_iIPversion, SOCK_DGRAM, 0);

   #ifdef WIN32
      if (INVALID_SOCKET == m_iSocket)
   #else
      if (m_iSocket < 0)
   #endif
      throw CUDTException(1, 0, NET_ERROR);

   if (NULL != addr)
   {
      socklen_t namelen = m_iSockAddrSize;

      if (0 != ::bind(m_iSocket, addr, namelen))
         throw CUDTException(1, 3, NET_ERROR);
   }
   else
   {
      //sendto or WSASendTo will also automatically bind the socket
      addrinfo hints;
      addrinfo* res;

      memset(&hints, 0, sizeof(struct addrinfo));

      hints.ai_flags = AI_PASSIVE;
      hints.ai_family = m_iIPversion;
      hints.ai_socktype = SOCK_DGRAM;

      if (0 != ::getaddrinfo(NULL, "0", &hints, &res))
         throw CUDTException(1, 3, NET_ERROR);

      if (0 != ::bind(m_iSocket, res->ai_addr, res->ai_addrlen))
         throw CUDTException(1, 3, NET_ERROR);

      ::freeaddrinfo(res);
   }

   setUDPSockOpt();
}

void CChannel::open(UDPSOCKET udpsock)
{
   m_iSocket = udpsock;
   setUDPSockOpt();
}

void CChannel::setUDPSockOpt()
{
   #if defined(BSD) || defined(OSX)
      // BSD system will fail setsockopt if the requested buffer size exceeds system maximum value
      int maxsize = 64000;
      if (0 != ::setsockopt(m_iSocket, SOL_SOCKET, SO_RCVBUF, (char*)&m_iRcvBufSize, sizeof(int)))
         ::setsockopt(m_iSocket, SOL_SOCKET, SO_RCVBUF, (char*)&maxsize, sizeof(int));
      if (0 != ::setsockopt(m_iSocket, SOL_SOCKET, SO_SNDBUF, (char*)&m_iSndBufSize, sizeof(int)))
         ::setsockopt(m_iSocket, SOL_SOCKET, SO_SNDBUF, (char*)&maxsize, sizeof(int));
   #else
      // for other systems, if requested is greated than maximum, the maximum value will be automactally used
      if ((0 != ::setsockopt(m_iSocket, SOL_SOCKET, SO_RCVBUF, (char*)&m_iRcvBufSize, sizeof(int))) ||
          (0 != ::setsockopt(m_iSocket, SOL_SOCKET, SO_SNDBUF, (char*)&m_iSndBufSize, sizeof(int))))
         throw CUDTException(1, 3, NET_ERROR);
   #endif

   timeval tv;
   tv.tv_sec = 0;
   #if defined (BSD) || defined (OSX)
      // Known BSD bug as the day I wrote this code.
      // A small time out value will cause the socket to block forever.
      tv.tv_usec = 10000;
   #else
      tv.tv_usec = 100;
   #endif

   #ifdef UNIX
      // Set non-blocking I/O
      // UNIX does not support SO_RCVTIMEO
      int opts = ::fcntl(m_iSocket, F_GETFL);
      if (-1 == ::fcntl(m_iSocket, F_SETFL, opts | O_NONBLOCK))
         throw CUDTException(1, 3, NET_ERROR);
   #elif WIN32
      DWORD ot = 1; //milliseconds
      if (0 != ::setsockopt(m_iSocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&ot, sizeof(DWORD)))
         throw CUDTException(1, 3, NET_ERROR);
   #else
      // Set receiving time-out value
      if (0 != ::setsockopt(m_iSocket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(timeval)))
         throw CUDTException(1, 3, NET_ERROR);
   #endif
}

void CChannel::close() const
{
   #ifndef WIN32
      ::close(m_iSocket);
   #else
      ::closesocket(m_iSocket);
   #endif
}

int CChannel::getSndBufSize()
{
   socklen_t size = sizeof(socklen_t);
   ::getsockopt(m_iSocket, SOL_SOCKET, SO_SNDBUF, (char *)&m_iSndBufSize, &size);
   return m_iSndBufSize;
}

int CChannel::getRcvBufSize()
{
   socklen_t size = sizeof(socklen_t);
   ::getsockopt(m_iSocket, SOL_SOCKET, SO_RCVBUF, (char *)&m_iRcvBufSize, &size);
   return m_iRcvBufSize;
}

void CChannel::setSndBufSize(int size)
{
   m_iSndBufSize = size;
}

void CChannel::setRcvBufSize(int size)
{
   m_iRcvBufSize = size;
}

void CChannel::getSockAddr(sockaddr* addr) const
{
   socklen_t namelen = m_iSockAddrSize;
   ::getsockname(m_iSocket, addr, &namelen);
}

void CChannel::getPeerAddr(sockaddr* addr) const
{
   socklen_t namelen = m_iSockAddrSize;
   ::getpeername(m_iSocket, addr, &namelen);
}

int CChannel::sendto(const sockaddr* addr, CPacket& packet) const
{
   // convert control information into network order
   if (packet.getFlag())
      for (int i = 0, n = packet.getLength() / 4; i < n; ++ i)
         *((uint32_t *)packet.m_pcData + i) = htonl(*((uint32_t *)packet.m_pcData + i));
   int res = -1;

   #ifndef WIN32
      msghdr mh;
      mh.msg_name = (sockaddr*)addr;
      mh.msg_namelen = m_iSockAddrSize;
      mh.msg_iovlen = 2;
      mh.msg_control = NULL;
      mh.msg_controllen = 0;
      mh.msg_flags = 0;
      #ifdef UDT_COMPACT_NO_TIMESTAMP
         uint32_t header[3];
         header[0] = htonl(packet.m_nHeader[0]);
         header[1] = htonl(packet.m_nHeader[1]);
         header[2] = htonl(packet.m_nHeader[3]);
         iovec iov[2];
         iov[0].iov_base = (char*)header;
         iov[0].iov_len = UDT_WIRE_HEADER_SIZE;
         iov[1] = packet.m_PacketVector[1];
         mh.msg_iov = iov;
      #else
         uint32_t* p = packet.m_nHeader;
         for (int j = 0; j < 4; ++ j)
         {
            *p = htonl(*p);
            ++ p;
         }
         mh.msg_iov = (iovec*)packet.m_PacketVector;
      #endif
      res = ::sendmsg(m_iSocket, &mh, 0);
   #else
      DWORD size = 0;
      #ifdef UDT_COMPACT_NO_TIMESTAMP
         uint32_t header[3];
         header[0] = htonl(packet.m_nHeader[0]);
         header[1] = htonl(packet.m_nHeader[1]);
         header[2] = htonl(packet.m_nHeader[3]);
         WSABUF iov[2];
         iov[0].buf = (char*)header;
         iov[0].len = UDT_WIRE_HEADER_SIZE;
         iov[1].buf = packet.m_PacketVector[1].iov_base;
         iov[1].len = packet.m_PacketVector[1].iov_len;
         size = UDT_WIRE_HEADER_SIZE + packet.getLength();
         int addrsize = m_iSockAddrSize;
         res = ::WSASendTo(m_iSocket, iov, 2, &size, 0, addr, addrsize, NULL, NULL);
      #else
         uint32_t* p = packet.m_nHeader;
         for (int j = 0; j < 4; ++ j)
         {
            *p = htonl(*p);
            ++ p;
         }
         size = CPacket::m_iPktHdrSize + packet.getLength();
         int addrsize = m_iSockAddrSize;
         res = ::WSASendTo(m_iSocket, (LPWSABUF)packet.m_PacketVector, 2, &size, 0, addr, addrsize, NULL, NULL);
      #endif
      res = (0 == res) ? size : -1;
   #endif

   #ifndef UDT_COMPACT_NO_TIMESTAMP
      uint32_t* p2 = packet.m_nHeader;
      for (int k = 0; k < 4; ++ k)
      {
         *p2 = ntohl(*p2);
         ++ p2;
      }
   #endif

   if (packet.getFlag())
   {
      for (int l = 0, n = packet.getLength() / 4; l < n; ++ l)
         *((uint32_t *)packet.m_pcData + l) = ntohl(*((uint32_t *)packet.m_pcData + l));
   }

   return res;
}

int CChannel::recvfrom(sockaddr* addr, CPacket& packet) const
{
   int res = -1;
   #ifndef WIN32
      msghdr mh;   
      mh.msg_name = addr;
      mh.msg_namelen = m_iSockAddrSize;
      mh.msg_iovlen = 2;
      mh.msg_control = NULL;
      mh.msg_controllen = 0;
      mh.msg_flags = 0;
      #ifdef UDT_COMPACT_NO_TIMESTAMP
         uint32_t header[3];
         iovec iov[2];
         iov[0].iov_base = (char*)header;
         iov[0].iov_len = UDT_WIRE_HEADER_SIZE;
         iov[1] = packet.m_PacketVector[1];
         mh.msg_iov = iov;
      #else
         mh.msg_iov = packet.m_PacketVector;
      #endif

      #ifdef UNIX
         fd_set set;
         timeval tv;
         FD_ZERO(&set);
         FD_SET(m_iSocket, &set);
         tv.tv_sec = 0;
         tv.tv_usec = 10000;
         ::select(m_iSocket+1, &set, NULL, &set, &tv);
      #endif

      res = ::recvmsg(m_iSocket, &mh, 0);
   #else
      DWORD size = 0;
      #ifdef UDT_COMPACT_NO_TIMESTAMP
         uint32_t header[3];
         WSABUF iov[2];
         iov[0].buf = (char*)header;
         iov[0].len = UDT_WIRE_HEADER_SIZE;
         iov[1].buf = packet.m_PacketVector[1].iov_base;
         iov[1].len = packet.m_PacketVector[1].iov_len;
         size = UDT_WIRE_HEADER_SIZE + packet.getLength();
      #else
         size = CPacket::m_iPktHdrSize + packet.getLength();
      #endif
      DWORD flag = 0;
      int addrsize = m_iSockAddrSize;
      #ifdef UDT_COMPACT_NO_TIMESTAMP
         res = ::WSARecvFrom(m_iSocket, iov, 2, &size, &flag, addr, &addrsize, NULL, NULL);
      #else
         res = ::WSARecvFrom(m_iSocket, (LPWSABUF)packet.m_PacketVector, 2, &size, &flag, addr, &addrsize, NULL, NULL);
      #endif
      res = (0 == res) ? size : -1;
   #endif

   if (res <= 0)
   {
      packet.setLength(-1);
      return -1;
   }

   packet.setLength(res - UDT_WIRE_HEADER_SIZE);

   #ifdef UDT_COMPACT_NO_TIMESTAMP
      #ifndef WIN32
         packet.m_nHeader[0] = ntohl(header[0]);
         packet.m_nHeader[1] = ntohl(header[1]);
         packet.m_nHeader[3] = ntohl(header[2]);
      #else
         packet.m_nHeader[0] = ntohl(header[0]);
         packet.m_nHeader[1] = ntohl(header[1]);
         packet.m_nHeader[3] = ntohl(header[2]);
      #endif
      packet.m_nHeader[2] = 0;
   #else
      uint32_t* p = packet.m_nHeader;
      for (int i = 0; i < 4; ++ i)
      {
         *p = ntohl(*p);
         ++ p;
      }
   #endif

   if (packet.getFlag())
   {
      for (int j = 0, n = packet.getLength() / 4; j < n; ++ j)
         *((uint32_t *)packet.m_pcData + j) = ntohl(*((uint32_t *)packet.m_pcData + j));
   }

   return packet.getLength();
}
