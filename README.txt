Copyright (c) 2001 - 2011, The Board of Trustees of the University of Illinois.
All Rights Reserved.
Copyright (c) 2011, Google, Inc. All Rights Reserved.

UDP-based Data Transfer (UDT) Library - version 4
Author: Yunhong Gu [yunhong.gu @ gmail.com]

UDT version 4 is free software under BSD License. See ./LICENSE.txt.

============================================================================

UDT Website:
http://udt.sf.net
http://sf.net/projects/udt/ 


CONTENT: 
./src:     UDT source code 
./app:     Example programs 
./doc:     UDT documentation (HTML)
./win:     Visual C++ project files for the Windows version of UDT 


To make: 
     make -e os=XXX arch=YYY 
Windows (cross-compilation):
     make CXX='compiler' os=WIN32

XXX: [LINUX(default), BSD, OSX] 
YYY: [IA32(default), POWERPC, IA64, AMD64] 

For example, on OS X, you may need to do "make -e os=OSX arch=POWERPC"; 
on 32-bit i386 Linux system, simply use "make".

On Windows systems, use the Visual C++ project files in ./win directory.


To use UDT in your application:
Read index.htm in ./doc. The documentation is in HTML format and requires your
browser to support JavaScript.


Questions? please post to the UDT project forum:
https://sourceforge.net/projects/udt/forums


============================================================================
UDT packet/header layout (for narrowband tuning)

Data packet fixed header: 16 bytes
  1) Sequence Number (4 bytes)
  2) Message Number + boundary/order flags (4 bytes)
  3) Time Stamp (4 bytes)
  4) Destination Socket ID (4 bytes)

Control packet fixed header: 16 bytes
  1) Flag + Type (4 bytes)
  2) Additional Info (4 bytes)
  3) Time Stamp (4 bytes)
  4) Destination Socket ID (4 bytes)
  + Control payload (type dependent)

Notes:
  - UDT itself has no extra application-layer checksum field in this header.
  - Integrity mainly relies on UDP/IP stack checksums.

Optional compact build flags (non-standard, both peers must match):
  -DUDT_COMPACT_NO_TIMESTAMP
     Remove the 4-byte timestamp from on-wire packet header.
     Data/control fixed header changes from 16 bytes to 12 bytes on wire.
     Internally the timestamp field is set to 0 when receiving.

  -DUDT_COMPACT_CTRL_HEADER
     For control packets with empty control info fields, remove legacy
     4-byte dummy payload. Saves 4 bytes on each such control packet.

  -DUDT_COMPACT_HANDSHAKE
     Remove peer IP field (16 bytes) from handshake payload.
     Handshake payload changes from 48 bytes to 32 bytes.
