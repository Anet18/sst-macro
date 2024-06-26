/**
Copyright 2009-2022 National Technology and Engineering Solutions of Sandia,
LLC (NTESS).  Under the terms of Contract DE-NA-0003525, the U.S. Government
retains certain rights in this software.

Sandia National Laboratories is a multimission laboratory managed and operated
by National Technology and Engineering Solutions of Sandia, LLC., a wholly
owned subsidiary of Honeywell International, Inc., for the U.S. Department of
Energy's National Nuclear Security Administration under contract DE-NA0003525.

Copyright (c) 2009-2022, NTESS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Questions? Contact sst-macro-help@sandia.gov
*/

#ifndef MPITESTCONF_H_INCLUDED
#define MPITESTCONF_H_INCLUDED


/* Fortran names are lowercase with no trailing underscore */
/* #undef F77_NAME_LOWER */

/* Fortran names are lowercase with two trailing underscores */
/* #undef F77_NAME_LOWER_2USCORE */

/* Fortran names are lowercase with two trailing underscores in stdcall */
/* #undef F77_NAME_LOWER_2USCORE_STDCALL */

/* Fortran names are lowercase with no trailing underscore in stdcall */
/* #undef F77_NAME_LOWER_STDCALL */

/* Fortran names are lowercase with one trailing underscore */
#define F77_NAME_LOWER_USCORE 1

/* Fortran names are lowercase with one trailing underscore in stdcall */
/* #undef F77_NAME_LOWER_USCORE_STDCALL */

/* Fortran names preserve the original case */
/* #undef F77_NAME_MIXED */

/* Fortran names preserve the original case in stdcall */
/* #undef F77_NAME_MIXED_STDCALL */

/* Fortran names preserve the original case with one trailing underscore */
/* #undef F77_NAME_MIXED_USCORE */

/* Fortran names preserve the original case with one trailing underscore in
   stdcall */
/* #undef F77_NAME_MIXED_USCORE_STDCALL */

/* Fortran names are uppercase */
/* #undef F77_NAME_UPPER */

/* Fortran names are uppercase in stdcall */
/* #undef F77_NAME_UPPER_STDCALL */

/* Define to 1 if the system has the type `double _Complex'. */
#define HAVE_DOUBLE__COMPLEX 1

/* Define to 1 if the system has the type `float _Complex'. */
#define HAVE_FLOAT__COMPLEX 1

/* Define if Fortran is supported */
#define HAVE_FORTRAN_BINDING 1

/* Define to 1 if you have the `getrusage' function. */
#define HAVE_GETRUSAGE 1

/* Define if struct hostent contains h_addr_list */
#define HAVE_H_ADDR_LIST 1

/* Define to 1 if you have the <inttypes.h> header file. */
#define HAVE_INTTYPES_H 1

/* Define if iostream is available */
#define HAVE_IOSTREAM 1

/* Define to 1 if you have the <iostream.h> header file. */
/* #undef HAVE_IOSTREAM_H */

/* Define if long double is supported */
#define HAVE_LONG_DOUBLE 1

/* Define to 1 if the system has the type `long double _Complex'. */
/* undef HAVE_LONG_DOUBLE__COMPLEX */

/* Define if compiler supports long long */
#define HAVE_LONG_LONG 1

/* Define to 1 if you have the <memory.h> header file. */
#define HAVE_MEMORY_H 1

/* Define if MPI_2COMPLEX is available */
/* #undef HAVE_MPI_2COMPLEX */

/* Define if MPI_2DOUBLE_COMPLEX is available */
/* #undef HAVE_MPI_2DOUBLE_COMPLEX */

/* Define to 1 if you have the `MPI_Init_thread' function. */
/* #undef HAVE_MPI_INIT_THREAD */

/* Define if MPI_INTEGER16 is available */
#define HAVE_MPI_INTEGER16 1

/* Define if MPI-IO (really ROMIO) is included */
#define HAVE_MPI_IO 1

/* Define if Dynamic Process functionality is available */
#define HAVE_MPI_SPAWN 1

/* Define if MPI_Win_create is available */
/* undef HAVE_MPI_WIN_CREATE */

/* define if the compiler implements namespaces */
#define HAVE_NAMESPACES /**/

/* define if the compiler implements namespace std */
#define HAVE_NAMESPACE_STD /**/

/* Define to 1 if you have the `pthread_barrier_init' function. */
/* #undef HAVE_PTHREAD_BARRIER_INIT */

/* Define to 1 if you have the `pthread_barrier_wait' function. */
/* #undef HAVE_PTHREAD_BARRIER_WAIT */

/* Define to 1 if you have the `pthread_create' function. */
/* #undef HAVE_PTHREAD_CREATE */

/* Define to 1 if you have the <pthread.h> header file. */
/* #undef HAVE_PTHREAD_H */

/* Define to 1 if you have the `pthread_yield' function. */
/* #undef HAVE_PTHREAD_YIELD */

/* Define to 1 if you have the <stdarg.h> header file. */
#define HAVE_STDARG_H 1

/* Define to 1 if you have the <stdint.h> header file. */
#define HAVE_STDINT_H 1

/* Define to 1 if you have the <stdlib.h> header file. */
#define HAVE_STDLIB_H 1

/* Define to 1 if you have the <strings.h> header file. */
#define HAVE_STRINGS_H 1

/* Define to 1 if you have the <string.h> header file. */
#define HAVE_STRING_H 1

/* Define to 1 if you have the <sys/resource.h> header file. */
#define HAVE_SYS_RESOURCE_H 1

/* Define to 1 if you have the <sys/stat.h> header file. */
#define HAVE_SYS_STAT_H 1

/* Define to 1 if you have the <sys/time.h> header file. */
#define HAVE_SYS_TIME_H 1

/* Define to 1 if you have the <sys/types.h> header file. */
#define HAVE_SYS_TYPES_H 1

/* Define to 1 if you have the <unistd.h> header file. */
#define HAVE_UNISTD_H 1

/* Define to 1 if the system has the type `_Bool'. */
#define HAVE__BOOL 1

/* Define if MPI IO uses MPI_Request */
/* #undef MPIO_USES_MPI_REQUEST */

/* Name of package */
#define PACKAGE "mpich2-testsuite"

/* Define to the address where bug reports for this package should be sent. */
#define PACKAGE_BUGREPORT ""

/* Define to the full name of this package. */
#define PACKAGE_NAME "mpich2-testsuite"

/* Define to the full name and version of this package. */
#define PACKAGE_STRING "mpich2-testsuite 1.2"

/* Define to the one symbol short name of this package. */
#define PACKAGE_TARNAME "mpich2-testsuite"

/* Define to the home page for this package. */
#define PACKAGE_URL ""

/* Define to the version of this package. */
#define PACKAGE_VERSION "1.2"

/* POINTERINT_t is a pointer-sized integer */
#define POINTERINT_t long

/* The size of `int', as computed by sizeof. */
#define SIZEOF_INT 4

/* The size of `long', as computed by sizeof. */
#define SIZEOF_LONG 8

/* The size of `long long', as computed by sizeof. */
#define SIZEOF_LONG_LONG 8

/* The size of `MPI_Offset', as computed by sizeof. */
#define SIZEOF_MPI_OFFSET 8

/* The size of `short', as computed by sizeof. */
#define SIZEOF_SHORT 2

/* The size of `void *', as computed by sizeof. */
#define SIZEOF_VOID_P 8

/* Define calling convention */
#define STDCALL 

/* Define to 1 if you have the ANSI C header files. */
#define STDC_HEADERS 1

/* Define if tests with long double complex should be included */
#define USE_LONG_DOUBLE_COMPLEX 1

/* Define if only operations defined in MPI should be tested */
/* #undef USE_STRICT_MPI */

/* Version number of package */
#define VERSION "1.2"

/* Define for Solaris 2.5.1 so the uint32_t typedef from <sys/synch.h>,
   <pthread.h>, or <semaphore.h> is not used. If the typedef were allowed, the
   #define below would cause a syntax error. */
/* #undef _UINT32_T */

/* Define for Solaris 2.5.1 so the uint64_t typedef from <sys/synch.h>,
   <pthread.h>, or <semaphore.h> is not used. If the typedef were allowed, the
   #define below would cause a syntax error. */
/* #undef _UINT64_T */

/* Define for Solaris 2.5.1 so the uint8_t typedef from <sys/synch.h>,
   <pthread.h>, or <semaphore.h> is not used. If the typedef were allowed, the
   #define below would cause a syntax error. */
/* #undef _UINT8_T */

/* Define to empty if `const' does not conform to ANSI C. */
/* #undef const */

/* Define to the type of a signed integer type of width exactly 16 bits if
   such a type exists and the standard includes do not define it. */
/* #undef int16_t */

/* Define to the type of a signed integer type of width exactly 32 bits if
   such a type exists and the standard includes do not define it. */
/* #undef int32_t */

/* Define to the type of a signed integer type of width exactly 64 bits if
   such a type exists and the standard includes do not define it. */
/* #undef int64_t */

/* Define to the type of a signed integer type of width exactly 8 bits if such
   a type exists and the standard includes do not define it. */
/* #undef int8_t */

/* Define to the equivalent of the C99 'restrict' keyword, or to
   nothing if this is not supported.  Do not define if restrict is
   supported directly.  */
#define restrict __restrict
/* Work around a bug in Sun C++: it does not support _Restrict or
   __restrict__, even though the corresponding Sun C compiler ends up with
   "#define restrict _Restrict" or "#define restrict __restrict__" in the
   previous line.  Perhaps some future version of Sun C++ will work with
   restrict; if so, hopefully it defines __RESTRICT like Sun C does.  */
#if defined __SUNPRO_CC && !defined __RESTRICT
# define _Restrict
# define __restrict__
#endif

/* Define to the type of an unsigned integer type of width exactly 16 bits if
   such a type exists and the standard includes do not define it. */
/* #undef uint16_t */

/* Define to the type of an unsigned integer type of width exactly 32 bits if
   such a type exists and the standard includes do not define it. */
/* #undef uint32_t */

/* Define to the type of an unsigned integer type of width exactly 64 bits if
   such a type exists and the standard includes do not define it. */
/* #undef uint64_t */

/* Define to the type of an unsigned integer type of width exactly 8 bits if
   such a type exists and the standard includes do not define it. */
/* #undef uint8_t */

#endif
