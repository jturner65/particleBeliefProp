/*

Aalto University Game Tools license

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef DEBUG_H
#define DEBUG_H
#if defined(WIN32) || defined(WIN64)
#ifdef NOMINMAX
#include <windows.h>
#else
#define NOMINMAX
#include <windows.h>
#undef NOMINMAX
#endif
#else
#include <stdio.h>
#endif
#include <string>
#include <exception>
#include <stdarg.h>


namespace AaltoGames{
	namespace Debug
	{
		static void throwError(const char *format,...){
			char c[256];
			va_list params;
			va_start( params, format );     // params to point to the parameter list

			vsprintf_s(c, 256, format, params);
#if defined(WIN32) || defined(WIN64)
			MessageBoxA(0,c,"Exception",MB_OK);
#endif
			throw std::exception(c);
		}
		static void printf(const char *format,...)
		{
#if defined(_DEBUG) || defined(ENABLE_DEBUG_OUTPUT)
			char c[256];
			va_list params;
			va_start( params, format );     // params to point to the parameter list

			vsprintf_s(c, 256, format, params);
#if defined(WIN32) || defined(WIN64)
			OutputDebugStringA(c);
#else
			printf(c);
#endif
#endif
		}
	}
} //AaltoGames

#if defined(_DEBUG) || defined(ENABLE_DEBUG_OUTPUT)
#define AALTO_ASSERT(test, message, ...) if (!(test)) (AaltoGames::Debug::throwError("AALTO_ASSERT ( " #test " ) failed, message: " message, ##__VA_ARGS__),0)
#define AALTO_ASSERT1(test) if (!(test)) (AaltoGames::Debug::throwError("AALTO_ASSERT1 ( " #test " ) failed."),0)
#else
#define AALTO_ASSERT(x, y, ...)
#define AALTO_ASSERT1(x)
#endif

#endif //DEBUG_H
