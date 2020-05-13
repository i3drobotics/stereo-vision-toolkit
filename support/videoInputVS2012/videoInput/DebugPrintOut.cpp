#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "DebugPrintOut.h"


DebugPrintOut::DebugPrintOut(void):verbose(true)
{
}

DebugPrintOut::~DebugPrintOut(void)
{
}

DebugPrintOut& DebugPrintOut::getInstance()
{
	static DebugPrintOut instance;

	return instance;
}

void DebugPrintOut::printOut(const wchar_t *format, ...)
{
	if(verbose)
	{
		int i = 0;

		wchar_t *p = NULL;

		va_list args;

		va_start(args, format);

		bool state = true;

	
		if(wcscmp(format, L"%i"))
		{
				i = va_arg (args, int);
		}
 
		if(wcscmp(format, L"%s"))
		{
				p = va_arg (args, wchar_t *);
		}
		
		wprintf(format, i,p);

		va_end (args);
	}
}

void DebugPrintOut::setVerbose(bool state)
{
	verbose = state;
}