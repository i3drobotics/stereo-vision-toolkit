#pragma once

/// Class for printing info into consol
class DebugPrintOut
{
public:
	~DebugPrintOut(void);
	static DebugPrintOut& getInstance();

	void printOut(const wchar_t *format, ...);

	void setVerbose(bool state);
	
	bool verbose;

private:	
	DebugPrintOut(void);	
		
};

