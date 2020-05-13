#pragma once

struct IMFAttributes;

// Class for creating of Media Foundation context
class Media_Foundation
{
public:
	virtual ~Media_Foundation(void);

	static Media_Foundation& getInstance();

	bool buildListOfDevices();

private: 
    	
	Media_Foundation(void);

};

