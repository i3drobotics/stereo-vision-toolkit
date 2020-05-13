#pragma once

template <class T> void SafeRelease(T **ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

template <class T> void SafeReleaseAllCount(T **ppT)
{
    if (*ppT)
    {
        ULONG e = (*ppT)->Release();

		while (e)
		{
			e = (*ppT)->Release();
		}

        *ppT = NULL;
    }
}

