// FontMgr.cpp : implementation file
//


#include "stdafx.h"
#include <strsafe.h>
#include "FontMgr.h"


/////////////////////////////////////////////////////////////////////////////
// CFontMgr

CFontMgr::CFontMgr()
{
	ZeroMemory(m_szFontFace,sizeof(m_szFontFace));
	m_hFont			= NULL;
	m_Style			= 0;
	m_crFont		= RGB(10,255,10);
	m_nHeight		= 40;
	m_nWeight		= 400;
	m_nOrientation	= 0;

	SetFont( L"Arial");

	SetFontUnderline(TRUE);
	SetFontStrikeout(FALSE);
	SetFontItalic(TRUE);
}

CFontMgr::~CFontMgr()
{
	DestroyFont();
}



/////////////////////////////////////////////////////////////////////////////
//
//		PUBLIC FUNTIONS
//
/////////////////////////////////////////////////////////////////////////////
void CFontMgr::SetDefaultFont()
{
	m_crFont	= RGB(255, 255, 255);
	m_nHeight	= 10;
	SetFont(L"Arial" );

	SetFontUnderline( FALSE ); 
	SetFontStrikeout( FALSE ); 
	SetFontItalic( FALSE );
}


//
//-----------------------------------------------------------------------
//  
HFONT CFontMgr::GetFontHandle(HDC hDC)
{
	if( m_hFont == NULL )
	{
		CreateFont( hDC );	//only create the font when first requested
	}

	return m_hFont;
}

//
//-----------------------------------------------------------------------
//  
//set TRUE if the font changed
BOOL CFontMgr::SetFont(LPCTSTR lpszFontFace, int nFontHeight)
{
	SetFont(lpszFontFace);
	SetHeight( nFontHeight );
	return ( m_hFont==NULL );
}

//
//-----------------------------------------------------------------------
//  
BOOL CFontMgr::SetFont(LPCTSTR lpszFontFace)
{
	if( StringCbCopy( m_szFontFace,sizeof(m_szFontFace), lpszFontFace)==S_OK )
	{
		DestroyFont();
	}
	return ( m_hFont==NULL );
}

//
//-----------------------------------------------------------------------
//  
BOOL CFontMgr::SetHeight( int nHeight )
{
	if( nHeight < 0 )	nHeight = 2;

	if( m_nHeight != nHeight )
	{
		DestroyFont();
		m_nHeight = nHeight;
	}
	return ( m_hFont==NULL );
}

//
//-----------------------------------------------------------------------
//  
BOOL CFontMgr::SetWeight( int nWeight )
{
	if( nWeight < 0 )	nWeight = 0;
	if( nWeight > 900 )	nWeight = 900;

	if( m_nWeight != nWeight )
	{
		DestroyFont();
		m_nWeight = nWeight;
	}
	return ( m_hFont==NULL );
}


//
//-----------------------------------------------------------------------
//  
BOOL CFontMgr::SetOrientation( int nOrientation )
{
	if( nOrientation < 0 )	nOrientation = 0;
	nOrientation = nOrientation % 3600;

	if( m_nOrientation != nOrientation )
	{
		DestroyFont();
		m_nOrientation = nOrientation;
	}
	return ( m_hFont==NULL );
}



/////////////////////////////////////////////////////////////////////////////
//
//		PROTECTED FUNTIONS
//
/////////////////////////////////////////////////////////////////////////////

void CFontMgr::CreateFont(HDC hDC)
{
	DestroyFont();

	int nHeight = -MulDiv(m_nHeight, GetDeviceCaps(hDC, LOGPIXELSY), 72);

	m_hFont = ::CreateFont(
		nHeight,								// int nHeight
		0,										// int nWidth
		0,										// int nEscapement
		m_nOrientation,							// int nOrientation
		m_nWeight,								// int nWeight, FW_NORMAL = 400
		(BYTE)((BOOL)(m_Style & FONT_ITALIC)),		// BYTE bItalic
		(BYTE)((BOOL)(m_Style & FONT_UNDERLINE)),	// BYTE bUnderline
		(BYTE)((BOOL)(m_Style & FONT_STRIKEOUT)),	// BYTE cStrikeOut
		ANSI_CHARSET,							// BYTE nCharSet
		OUT_DEFAULT_PRECIS,						// BYTE nOutPrecision
		CLIP_DEFAULT_PRECIS,					// BYTE nClipPrecision
		( (m_Style & FONT_ANTIALIASED) ? ANTIALIASED_QUALITY : NONANTIALIASED_QUALITY),
		//	NONANTIALIASED_QUALITY, //ANTIALIASED_QUALITY,					// BYTE nQuality, NONANTIALIASED_QUALITY,
		DEFAULT_PITCH | FF_DONTCARE,							// BYTE nPitchAndFamily, FF_ROMAN
		m_szFontFace							// LPCTSTR lpszFacename
		);
}


//
//-----------------------------------------------------------------------
//  
void CFontMgr::DestroyFont()
{ 
	if(m_hFont != NULL)
	{
		::DeleteObject(m_hFont);
		m_hFont = NULL;
	}
}

//
//-----------------------------------------------------------------------
//  
void CFontMgr::SetStyleBit( BYTE BitNum ) 
{
	if( !(m_Style & BitNum) ) 
	{
		DestroyFont(); 
		m_Style |= BitNum;
	}
}

//
//-----------------------------------------------------------------------
//  
void CFontMgr::RemoveStyleBit( BYTE BitNum ) 
{ 
	if( m_Style & BitNum ) 
	{
		DestroyFont(); 
		m_Style &= ~BitNum;
	} 
}

//
//-----------------------------------------------------------------------
//  
LPCTSTR CFontMgr::GetFontFace()
{
	return m_szFontFace;
}


//
//-----------------------------------------------------------------------
//  
