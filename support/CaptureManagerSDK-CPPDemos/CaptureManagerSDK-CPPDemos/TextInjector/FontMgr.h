#if !defined(AFX_FONTMGR_H__E827D198_7477_11D5_A35B_525400E8DF54__INCLUDED_)
#define AFX_FONTMGR_H__E827D198_7477_11D5_A35B_525400E8DF54__INCLUDED_


#define FONT_UNDERLINE		0x01
#define FONT_ITALIC			0x02
#define FONT_STRIKEOUT		0x04
#define FONT_ANTIALIASED	0x08


/////////////////////////////////////////////////////////////////////////////
// CFontMgr 

class CFontMgr 
{
// Construction
public:
	CFontMgr();
	virtual ~CFontMgr();

protected:

	HFONT	m_hFont;		//pointer to the font created.
	BYTE	m_Style;		//style of the font

	COLORREF m_crFont;		//color of the font

	int		m_nHeight;
	int		m_nWeight;
	int		m_nOrientation;

	TCHAR	m_szFontFace[33];	//max font name size is 32 chars, so add 1 for a null terminator

	// Attributes
public:

	HFONT	GetFontHandle(HDC hDC);

	void	SetDefaultFont();

	BOOL	SetFont(LPCTSTR lpszFontFace);
	BOOL	SetFont(LPCTSTR lpszFontFace, int nFontHeight);

	BOOL	SetHeight( int nHeight );
	inline int 	GetHeight() {return m_nHeight; }


	BOOL	SetWeight( int nWeight );
	inline int GetWeight() { return m_nWeight;}
	
	BOOL	SetOrientation( int nOrientation );
	inline int GetOrientation() { return m_nOrientation;}

	inline void SetFontColor( COLORREF color ) {m_crFont=color;}
	inline COLORREF GetFontColor() { return m_crFont;}

	inline BOOL SetFontUnderline( BOOL Underline ) 
				{ SetFontStyle( Underline, FONT_UNDERLINE); return ( m_hFont==NULL );}

	inline BOOL SetFontStrikeout( BOOL StrikeOut ) 
				{ SetFontStyle( StrikeOut, FONT_STRIKEOUT); return ( m_hFont==NULL );}

	inline BOOL SetFontItalic( BOOL Italics) 
				{ SetFontStyle( Italics, FONT_ITALIC); return ( m_hFont==NULL );}

	inline BOOL SetFontAntialiased( BOOL Alias) 
				{ SetFontStyle( Alias, FONT_ANTIALIASED); return ( m_hFont==NULL );}


	inline void SetFontStyle( BOOL SetStyle, BYTE StyleFlag ) 
				{ SetStyle?(SetStyleBit(StyleFlag)):(RemoveStyleBit(StyleFlag)) ;}


	inline BOOL GetFontUnderline()
			{ return (  m_Style & FONT_UNDERLINE  ); }

	inline BOOL GetFontStrikeout( ) 
			{ return (  m_Style & FONT_STRIKEOUT  ); }

	inline BOOL GetFontItalic() 
			{ return (  m_Style &  FONT_ITALIC ); }

	inline BOOL GetFontAntialiased() 
			{ return (  m_Style &  FONT_ANTIALIASED ); }


	LPCTSTR GetFontFace();

protected:

	void	CreateFont(HDC hDC);
	void	DestroyFont(); 

		//any style change requires the font to be recreated so delete the font
		//to indicate to recreate the font
	void	SetStyleBit( BYTE BitNum );
	void	RemoveStyleBit( BYTE BitNum );

};

/////////////////////////////////////////////////////////////////////////////

#endif // !defined(AFX_FONTMGR_H__E827D198_7477_11D5_A35B_525400E8DF54__INCLUDED_)
