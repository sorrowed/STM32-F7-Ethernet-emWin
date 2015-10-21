#include "DIALOG.h"

#define ID_FRAMEWIN_0    (GUI_ID_USER + 0x00)
#define ID_GRAPH_0    (GUI_ID_USER + 0x01)
#define ID_TEXT_0    (GUI_ID_USER + 0x02)

static const GUI_WIDGET_CREATE_INFO _aDialogCreate[ ] =
{
	{ FRAMEWIN_CreateIndirect, "Framewin", ID_FRAMEWIN_0, 0, 0,	480, 272, 0, 0x0, 0 },
	{ GRAPH_CreateIndirect, "Graph", ID_GRAPH_0, 35, 25, 400, 200, 0, 0x0, 0 },
	{ TEXT_CreateIndirect, "CPU", ID_TEXT_0, 5,  10,  50,  20, TEXT_CF_LEFT },
};

static GRAPH_DATA_Handle data;

static void _cbDialog( WM_MESSAGE * pMsg )
{
	WM_HWIN hItem = WM_GetDialogItem( pMsg->hWin, ID_GRAPH_0 );

	switch( pMsg->MsgId )
	{
		case WM_INIT_DIALOG:
			data = GRAPH_DATA_YT_Create( GUI_RED, 400, 0, 0 );
			GRAPH_AttachData( hItem, data );
			break;
		default:
			WM_DefaultProc( pMsg );
			break;
	}
}

WM_HWIN CreateFramewin( void )
{
	return GUI_CreateDialogBox( _aDialogCreate, GUI_COUNTOF( _aDialogCreate ), _cbDialog, WM_HBKWIN, 0, 0 );
}

void AddGraphPoint( int value )
{
	GRAPH_DATA_YT_AddValue( data, value );
}

#include <stdio.h>
void DisplayCpu( WM_HWIN dlg, int value )
{
	char text[ 16 ] = { 0 };
	snprintf( text, 15, "CPU: %d%%", (int)osGetCPUUsage() );

	TEXT_Handle h = WM_GetDialogItem( dlg, ID_TEXT_0 );

	TEXT_SetText( h, text );
}
// USER START (Optionally insert additional public code)
// USER END

/*************************** End of file ****************************/
