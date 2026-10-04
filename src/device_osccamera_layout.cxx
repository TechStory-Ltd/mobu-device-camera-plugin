
/**	\file	device_osccamera_layout.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declarations
#include "device_osccamera_device.h"
#include "device_osccamera_layout.h"

//--- Registration define
#define CDEVICEOSCCAMERA__LAYOUT	CDevice_OSCCamera_Layout

//--- FiLMBOX implementation and registration
FBDeviceLayoutImplementation(	CDEVICEOSCCAMERA__LAYOUT	);
FBRegisterDeviceLayout		(	CDEVICEOSCCAMERA__LAYOUT,
								CDEVICEOSCCAMERA__CLASSSTR,
								"camera.png"			);	// Icon filename (default=Open Reality icon)

/************************************************
 *	FiLMBOX constructor.
 ************************************************/
bool CDevice_OSCCamera_Layout::FBCreate()
{
	// Get a handle on the device.
	mDevice = ((CDevice_OSCCamera *)(FBDevice *)Device);

	// Create/configure UI
	UICreate	();
	UIConfigure	();
	UIReset		();

	// Add device callbacks
	mDevice->OnStatusChange.Add	( this,(FBCallback)&CDevice_OSCCamera_Layout::EventDeviceStatusChange );

	return true;
}

/************************************************
 *	FiLMBOX destructor.
 ************************************************/
void CDevice_OSCCamera_Layout::FBDestroy()
{
	// Remove device callbacks
	mDevice->OnStatusChange.Remove	( this,(FBCallback)&CDevice_OSCCamera_Layout::EventDeviceStatusChange );
}


/************************************************
 *	Create the UI.
 ************************************************/
void CDevice_OSCCamera_Layout::UICreate()
{
	int lS, lH;		// space, height
	lS = 4;
	lH = 25;

	// Create regions
	AddRegion	( "TabPanel",	"TabPanel",		0,		kFBAttachLeft,		"",			1.00,
												0,		kFBAttachTop,		"",			1.00,
												0,		kFBAttachRight,		"",			1.00,
												lH,		kFBAttachNone,		NULL,		1.00 );
	AddRegion	( "MainLayout",	"MainLayout",	lS,		kFBAttachLeft,		"TabPanel",	1.00,
												lS,		kFBAttachBottom,	"TabPanel",	1.00,
												-lS,	kFBAttachRight,		"TabPanel",	1.00,
												-lS,	kFBAttachBottom,	"",			1.00 );

	// Assign regions
	SetControl	( "TabPanel",	mTabPanel		);
	SetControl	( "MainLayout",	mLayoutGeneral	);

	// Create sub layouts
	UICreateLayout0();
	UICreateLayout1();
}


/************************************************
 *	Create General layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UICreateLayout0()
{
	int lS, lH;		// space, height
	lS = 4;
	lH = 18;

	// Add regions
	mLayoutGeneral.AddRegion ( "LabelSamplingRate",	"LabelSamplingRate",
													lS,		kFBAttachLeft,		"",		1.00,
													lS,		kFBAttachTop,		"",		1.00,
													100,	kFBAttachNone,		NULL,	1.00,
													lH,		kFBAttachNone,		NULL,	1.00 );
	mLayoutGeneral.AddRegion ( "EditNumberSamplingRate",	"EditNumberSamplingRate",
													lS,		kFBAttachRight,		"LabelSamplingRate",	1.0,
													0,		kFBAttachTop,		"LabelSamplingRate",	1.0,
													100,	kFBAttachNone,		NULL,					1.0,
													0,		kFBAttachHeight,	"LabelSamplingRate",	1.0 );
	mLayoutGeneral.AddRegion ( "LabelSamplingType",	"LabelSamplingType",
													0,		kFBAttachLeft,		"LabelSamplingRate",	1.0,
													lS,		kFBAttachBottom,	"LabelSamplingRate",	1.0,
													0,		kFBAttachWidth,		"LabelSamplingRate",	1.0,
													0,		kFBAttachHeight,	"LabelSamplingRate",	1.0 );
	mLayoutGeneral.AddRegion ( "ListSamplingType",	"ListSamplingType",
													lS,		kFBAttachRight,		"LabelSamplingType",	1.0,
													0,		kFBAttachTop,		"LabelSamplingType",	1.0,
													150,	kFBAttachNone,		NULL,					1.0,
													0,		kFBAttachHeight,	"LabelSamplingType",	1.0 );
	mLayoutGeneral.AddRegion ( "ButtonAbout",	"ButtonAbout",
													0,		kFBAttachLeft,		"ListSamplingType",	1.0,
													lS,		kFBAttachBottom,	"ListSamplingType",	1.0,
													100,	kFBAttachNone,		NULL,				1.0,
													0,		kFBAttachHeight,	"ListSamplingType",	1.0 );

	// Assign regions
	mLayoutGeneral.SetControl("LabelSamplingRate",		mLabelSamplingRate		);
	mLayoutGeneral.SetControl("EditNumberSamplingRate",	mEditNumberSamplingRate	);
	mLayoutGeneral.SetControl("LabelSamplingType",		mLabelSamplingType		);
	mLayoutGeneral.SetControl("ListSamplingType",		mListSamplingType		);
	mLayoutGeneral.SetControl("ButtonAbout",			mButtonAbout			);
}


/************************************************
 *	Create the communications layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UICreateLayout1()
{
	int lS		= 4;
	int lSx		= 10;
	int lSy		= 15;

	int lW		= 90;
	int lH		= 18;
	int lHlr	= 55;
	int lWlr	= 200;

	int lSlbx	= 15;
	int lSlby	= 10;
	int lWlb	= 80;

	// Add regions (network)
	mLayoutCommunication.AddRegion( "LayoutRegionNetwork",	"LayoutRegionNetwork",
													lSx,	kFBAttachLeft,		"",						1.00,
													lSy,	kFBAttachTop,		"",						1.00,
													lWlr,	kFBAttachNone,		NULL,					1.00,
													lHlr,	kFBAttachNone,		NULL,					1.00 );
	mLayoutCommunication.AddRegion( "LabelPort",	"LabelPort",
													lSlbx,	kFBAttachLeft,		"LayoutRegionNetwork",	1.00,
													lSlby,	kFBAttachTop,		"LayoutRegionNetwork",	1.00,
													lWlb,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutCommunication.AddRegion( "EditNumberPort",	"EditNumberPort",
													lS,		kFBAttachRight,		"LabelPort",			1.00,
													0,		kFBAttachTop,		"LabelPort",			1.00,
													lW,		kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );

	// Assign regions (network)
	mLayoutCommunication.SetControl( "LabelPort",		mLabelPort		);
	mLayoutCommunication.SetControl( "EditNumberPort",	mEditNumberPort	);
}


/************************************************
 *	Configure the UI.
 ************************************************/
void CDevice_OSCCamera_Layout::UIConfigure()
{
	SetBorder ("MainLayout", kFBStandardBorder, false,true, 1, 0,90,0);

	mTabPanel.Items.SetString("General~Communication");
	mTabPanel.OnChange.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventTabPanelChange );

	UIConfigureLayout0();
	UIConfigureLayout1();
}


/************************************************
 *	Configure the general layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UIConfigureLayout0()
{
	mLabelSamplingRate.Caption = "Sampling Rate :";
	mLabelSamplingType.Caption = "Sampling Type :";

	mListSamplingType.Items.Add( "kFBHardwareTimestamp",	kFBHardwareTimestamp	);
	mListSamplingType.Items.Add( "kFBHardwareFrequency",	kFBHardwareFrequency	);
	mListSamplingType.Items.Add( "kFBAutoFrequency",		kFBAutoFrequency		);
	mListSamplingType.Items.Add( "kFBSoftwareTimestamp",	kFBSoftwareTimestamp	);

	mEditNumberSamplingRate.LargeStep = 0.0;
	mEditNumberSamplingRate.SmallStep = 0.0;
	mEditNumberSamplingRate.OnChange.Add( this, (FBCallback)&CDevice_OSCCamera_Layout::EventEditNumberSamplingRateChange );
	mListSamplingType.OnChange.Add( this, (FBCallback)&CDevice_OSCCamera_Layout::EventListSamplingTypeChange );

	mButtonAbout.OnClick.Add(this, (FBCallback)&CDevice_OSCCamera_Layout::EventButtonAboutClick);
	mButtonAbout.Caption = "About";
}


/************************************************
 *	Configure the communications layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UIConfigureLayout1()
{
	mLayoutCommunication.SetBorder( "LayoutRegionNetwork",	kFBEmbossBorder,false,true,2,1,90.0,0);

	mLabelPort.Caption = "UDP Port :";

	mEditNumberPort.Min = 1.0;
	mEditNumberPort.Max = 65535.0;
	mEditNumberPort.Precision = 0.0;
	mEditNumberPort.LargeStep = 0.0;
	mEditNumberPort.SmallStep = 0.0;
	mEditNumberPort.OnChange.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventEditNumberPortChange );
}


/************************************************
 *	Reset the UI values from the device.
 ************************************************/
void CDevice_OSCCamera_Layout::UIReset()
{
	mEditNumberPort.Value				= static_cast<double>(mDevice->Port);

	mEditNumberSamplingRate.Value		= 1.0/((FBTime)mDevice->SamplingPeriod).GetSecondDouble();
	mListSamplingType.ItemIndex			= mListSamplingType.Items.Find( mDevice->SamplingMode.AsInt() );
}


/************************************************
 *	Tab panel change callback.
 ************************************************/
void CDevice_OSCCamera_Layout::EventTabPanelChange( HISender pSender, HKEvent pEvent )
{
	switch( mTabPanel.ItemIndex )
	{
		case 0:	SetControl("MainLayout", mLayoutGeneral			);	break;
		case 1:	SetControl("MainLayout", mLayoutCommunication	);	break;
	}
}

/************************************************
 *	Network port change callback.
 ************************************************/
void CDevice_OSCCamera_Layout::EventEditNumberPortChange( HISender pSender, HKEvent pEvent )
{
	const int lPort = static_cast<int>(mEditNumberPort.Value);
	if (lPort == mDevice->Port)
	{
		return;
	}

	// the socket is bound on start, so an online device has to be restarted to listen to the new port
	const bool lOnline = mDevice->Online;
	if (lOnline)
	{
		mDevice->DeviceSendCommand( FBDevice::kOpStop );
	}

	mDevice->Port = lPort;

	if (lOnline)
	{
		mDevice->DeviceSendCommand( FBDevice::kOpStart );
	}

	UIReset();
}


/************************************************
 *	Device status change callback.
 ************************************************/
void CDevice_OSCCamera_Layout::EventDeviceStatusChange( HISender pSender, HKEvent pEvent )
{
	UIReset();
}


void CDevice_OSCCamera_Layout::EventEditNumberSamplingRateChange( HISender pSender, HKEvent pEvent )
{
	bool lOnline = mDevice->Online;
	double lVal = mEditNumberSamplingRate.Value;

	if( lVal > 0.0 )
	{
		if( lOnline )
		{
			mDevice->DeviceSendCommand( FBDevice::kOpStop );
		}

		FBTime lTime;
		lTime.SetSecondDouble( 1.0 / lVal );
		mDevice->SamplingPeriod = lTime;

		if( lOnline )
		{
			mDevice->DeviceSendCommand( FBDevice::kOpStart );
		}

		UIReset();
	}
}

void CDevice_OSCCamera_Layout::EventListSamplingTypeChange(  HISender pSender, HKEvent pEvent )
{
	mDevice->SamplingMode.SetPropertyValue((FBDeviceSamplingMode)mListSamplingType.Items.GetReferenceAt(mListSamplingType.ItemIndex));
	UIReset();
}

void CDevice_OSCCamera_Layout::EventButtonAboutClick(HISender pSender, HKEvent pEvent)
{
	FBMessageBox("OSC Camera Tracker Device Plugin", "Copyright (c) 2026 TechStory Ltd\n MIT License", "Ok");
}
