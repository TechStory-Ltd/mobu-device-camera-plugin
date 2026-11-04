
/**	\file	device_osccamera_layout.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declarations
#include "device_osccamera_device.h"
#include "device_osccamera_layout.h"
#include "osc_camera_version.h"

#include <algorithm>
#include <cmath>
#include <type_traits>

//--- Registration define
#define CDEVICEOSCCAMERA__LAYOUT	CDevice_OSCCamera_Layout

//--- FiLMBOX implementation and registration
FBDeviceLayoutImplementation(	CDEVICEOSCCAMERA__LAYOUT	);
FBRegisterDeviceLayout		(	CDEVICEOSCCAMERA__LAYOUT,
								CDEVICEOSCCAMERA__CLASSSTR,
								"camera.png"			);	// Icon filename (default=Open Reality icon)

namespace
{
	// the tabs of the layout
	enum ETab
	{
		kTabGeneral = 0, kTabCommunication, kTabCalibration, kTabMarkers
	};

	const int kPropertyRowS			= 4;
	const int kPropertyRowH			= 18;
	const int kPropertyCaptionW		= 140;	//!< the label with the name of the property, the value starts after it
}


/************************************************
 *	A row of a device property under the region \p pAttachTo, lined up with its left side: a label with the name
 *	of the property and the editor of the value next to it. With \p pTopAttach kFBAttachTop the row is put inside
 *	the region instead ("" - the layout itself). The region of the row is \p pRegion, the next row attaches to it.
 *	The editor follows the property, a change from anywhere (the property editor, a script) is shown.
 ************************************************/
template <class TEdit>
void CDevice_OSCCamera_Layout::AddPropertyRow( FBLayout& pLayout, const char* pRegion, const char* pAttachTo, TEdit& pEdit, FBProperty* pProperty, int pWidth,
												int pLeft, FBAttachType pTopAttach, int pTop )
{
	// the editor does not show the caption of the property, it gets a label of its own
	const char* lValueRegion = KeepRegionName( std::string(pRegion) + "Value" );

	pLayout.AddRegion( pRegion, pRegion,	pLeft,							kFBAttachLeft,		pAttachTo,	1.00,
											pTop,							pTopAttach,			pAttachTo,	1.00,
											kPropertyCaptionW - kPropertyRowS,	kFBAttachNone,	NULL,		1.00,
											kPropertyRowH,					kFBAttachNone,		NULL,		1.00 );
	pLayout.AddRegion( lValueRegion, lValueRegion,
											kPropertyRowS,					kFBAttachRight,		pRegion,	1.00,
											0,								kFBAttachTop,		pRegion,	1.00,
											pWidth - kPropertyCaptionW,		kFBAttachNone,		NULL,		1.00,
											kPropertyRowH,					kFBAttachNone,		NULL,		1.00 );

	FBLabel& lLabel = NextPropertyLabel();
	lLabel.Caption = (std::string(pProperty->GetName()) + " :").c_str();

	pLayout.SetControl( pRegion, lLabel );
	pLayout.SetControl( lValueRegion, pEdit );

	pEdit.Property = pProperty;
	if constexpr (std::is_same_v<TEdit, FBEditProperty>)
	{
		pEdit.CaptionSize = 0;
	}
}


/************************************************
 *	A hint on the right of the property row \p pRegion: a short line that stays in view, and a tooltip
 *	with the details on the hint and on the editor of the value.
 ************************************************/
template <class TEdit>
void CDevice_OSCCamera_Layout::AddPropertyHint( FBLayout& pLayout, const char* pRegion, TEdit& pEdit, const char* pText, const char* pTooltip )
{
	const char* lValueRegion = KeepRegionName( std::string(pRegion) + "Value" );
	const char* lHintRegion = KeepRegionName( std::string(pRegion) + "Hint" );

	pLayout.AddRegion( lHintRegion, lHintRegion,
											2*kPropertyRowS,	kFBAttachRight,		lValueRegion,	1.00,
											0,					kFBAttachTop,		lValueRegion,	1.00,
											-kPropertyRowS,		kFBAttachRight,		"",						1.00,
											kPropertyRowH,		kFBAttachNone,		NULL,					1.00 );

	FBLabel& lHint = NextPropertyLabel();
	lHint.Caption = pText;
	lHint.Hint = pTooltip;
	pLayout.SetControl( lHintRegion, lHint );

	pEdit.Hint = pTooltip;
}


/************************************************
 *	A row of a property that is one of a few choices, laid out like AddPropertyRow. The list is filled and its
 *	change handled by the caller.
 ************************************************/
void CDevice_OSCCamera_Layout::AddListRow( FBLayout& pLayout, const char* pRegion, const char* pAttachTo, FBList& pList, FBProperty* pProperty, int pWidth )
{
	const char* lValueRegion = KeepRegionName( std::string(pRegion) + "Value" );

	pLayout.AddRegion( pRegion, pRegion,	0,								kFBAttachLeft,		pAttachTo,	1.00,
											kPropertyRowS,					kFBAttachBottom,	pAttachTo,	1.00,
											kPropertyCaptionW - kPropertyRowS,	kFBAttachNone,	NULL,		1.00,
											kPropertyRowH,					kFBAttachNone,		NULL,		1.00 );
	pLayout.AddRegion( lValueRegion, lValueRegion,
											kPropertyRowS,					kFBAttachRight,		pRegion,	1.00,
											0,								kFBAttachTop,		pRegion,	1.00,
											pWidth - kPropertyCaptionW,		kFBAttachNone,		NULL,		1.00,
											kPropertyRowH,					kFBAttachNone,		NULL,		1.00 );

	FBLabel& lLabel = NextPropertyLabel();
	lLabel.Caption = (std::string(pProperty->GetName()) + " :").c_str();

	pLayout.SetControl( pRegion, lLabel );
	pLayout.SetControl( lValueRegion, pList );
}


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
	FBSystem::TheOne().OnUIIdle.Add( this,(FBCallback)&CDevice_OSCCamera_Layout::EventUIIdle );

	return true;
}

/************************************************
 *	FiLMBOX destructor.
 ************************************************/
void CDevice_OSCCamera_Layout::FBDestroy()
{
	// Remove device callbacks
	FBSystem::TheOne().OnUIIdle.Remove( this,(FBCallback)&CDevice_OSCCamera_Layout::EventUIIdle );
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
	UICreateLayout2();
	UICreateLayout3();
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
	// the values are lined up with the values of the property editors below
	mLayoutGeneral.AddRegion ( "LabelSamplingRate",	"LabelSamplingRate",
													lS,		kFBAttachLeft,		"",		1.00,
													lS,		kFBAttachTop,		"",		1.00,
													kPropertyCaptionW - lS,	kFBAttachNone,	NULL,	1.00,
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

	// the rows of the properties are lined up with the labels above, a hint next to each one says what it is for
	const int lW = 254;
	AddPropertyRow( mLayoutGeneral, "EditCameraId",		"LabelSamplingType",	mEditCameraId,		&mDevice->CameraId,		lW );
	AddListRow( mLayoutGeneral, "ListPlaybackMode",			"EditCameraId",			mListPlaybackMode,	&mDevice->PlaybackMode,		lW );
	AddPropertyRow( mLayoutGeneral, "EditPlaybackDelay",	"ListPlaybackMode",		mEditPlaybackDelay,	&mDevice->PlaybackDelay,	lW );
	AddPropertyRow( mLayoutGeneral, "EditMaxAutoDelay",		"EditPlaybackDelay",	mEditMaxAutoDelay,	&mDevice->MaxAutoDelay,		lW );
	AddListRow( mLayoutGeneral, "ListResponsiveness",		"EditMaxAutoDelay",		mListResponsiveness,&mDevice->Responsiveness,	lW );
	AddPropertyRow( mLayoutGeneral, "EditApplySensor",		"ListResponsiveness",	mEditApplySensor,	&mDevice->ApplySensor,	lW );

	AddPropertyHint( mLayoutGeneral, "EditCameraId", mEditCameraId,
		"-1 - any camera, otherwise only this Camera ID of the app",
		"Only the poses of this camera id (set in the app) are accepted.\n"
		"-1 takes the first camera that streams to the port, until it falls silent.\n"
		"Set it when several phones stream to the same port." );
	AddPropertyHint( mLayoutGeneral, "EditPlaybackDelay", mEditPlaybackDelay,
		"Fixed: how far behind the stream; Auto: the least delay",
		"How far behind the stream the live camera is shown, to interpolate between the samples around that time.\n"
		"With Fixed, 0 shows the newest sample at once, pushed on by how late it arrived (a guess); more evens out the jitter of Wi-Fi\n"
		"at the cost of that much latency. With Auto it is the lowest the delay may go.\n"
		"Only the live view: a recorded take keys every sample at the time it was taken." );
	AddPropertyHint( mLayoutGeneral, "EditMaxAutoDelay", mEditMaxAutoDelay,
		"the most delay Auto may add",
		"Auto follows the jitter of the network (the late packets of the last seconds) and never goes above this.\n"
		"It rises quickly when the network gets worse and falls slowly when it is calm again." );
	AddPropertyHint( mLayoutGeneral, "EditApplySensor", mEditApplySensor,
		"set the film back of the camera to the sensor of the phone",
		"The focal length gives the same field of view as in the app only on the same film back.\n"
		"Turn it off to keep the film back set on the camera." );

	mLayoutGeneral.AddRegion ( "ButtonAbout",	"ButtonAbout",
													0,		kFBAttachLeft,		"EditApplySensor",	1.0,
													3*lS,	kFBAttachBottom,	"EditApplySensor",	1.0,
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
	int lHlr	= 132;		// 5 rows
	int lHsr	= 154;		// 6 rows
	int lWlr	= 360;
	int lWrow	= 320;		// the property rows

	int lSlbx	= 15;
	int lSlby	= 10;
	int lWlb	= kPropertyCaptionW - lS;	// the value of the port is lined up with the values of the properties

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

	AddPropertyRow( mLayoutCommunication, "EditTimeout",		"LabelPort",		mEditTimeout,		&mDevice->Timeout,		lWrow );
	AddPropertyRow( mLayoutCommunication, "EditLogPackets",		"EditTimeout",		mEditLogPackets,	&mDevice->LogPackets,	lWrow );
	AddPropertyRow( mLayoutCommunication, "EditRemoteControl",	"EditLogPackets",	mEditRemoteControl,	&mDevice->RemoteControl,	lWrow );
	AddPropertyRow( mLayoutCommunication, "EditLastCommand",	"EditRemoteControl",mEditLastCommand,	&mDevice->LastCommand,	lWrow );

	// the stream as seen by the device, read only
	mLayoutCommunication.AddRegion( "LayoutRegionStream",	"LayoutRegionStream",
													0,		kFBAttachLeft,		"LayoutRegionNetwork",	1.00,
													lSy,	kFBAttachBottom,	"LayoutRegionNetwork",	1.00,
													lWlr,	kFBAttachNone,		NULL,					1.00,
													lHsr,	kFBAttachNone,		NULL,					1.00 );

	AddPropertyRow( mLayoutCommunication, "EditPhoneAddress",	"LayoutRegionStream",	mEditPhoneAddress,	&mDevice->PhoneAddress,	lWrow, lSlbx, kFBAttachTop, lSlby );
	AddPropertyRow( mLayoutCommunication, "EditJitter",			"EditPhoneAddress",		mEditJitter,		&mDevice->Jitter,		lWrow );
	AddPropertyRow( mLayoutCommunication, "EditDelayInUse",		"EditJitter",			mEditDelayInUse,	&mDevice->DelayInUse,	lWrow );
	AddPropertyRow( mLayoutCommunication, "EditLostPackets",	"EditDelayInUse",		mEditLostPackets,	&mDevice->LostPackets,	lWrow );
	AddPropertyRow( mLayoutCommunication, "EditSensorSize",		"EditLostPackets",		mEditSensorSize,	&mDevice->SensorSize,	lWrow );
	AddPropertyRow( mLayoutCommunication, "EditStreamUnits",	"EditSensorSize",		mEditStreamUnits,	&mDevice->StreamUnits,	lWrow );
}


/************************************************
 *	Configure the UI.
 ************************************************/
void CDevice_OSCCamera_Layout::UIConfigure()
{
	SetBorder ("MainLayout", kFBStandardBorder, false,true, 1, 0,90,0);

	mTabPanel.Items.SetString("General~Communication~Calibration~Markers");
	mTabPanel.OnChange.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventTabPanelChange );

	UIConfigureLayout0();
	UIConfigureLayout1();
	UIConfigureLayout2();
	UIConfigureLayout3();
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

	mListPlaybackMode.Items.Add( "Auto - follows the jitter",	static_cast<int>(OSCCamera::Playback::Mode::Auto) );
	mListPlaybackMode.Items.Add( "Fixed delay",				static_cast<int>(OSCCamera::Playback::Mode::Fixed) );
	mListPlaybackMode.Items.Add( "Newest sample",			static_cast<int>(OSCCamera::Playback::Mode::Newest) );
	mListPlaybackMode.OnChange.Add( this, (FBCallback)&CDevice_OSCCamera_Layout::EventListPlaybackModeChange );
	mListResponsiveness.Items.Add( "Responsive",			static_cast<int>(OSCCamera::Playback::Responsiveness::Responsive) );
	mListResponsiveness.Items.Add( "Balanced",				static_cast<int>(OSCCamera::Playback::Responsiveness::Balanced) );
	mListResponsiveness.Items.Add( "Smooth",				static_cast<int>(OSCCamera::Playback::Responsiveness::Smooth) );
	mListResponsiveness.OnChange.Add( this, (FBCallback)&CDevice_OSCCamera_Layout::EventListResponsivenessChange );

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
	mLayoutCommunication.SetBorder( "LayoutRegionStream",	kFBEmbossBorder,false,true,2,1,90.0,0);

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
	UIResetPlayback();

	mButtonMarkerLink.State				= mDevice->MarkerLink ? 1 : 0;
	UIResetRoles();
	mEditNumberLinkPort.Value			= static_cast<double>(mDevice->LinkPort);
}


/************************************************
 *	Tab panel change callback.
 ************************************************/
void CDevice_OSCCamera_Layout::EventTabPanelChange( HISender pSender, HKEvent pEvent )
{
	switch( mTabPanel.ItemIndex )
	{
		case kTabGeneral:		SetControl("MainLayout", mLayoutGeneral			);	break;
		case kTabCommunication:	SetControl("MainLayout", mLayoutCommunication	);	break;
		case kTabCalibration:	SetControl("MainLayout", mLayoutCalibration		);	break;
		case kTabMarkers:		SetControl("MainLayout", mLayoutMarkers			);	UIResetMarkers();	break;
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

/************************************************
 *	The lists of the live camera follow the properties, a change from anywhere (the property editor, a script) is shown.
 ************************************************/
void CDevice_OSCCamera_Layout::UIResetPlayback()
{
	const int lMode = mListPlaybackMode.Items.Find( static_cast<int>(mDevice->PlaybackMode) );
	if (lMode >= 0 && mListPlaybackMode.ItemIndex != lMode)
	{
		mListPlaybackMode.ItemIndex = lMode;
	}
	const int lResponsiveness = mListResponsiveness.Items.Find( static_cast<int>(mDevice->Responsiveness) );
	if (lResponsiveness >= 0 && mListResponsiveness.ItemIndex != lResponsiveness)
	{
		mListResponsiveness.ItemIndex = lResponsiveness;
	}
}

void CDevice_OSCCamera_Layout::EventListPlaybackModeChange( HISender pSender, HKEvent pEvent )
{
	if (mListPlaybackMode.ItemIndex >= 0)
	{
		mDevice->PlaybackMode = mListPlaybackMode.Items.GetReferenceAt(mListPlaybackMode.ItemIndex);
	}
}

void CDevice_OSCCamera_Layout::EventListResponsivenessChange( HISender pSender, HKEvent pEvent )
{
	if (mListResponsiveness.ItemIndex >= 0)
	{
		mDevice->Responsiveness = mListResponsiveness.Items.GetReferenceAt(mListResponsiveness.ItemIndex);
	}
}

void CDevice_OSCCamera_Layout::EventListSamplingTypeChange(  HISender pSender, HKEvent pEvent )
{
	mDevice->SamplingMode.SetPropertyValue((FBDeviceSamplingMode)mListSamplingType.Items.GetReferenceAt(mListSamplingType.ItemIndex));
	UIReset();
}

void CDevice_OSCCamera_Layout::EventButtonAboutClick(HISender pSender, HKEvent pEvent)
{
	char lMessage[160];
	snprintf(lMessage, sizeof(lMessage), "Version %s\nCopyright (c) 2026 TechStory Ltd\n MIT License", OSC_CAMERA_VERSION_STRING);
	FBMessageBox("OSC Camera Tracker Device Plugin", lMessage, "Ok");
}


/************************************************
 *	Create the calibration layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UICreateLayout3()
{
	const int lS = 4;
	const int lW = 340;

	// the stream to the scene
	AddPropertyRow( mLayoutCalibration, "EditSpaceScale",		"",					mEditSpaceScale,		&mDevice->SpaceScale,		lW, lS, kFBAttachTop, lS );
	AddPropertyRow( mLayoutCalibration, "EditFocusAngleScale",	"EditSpaceScale",		mEditFocusAngleScale,	&mDevice->FocusAngleScale,	lW );

	// the lens channels that are not calibrated in the app
	mLayoutCalibration.AddRegion( "LabelLensRanges",	"LabelLensRanges",
													0,		kFBAttachLeft,		"EditFocusAngleScale",	1.00,
													3*lS,	kFBAttachBottom,	"EditFocusAngleScale",	1.00,
													-lS,	kFBAttachRight,		"",						1.00,
													kPropertyRowH,	kFBAttachNone,	NULL,				1.00 );
	mLayoutCalibration.SetControl( "LabelLensRanges",	mLabelLensRanges );

	AddPropertyRow( mLayoutCalibration, "EditZoomRawRange",		"LabelLensRanges",		mEditZoomRawRange,		&mDevice->ZoomRawRange,		lW );
	AddPropertyRow( mLayoutCalibration, "EditZoomOutputRange",	"EditZoomRawRange",		mEditZoomOutputRange,	&mDevice->ZoomOutputRange,	lW );
	AddPropertyRow( mLayoutCalibration, "EditFocusRawRange",	"EditZoomOutputRange",	mEditFocusRawRange,		&mDevice->FocusRawRange,		lW, 0, kFBAttachBottom, 3*lS );
	AddPropertyRow( mLayoutCalibration, "EditFocusOutputRange",	"EditFocusRawRange",	mEditFocusOutputRange,	&mDevice->FocusOutputRange,	lW );
	AddPropertyRow( mLayoutCalibration, "EditIrisRawRange",		"EditFocusOutputRange",	mEditIrisRawRange,		&mDevice->IrisRawRange,		lW, 0, kFBAttachBottom, 3*lS );
	AddPropertyRow( mLayoutCalibration, "EditIrisOutputRange",	"EditIrisRawRange",		mEditIrisOutputRange,	&mDevice->IrisOutputRange,	lW );

	mLayoutCalibration.AddRegion( "ButtonCalibrationReset",	"ButtonCalibrationReset",
													0,		kFBAttachLeft,		"EditIrisOutputRange",	1.00,
													3*lS,	kFBAttachBottom,	"EditIrisOutputRange",	1.00,
													120,	kFBAttachNone,		NULL,					1.00,
													22,		kFBAttachNone,		NULL,					1.00 );
	mLayoutCalibration.SetControl( "ButtonCalibrationReset",	mButtonCalibrationReset );
}


/************************************************
 *	Configure the calibration layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UIConfigureLayout3()
{
	mLabelLensRanges.Caption = "Lens ranges, used for the channels that are not calibrated in the app (raw encoder value to output):";

	mButtonCalibrationReset.Caption = "Reset to Defaults";
	mButtonCalibrationReset.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonCalibrationResetClick );
}


void CDevice_OSCCamera_Layout::EventButtonCalibrationResetClick( HISender pSender, HKEvent pEvent )
{
	if (FBMessageBox( "Calibration", "Set the scales, the signs and the lens ranges back to the defaults?", "Reset", "Cancel" ) == 1)
	{
		mDevice->ResetCalibration();
	}
}


/************************************************
 *	Create the markers layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UICreateLayout2()
{
	const int lS = 6;
	const int lH = 22;

	mLayoutMarkers.AddRegion( "ButtonMarkerLink",	"ButtonMarkerLink",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachTop,		"",						1.00,
													210,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "LabelLinkPort",		"LabelLinkPort",
													lS,		kFBAttachRight,		"ButtonMarkerLink",		1.00,
													0,		kFBAttachTop,		"ButtonMarkerLink",		1.00,
													70,		kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "EditNumberLinkPort",	"EditNumberLinkPort",
													lS,		kFBAttachRight,		"LabelLinkPort",		1.00,
													0,		kFBAttachTop,		"LabelLinkPort",		1.00,
													80,		kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "LabelMarkerStatus",	"LabelMarkerStatus",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachBottom,	"ButtonMarkerLink",		1.00,
													-lS,	kFBAttachRight,		"",						1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	// the roles on the phone: the Role of each marker is in the table, these are for all of them
	mLayoutMarkers.AddRegion( "ButtonRefFollowHeading",	"ButtonRefFollowHeading",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachBottom,	"LabelMarkerStatus",	1.00,
													150,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "ButtonSetLookAt",	"ButtonSetLookAt",
													lS,		kFBAttachRight,		"ButtonRefFollowHeading",	1.00,
													0,		kFBAttachTop,		"ButtonRefFollowHeading",	1.00,
													95,		kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "EditNumberLookAt",	"EditNumberLookAt",
													0,		kFBAttachRight,		"ButtonSetLookAt",		1.00,
													0,		kFBAttachTop,		"ButtonSetLookAt",		1.00,
													55,		kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "ButtonMirrorPhone",	"ButtonMirrorPhone",
													3*lS,	kFBAttachRight,		"EditNumberLookAt",		1.00,
													0,		kFBAttachTop,		"EditNumberLookAt",		1.00,
													130,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "ButtonSendRoles",	"ButtonSendRoles",
													lS,		kFBAttachRight,		"ButtonMirrorPhone",	1.00,
													0,		kFBAttachTop,		"ButtonMirrorPhone",	1.00,
													150,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "LabelPhoneRoles",	"LabelPhoneRoles",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachBottom,	"ButtonRefFollowHeading",	1.00,
													-lS,	kFBAttachRight,		"",						1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );

	// the camera relative to its Ref: the pose the camera follows when the phone has a Ref
	mLayoutMarkers.AddRegion( "ButtonReferenceLocal",	"ButtonReferenceLocal",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachBottom,	"LabelPhoneRoles",		1.00,
													170,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "LabelCameraSource",	"LabelCameraSource",
													lS,		kFBAttachRight,		"ButtonReferenceLocal",	1.00,
													0,		kFBAttachTop,		"ButtonReferenceLocal",	1.00,
													-lS,	kFBAttachRight,		"",						1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );

	mLayoutMarkers.AddRegion( "ButtonMarkerAdd",	"ButtonMarkerAdd",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachBottom,	"ButtonReferenceLocal",	1.00,
													150,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "ButtonMarkerRemove",	"ButtonMarkerRemove",
													lS,		kFBAttachRight,		"ButtonMarkerAdd",		1.00,
													0,		kFBAttachTop,		"ButtonMarkerAdd",		1.00,
													150,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "ButtonMarkerMeasure","ButtonMarkerMeasure",
													lS,		kFBAttachRight,		"ButtonMarkerRemove",	1.00,
													0,		kFBAttachTop,		"ButtonMarkerRemove",	1.00,
													150,	kFBAttachNone,		NULL,					1.00,
													lH,		kFBAttachNone,		NULL,					1.00 );
	mLayoutMarkers.AddRegion( "SpreadMarkers",		"SpreadMarkers",
													lS,		kFBAttachLeft,		"",						1.00,
													lS,		kFBAttachBottom,	"ButtonMarkerAdd",		1.00,
													-lS,	kFBAttachRight,		"",						1.00,
													-lS,	kFBAttachBottom,	"",						1.00 );

	mLayoutMarkers.SetControl( "ButtonMarkerLink",		mButtonMarkerLink		);
	mLayoutMarkers.SetControl( "LabelLinkPort",			mLabelLinkPort			);
	mLayoutMarkers.SetControl( "EditNumberLinkPort",	mEditNumberLinkPort		);
	mLayoutMarkers.SetControl( "LabelMarkerStatus",		mLabelMarkerStatus		);
	mLayoutMarkers.SetControl( "ButtonMarkerAdd",		mButtonMarkerAdd		);
	mLayoutMarkers.SetControl( "ButtonMarkerRemove",	mButtonMarkerRemove		);
	mLayoutMarkers.SetControl( "ButtonMarkerMeasure",	mButtonMarkerMeasure	);
	mLayoutMarkers.SetControl( "ButtonRefFollowHeading",	mButtonRefFollowHeading	);
	mLayoutMarkers.SetControl( "ButtonSetLookAt",		mButtonSetLookAt		);
	mLayoutMarkers.SetControl( "EditNumberLookAt",		mEditNumberLookAt		);
	mLayoutMarkers.SetControl( "ButtonMirrorPhone",		mButtonMirrorPhone		);
	mLayoutMarkers.SetControl( "ButtonSendRoles",		mButtonSendRoles		);
	mLayoutMarkers.SetControl( "LabelPhoneRoles",		mLabelPhoneRoles		);
	mLayoutMarkers.SetControl( "ButtonReferenceLocal",	mButtonReferenceLocal	);
	mLayoutMarkers.SetControl( "LabelCameraSource",		mLabelCameraSource		);
	mLayoutMarkers.SetControl( "SpreadMarkers",			mSpreadMarkers			);
}


/************************************************
 *	Configure the markers layout.
 ************************************************/
void CDevice_OSCCamera_Layout::UIConfigureLayout2()
{
	mButtonMarkerLink.Style = kFBCheckbox;
	mButtonMarkerLink.Caption = "Send markers to the phone";
	mButtonMarkerLink.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonMarkerLinkClick );

	mLabelLinkPort.Caption = "Link port :";
	mEditNumberLinkPort.Min = 1.0;
	mEditNumberLinkPort.Max = 65535.0;
	mEditNumberLinkPort.Precision = 0.0;
	mEditNumberLinkPort.LargeStep = 0.0;
	mEditNumberLinkPort.SmallStep = 0.0;
	mEditNumberLinkPort.OnChange.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventEditNumberLinkPortChange );

	mLabelMarkerStatus.Caption = "";

	mButtonMarkerAdd.Caption = "Add Selected Models";
	mButtonMarkerAdd.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonMarkerAddClick );
	mButtonMarkerRemove.Caption = "Remove Row";
	mButtonMarkerRemove.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonMarkerRemoveClick );
	mButtonMarkerMeasure.Caption = "Size From Model";
	mButtonMarkerMeasure.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonMarkerMeasureClick );

	mSpreadMarkers.OnCellChange.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventSpreadMarkersCellChange );

	mButtonRefFollowHeading.Style = kFBCheckbox;
	mButtonRefFollowHeading.Caption = "Ref follows heading";
	mButtonRefFollowHeading.Hint = "What the Ref marker passes on to the phone's camera.\n"
		"Off: the camera moves with the Ref but keeps its own heading (a reference offset).\n"
		"On: the camera moves and turns with the Ref (a camera on a vehicle).";
	mButtonSetLookAt.Style = kFBCheckbox;
	mButtonSetLookAt.Caption = "Set look-at %";
	mButtonSetLookAt.Hint = "Set how far the phone turns the camera toward the Aim marker. Off: the operator chooses on the phone.";
	mEditNumberLookAt.Min = 0.0;
	mEditNumberLookAt.Max = 100.0;
	mEditNumberLookAt.Precision = 0.0;
	mEditNumberLookAt.LargeStep = 0.0;
	mEditNumberLookAt.SmallStep = 0.0;
	mEditNumberLookAt.Hint = "How far the phone turns the camera toward the Aim marker, used with Set look-at on.";
	mButtonMirrorPhone.Style = kFBCheckbox;
	mButtonMirrorPhone.Caption = "Mirror the phone";
	mButtonMirrorPhone.Hint = "When the operator picks a role on the phone, take it into these markers too, so both ends agree.\n"
		"Off: the device keeps its roles and shows what the phone has instead.";
	mButtonSendRoles.Caption = "Send Roles to Phone";
	mButtonSendRoles.Hint = "Give every role on the phone back to the ones set here, including those the operator changed on the phone.";
	mLabelPhoneRoles.Caption = "";

	mButtonReferenceLocal.Style = kFBCheckbox;
	mButtonReferenceLocal.Caption = "Reference-local camera";
	mButtonReferenceLocal.Hint = "When the phone's camera is relative to its Ref marker, place it by that marker where it is in the scene now.\n"
		"The Ref has to be one of the markers of the table. Off, or when it is not: the world pose of the stream is used.\n"
		"Recorded keys are always the world poses of the phone.";
	mLabelCameraSource.Caption = "";

	mButtonReferenceLocal.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonRoleOptionClick );
	mButtonRefFollowHeading.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonRoleOptionClick );
	mButtonSetLookAt.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonRoleOptionClick );
	mButtonMirrorPhone.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonRoleOptionClick );
	mEditNumberLookAt.OnChange.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventEditNumberLookAtChange );
	mButtonSendRoles.OnClick.Add( this, (FBCallback) &CDevice_OSCCamera_Layout::EventButtonSendRolesClick );
}


/************************************************
 *	The role options from the device: they change with the property editor, a script, or the phone mirrored.
 ************************************************/
void CDevice_OSCCamera_Layout::UIResetRoles()
{
	const int lRefFollowHeading = mDevice->RefFollowHeading ? 1 : 0;
	const int lSetLookAt = mDevice->SetLookAt ? 1 : 0;
	const int lMirrorPhone = mDevice->MirrorPhone ? 1 : 0;
	const int lReferenceLocal = mDevice->ReferenceLocal ? 1 : 0;
	const double lLookAt = static_cast<double>(static_cast<int>(mDevice->LookAt));

	if (mButtonRefFollowHeading.State != lRefFollowHeading)	{ mButtonRefFollowHeading.State = lRefFollowHeading; }
	if (mButtonSetLookAt.State != lSetLookAt)				{ mButtonSetLookAt.State = lSetLookAt; }
	if (mButtonMirrorPhone.State != lMirrorPhone)			{ mButtonMirrorPhone.State = lMirrorPhone; }
	if (mButtonReferenceLocal.State != lReferenceLocal)		{ mButtonReferenceLocal.State = lReferenceLocal; }
	if (mEditNumberLookAt.Value != lLookAt)					{ mEditNumberLookAt.Value = lLookAt; }
	if (static_cast<bool>(mEditNumberLookAt.Enabled) != (lSetLookAt != 0))	{ mEditNumberLookAt.Enabled = (lSetLookAt != 0); }
}


void CDevice_OSCCamera_Layout::EventButtonRoleOptionClick( HISender pSender, HKEvent pEvent )
{
	mDevice->RefFollowHeading = (mButtonRefFollowHeading.State != 0);
	mDevice->SetLookAt = (mButtonSetLookAt.State != 0);
	mDevice->MirrorPhone = (mButtonMirrorPhone.State != 0);
	mDevice->ReferenceLocal = (mButtonReferenceLocal.State != 0);
	UIResetRoles();
}


void CDevice_OSCCamera_Layout::EventEditNumberLookAtChange( HISender pSender, HKEvent pEvent )
{
	const int lLookAt = std::min( std::max( static_cast<int>( std::lround( static_cast<double>(mEditNumberLookAt.Value) ) ), 0 ), 100 );
	if (lLookAt != mDevice->LookAt)
	{
		mDevice->LookAt = lLookAt;
	}
}


void CDevice_OSCCamera_Layout::EventButtonSendRolesClick( HISender pSender, HKEvent pEvent )
{
	if (!mDevice->MarkerLink || !mDevice->Online)
	{
		FBMessageBox( "Markers", "The roles are sent while the device is online and the markers are sent to the phone.", "Ok" );
		return;
	}
	mDevice->SendRolesAgain();
}


namespace
{
	// the columns of the table of the markers
	enum EMarkerColumn
	{
		kColModel = 0, kColName, kColOn, kColFocus, kColAim, kColRef, kColShape, kColColor, kColWidth, kColDepth, kColHeight, kColOrigin,
		kColAnchor, kColCount
	};

	const char* kShapeItems = "Point~Box~Sphere~Capsule";
	const char* kColorItems = "Orange~Blue~Green~Purple~Yellow~Red~White~Teal";
	const char* kOriginItems = "At base~In the centre";
	const char* kAnchorItems = "Base~Centre~Top";

	// the role a column switches on and off, kRoleNone for the other columns
	int RoleOfColumn( int pColumn )
	{
		return (pColumn == kColFocus) ? OSCCamera::Marker::kRoleFocus : (pColumn == kColAim) ? OSCCamera::Marker::kRoleAim
			: (pColumn == kColRef) ? OSCCamera::Marker::kRoleRef : OSCCamera::Marker::kRoleNone;
	}
}


/************************************************
 *	Rebuild the table of the markers: the columns, a row for every entry of the list.
 ************************************************/
void CDevice_OSCCamera_Layout::UIResetMarkers()
{
	mFillingMarkers = true;

	mSpreadMarkers.Clear();

	struct Column { const char* caption; int width; FBCellStyle style; bool readOnly; };
	const Column lColumns[kColCount] = {
		{ "Model",				110,	kFBCellStyleString,			true	},
		{ "Name on phone",		110,	kFBCellStyleString,			false	},
		{ "On",					34,		kFBCellStyle2StatesButton,	false	},
		{ "Focus",				44,		kFBCellStyle2StatesButton,	false	},
		{ "Aim",				40,		kFBCellStyle2StatesButton,	false	},
		{ "Ref",				40,		kFBCellStyle2StatesButton,	false	},
		{ "Shape",				70,		kFBCellStyleMenu,			false	},
		{ "Color",				70,		kFBCellStyleMenu,			false	},
		{ "Width (m)",			62,		kFBCellStyleDouble,			false	},
		{ "Depth (m)",			62,		kFBCellStyleDouble,			false	},
		{ "Height (m)",			62,		kFBCellStyleDouble,			false	},
		{ "Model origin",		92,		kFBCellStyleMenu,			false	},
		{ "Focus / aim at",		80,		kFBCellStyleMenu,			false	},
	};

	for (int i = 0; i < kColCount; ++i)
	{
		mSpreadMarkers.ColumnAdd( lColumns[i].caption );
		mSpreadMarkers.GetColumn( i ).Width = lColumns[i].width;
		mSpreadMarkers.GetColumn( i ).Style = lColumns[i].style;
		mSpreadMarkers.GetColumn( i ).ReadOnly = lColumns[i].readOnly;
	}

	for (int i = 0, lCount = mDevice->GetMarkerCount(); i < lCount; ++i)
	{
		FBModel* lModel = mDevice->GetMarkerModel( i );
		mSpreadMarkers.RowAdd( lModel != nullptr ? (const char*)lModel->Name : "?", i );
	}

	mFillingMarkers = false;
	UIFillMarkers();
}


/************************************************
 *	Set the cells of the table from the settings of the device.
 ************************************************/
void CDevice_OSCCamera_Layout::UIFillMarkers()
{
	mFillingMarkers = true;

	const std::vector<OSCCamera::Marker::Settings> lRows = mDevice->GetMarkerSettings();

	// a menu cell takes its items as a string and its choice as an index
	auto lSetMenu = [this](int pRow, int pColumn, const char* pItems, int pIndex)
	{
		mSpreadMarkers.SetCell( pRow, pColumn, pItems );
		mSpreadMarkers.SetCell( pRow, pColumn, pIndex );
	};

	for (int i = 0, lCount = mDevice->GetMarkerCount(); i < lCount && i < static_cast<int>(lRows.size()); ++i)
	{
		const OSCCamera::Marker::Settings& lRow = lRows[i];
		FBModel* lModel = mDevice->GetMarkerModel( i );

		mSpreadMarkers.SetCell( i, kColModel, lModel != nullptr ? (const char*)lModel->Name : "(not a model)" );
		mSpreadMarkers.SetCell( i, kColName, lRow.name.c_str() );
		mSpreadMarkers.SetCell( i, kColOn, lRow.visible ? 1 : 0 );
		lSetMenu( i, kColShape, kShapeItems, lRow.shape );
		lSetMenu( i, kColColor, kColorItems, lRow.color );
		mSpreadMarkers.SetCell( i, kColWidth, lRow.width );
		mSpreadMarkers.SetCell( i, kColDepth, lRow.depth );
		mSpreadMarkers.SetCell( i, kColHeight, lRow.height );
		lSetMenu( i, kColOrigin, kOriginItems, lRow.pivot );
		lSetMenu( i, kColAnchor, kAnchorItems, lRow.anchor );
		for (const int lColumn : { kColFocus, kColAim, kColRef })
		{
			mSpreadMarkers.SetCell( i, lColumn, (lRow.roles & RoleOfColumn( lColumn )) ? 1 : 0 );
		}
	}
	mFilledSettingsText = (const char*)mDevice->MarkerSettings;

	mFillingMarkers = false;
}


/************************************************
 *	UI idle: the status of the marker link.
 ************************************************/
void CDevice_OSCCamera_Layout::EventUIIdle( HISender pSender, HKEvent pEvent )
{
	if (mTabPanel.ItemIndex == kTabGeneral)
	{
		UIResetPlayback();
	}

	if (mTabPanel.ItemIndex != kTabMarkers)
	{
		return;
	}

	std::string lStatus = (const char*)mDevice->PhoneLink;
	if (lStatus.empty())
	{
		lStatus = mDevice->MarkerLink ? "" : "The markers are sent to the phone while the device is online and this is on";
	}
	if (lStatus != mShownMarkerStatus)
	{
		mShownMarkerStatus = lStatus;
		mLabelMarkerStatus.Caption = lStatus.c_str();
	}

	const std::string lPhoneRoles = (const char*)mDevice->PhoneRoles;
	if (lPhoneRoles != mShownPhoneRoles)
	{
		mShownPhoneRoles = lPhoneRoles;
		mLabelPhoneRoles.Caption = lPhoneRoles.c_str();
	}

	const std::string lCameraSource = (const char*)mDevice->CameraSource;
	if (lCameraSource != mShownCameraSource)
	{
		mShownCameraSource = lCameraSource;
		mLabelCameraSource.Caption = lCameraSource.c_str();
	}

	UIResetRoles();

	// the roles the operator picked on the phone are mirrored into the settings (or a script changed them): the table shows them
	if (!mFillingMarkers && mFilledSettingsText != (const char*)mDevice->MarkerSettings)
	{
		UIResetMarkers();
	}
}


void CDevice_OSCCamera_Layout::EventButtonMarkerLinkClick( HISender pSender, HKEvent pEvent )
{
	mDevice->MarkerLink = (mButtonMarkerLink.State != 0);
}


void CDevice_OSCCamera_Layout::EventEditNumberLinkPortChange( HISender pSender, HKEvent pEvent )
{
	const int lPort = static_cast<int>(mEditNumberLinkPort.Value);
	if (lPort != mDevice->LinkPort)
	{
		mDevice->LinkPort = lPort;
	}
}


void CDevice_OSCCamera_Layout::EventButtonMarkerAddClick( HISender pSender, HKEvent pEvent )
{
	FBModelList lSelected;
	FBGetSelectedModels( lSelected );

	std::vector<FBModel*> lModels;
	for (int i = 0, lCount = lSelected.GetCount(); i < lCount; ++i)
	{
		lModels.push_back( lSelected[i] );
	}

	if (lModels.empty())
	{
		FBMessageBox( "Markers", "Select the models of the scene to send to the phone first.", "Ok" );
		return;
	}

	const int lAdded = mDevice->AddMarkerModels( lModels );
	if (lAdded == 0)
	{
		FBMessageBox( "Markers", "The selected models are in the list already (the camera of the device itself cannot be a marker).", "Ok" );
	}
	else if (mDevice->GetMarkerCount() > OSCCamera::Marker::kMaxMarkers)
	{
		FBMessageBox( "Markers", "The phone takes 16 markers: the first 16 rows of the list are sent.", "Ok" );
	}

	UIResetMarkers();
}


void CDevice_OSCCamera_Layout::EventButtonMarkerRemoveClick( HISender pSender, HKEvent pEvent )
{
	const int lRow = static_cast<int>( mSpreadMarkers.Row );
	if (lRow >= 0 && lRow < mDevice->GetMarkerCount())
	{
		mDevice->RemoveMarkerAt( lRow );
		UIResetMarkers();
	}
}


void CDevice_OSCCamera_Layout::EventButtonMarkerMeasureClick( HISender pSender, HKEvent pEvent )
{
	const int lRow = static_cast<int>( mSpreadMarkers.Row );
	std::vector<OSCCamera::Marker::Settings> lRows = mDevice->GetMarkerSettings();
	if (lRow < 0 || lRow >= static_cast<int>(lRows.size()))
	{
		return;
	}

	if (!mDevice->MeasureMarkerModel( lRow, lRows[lRow] ))
	{
		FBMessageBox( "Markers", "The model has no geometry to measure.", "Ok" );
		return;
	}

	mDevice->SetMarkerSettings( lRows );
	UIFillMarkers();
}


/************************************************
 *	The operator edited a cell of the table.
 ************************************************/
void CDevice_OSCCamera_Layout::EventSpreadMarkersCellChange( HISender pSender, HKEvent pEvent )
{
	if (mFillingMarkers)
	{
		return;
	}

	FBEventSpread lSpreadEvent( pEvent );
	const int lRow = lSpreadEvent.Row;
	const int lColumn = lSpreadEvent.Column;

	std::vector<OSCCamera::Marker::Settings> lRows = mDevice->GetMarkerSettings();
	if (lRow < 0 || lRow >= static_cast<int>(lRows.size()))
	{
		return;
	}

	OSCCamera::Marker::Settings& lSettings = lRows[lRow];
	int lValue = 0;
	double lNumber = 0.0;
	const char* lText = nullptr;

	switch (lColumn)
	{
		case kColName:
			mSpreadMarkers.GetCell( lRow, lColumn, lText );
			lSettings.name = (lText != nullptr) ? lText : "";
			break;
		case kColOn:
			mSpreadMarkers.GetCell( lRow, lColumn, lValue );
			lSettings.visible = (lValue != 0);
			break;
		case kColShape:
			mSpreadMarkers.GetCell( lRow, lColumn, lValue );
			lSettings.shape = std::min( std::max( lValue, 0 ), static_cast<int>(OSCCamera::Marker::kShapeCount) - 1 );
			break;
		case kColColor:
			mSpreadMarkers.GetCell( lRow, lColumn, lValue );
			lSettings.color = std::min( std::max( lValue, 0 ), OSCCamera::Marker::kColorCount - 1 );
			break;
		case kColWidth:
		case kColDepth:
		case kColHeight:
			mSpreadMarkers.GetCell( lRow, lColumn, lNumber );
			// the phone takes 0.05 to 20 metres
			lNumber = std::min( std::max( lNumber, OSCCamera::Marker::kMinSizeMetres ), OSCCamera::Marker::kMaxSizeMetres );
			(lColumn == kColWidth ? lSettings.width : (lColumn == kColDepth ? lSettings.depth : lSettings.height)) = lNumber;
			break;
		case kColOrigin:
			mSpreadMarkers.GetCell( lRow, lColumn, lValue );
			lSettings.pivot = std::min( std::max( lValue, 0 ), static_cast<int>(OSCCamera::Marker::kPivotCount) - 1 );
			break;
		case kColAnchor:
			mSpreadMarkers.GetCell( lRow, lColumn, lValue );
			lSettings.anchor = std::min( std::max( lValue, 0 ), static_cast<int>(OSCCamera::Marker::kAnchorCount) - 1 );
			break;
		case kColFocus:
		case kColAim:
		case kColRef:
			mSpreadMarkers.GetCell( lRow, lColumn, lValue );
			// a marker holds any combination of the roles: this one is switched, the others stay
			lSettings.roles = (lValue != 0) ? (lSettings.roles | RoleOfColumn( lColumn )) : (lSettings.roles & ~RoleOfColumn( lColumn ));
			// one marker per role: the role is taken off the marker that had it
			OSCCamera::Marker::KeepOneRole( lRows, static_cast<size_t>(lRow) );
			break;
		default:
			return;
	}

	mDevice->SetMarkerSettings( lRows );

	// a value that was clamped or not understood is shown as it is kept
	UIFillMarkers();
}
