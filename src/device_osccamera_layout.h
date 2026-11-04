
#pragma once

/**	\file	device_osccamera_layout.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declaration
#include "device_osccamera_device.h"

#include <deque>
#include <string>
#include <vector>

//! Device layout.
class CDevice_OSCCamera_Layout : public FBDeviceLayout
{
	//--- FiLMBOX declaration.
	FBDeviceLayoutDeclare(CDevice_OSCCamera_Layout, FBDeviceLayout );
public:
	//--- FiLMBOX Creation/Destruction.
	virtual bool FBCreate() override;		//!< FiLMBOX Constructor.
	virtual void FBDestroy() override;		//!< FiLMBOX Destructor.

	// UI Management
	void	UICreate				();
	void		UICreateLayout0		();
	void		UICreateLayout1		();
	void		UICreateLayout2		();
	void		UICreateLayout3		();
	void	UIConfigure				();
	void		UIConfigureLayout0	();
	void		UIConfigureLayout1	();
	void		UIConfigureLayout2	();
	void		UIConfigureLayout3	();
	void	UIReset					();		// Reset from device values
	void	UIResetMarkers			();		// Rebuild the table of the markers from the device
	void	UIFillMarkers			();		// Set the values of the cells of the table

	//! a row of a device property: a label with its name on the left, the editor of the value (FBEditProperty or FBEditPropertyModern) on the right
	template <class TEdit>
	void	AddPropertyRow			( FBLayout& pLayout, const char* pRegion, const char* pAttachTo, TEdit& pEdit, FBProperty* pProperty, int pWidth,
									  int pLeft = 0, FBAttachType pTopAttach = kFBAttachBottom, int pTop = 4 );
	//! a hint next to a property row: \p pText in view, \p pTooltip with the details on hover
	template <class TEdit>
	void	AddPropertyHint			( FBLayout& pLayout, const char* pRegion, TEdit& pEdit, const char* pText, const char* pTooltip );

	//! a row of a device property that is one of a few choices: a label with its name on the left, the list on the right
	void	AddListRow				( FBLayout& pLayout, const char* pRegion, const char* pAttachTo, FBList& pList, FBProperty* pProperty, int pWidth );
	void	UIResetPlayback			();		// the lists of the live camera from the properties

	// Main Layout: Events
	void	EventDeviceStatusChange					( HISender pSender, HKEvent pEvent );
	void	EventTabPanelChange						( HISender pSender, HKEvent pEvent );

	// Layout 0: Events
	void	EventEditNumberSamplingRateChange		( HISender pSender, HKEvent pEvent );
	void	EventListSamplingTypeChange				( HISender pSender, HKEvent pEvent );
	void	EventListPlaybackModeChange				( HISender pSender, HKEvent pEvent );
	void	EventListResponsivenessChange			( HISender pSender, HKEvent pEvent );
	void	EventButtonAboutClick					( HISender pSender, HKEvent pEvent );

	// Layout 1: Events
	void	EventEditNumberPortChange				( HISender pSender, HKEvent pEvent );

	// Layout 2 (markers): Events
	void	EventUIIdle								( HISender pSender, HKEvent pEvent );
	void	EventButtonMarkerLinkClick				( HISender pSender, HKEvent pEvent );
	void	EventEditNumberLinkPortChange			( HISender pSender, HKEvent pEvent );
	void	EventButtonMarkerAddClick				( HISender pSender, HKEvent pEvent );
	void	EventButtonMarkerRemoveClick			( HISender pSender, HKEvent pEvent );
	void	EventButtonMarkerMeasureClick			( HISender pSender, HKEvent pEvent );
	void	EventSpreadMarkersCellChange			( HISender pSender, HKEvent pEvent );
	void	EventButtonRoleOptionClick				( HISender pSender, HKEvent pEvent );
	void	EventEditNumberLookAtChange				( HISender pSender, HKEvent pEvent );
	void	EventButtonSendRolesClick				( HISender pSender, HKEvent pEvent );
	void	UIResetRoles			();		// the role options from the device

	// Layout 3 (calibration): Events
	void	EventButtonCalibrationResetClick		( HISender pSender, HKEvent pEvent );

private:
	FBTabPanel			mTabPanel;

	FBLayout			mLayoutGeneral;
		FBLabel				mLabelSamplingRate;
		FBEditNumber		mEditNumberSamplingRate;
		FBLabel				mLabelSamplingType;
		FBList				mListSamplingType;
		FBEditPropertyModern		mEditCameraId;
		FBList				mListPlaybackMode;
		FBEditProperty		mEditPlaybackDelay;
		FBEditProperty		mEditMaxAutoDelay;
		FBList				mListResponsiveness;
		FBEditProperty		mEditApplySensor;
		FBButton			mButtonAbout;

	FBLayout			mLayoutCommunication;
		FBLabel				mLabelPort;
		FBEditNumber		mEditNumberPort;
		FBEditProperty		mEditTimeout;
		FBEditProperty		mEditLogPackets;
		FBEditProperty		mEditRemoteControl;
		FBEditProperty		mEditLastCommand;
		FBEditProperty		mEditPhoneAddress;
		FBEditProperty		mEditJitter;
		FBEditProperty		mEditDelayInUse;
		FBEditProperty		mEditLostPackets;
		FBEditProperty		mEditSensorSize;
		FBEditProperty		mEditStreamUnits;

	FBLayout			mLayoutCalibration;
		FBEditProperty		mEditSpaceScale;
		FBEditProperty		mEditFocusAngleScale;
		FBLabel				mLabelLensRanges;
		FBEditProperty		mEditZoomRawRange;
		FBEditProperty		mEditZoomOutputRange;
		FBEditProperty		mEditFocusRawRange;
		FBEditProperty		mEditFocusOutputRange;
		FBEditProperty		mEditIrisRawRange;
		FBEditProperty		mEditIrisOutputRange;
		FBButton			mButtonCalibrationReset;

	FBLayout			mLayoutMarkers;
		FBButton			mButtonMarkerLink;
		FBLabel				mLabelLinkPort;
		FBEditNumber		mEditNumberLinkPort;
		FBLabel				mLabelMarkerStatus;
		FBButton			mButtonMarkerAdd;
		FBButton			mButtonMarkerRemove;
		FBButton			mButtonMarkerMeasure;
		FBButton			mButtonRefFollowHeading;
		FBButton			mButtonSetLookAt;
		FBEditNumber		mEditNumberLookAt;
		FBButton			mButtonMirrorPhone;
		FBButton			mButtonSendRoles;
		FBLabel				mLabelPhoneRoles;
		FBButton			mButtonReferenceLocal;
		FBLabel				mLabelCameraSource;
		FBSpread			mSpreadMarkers;

private:
	CDevice_OSCCamera*	mDevice;		//!< Handle onto device.
	std::string			mShownMarkerStatus;
	std::string			mShownPhoneRoles;
	std::string			mShownCameraSource;
	std::string			mFilledSettingsText;		//!< the Marker Settings the table shows, a change from elsewhere (the phone's roles mirrored) fills it again
	bool				mFillingMarkers{ false };	//!< the cells are being set by the code, not by the operator
	// the captions and the hints of the property rows. Members like every other control: MotionBuilder frees the controls of a layout
	// it destroys (the Navigator rebuilds the layout on a scene change), a control made with new and deleted again would be freed twice
	static constexpr int	kMaxPropertyLabels = 64;
	FBLabel				mPropertyLabels[kMaxPropertyLabels];
	int					mPropertyLabelCount{ 0 };

	//! the next free label of the rows; past the last one the last is reused (a bug of the layout code, shows as a wrong caption)
	FBLabel&	NextPropertyLabel()	{ return mPropertyLabels[ (mPropertyLabelCount < kMaxPropertyLabels) ? mPropertyLabelCount++ : kMaxPropertyLabels - 1 ]; }
	std::deque<std::string>	mRegionNames;		//!< the region names made at run time: the layout may keep the pointers, they live as long as it

	//! a region name that stays valid as long as the layout
	const char*	KeepRegionName( std::string pName )	{ mRegionNames.push_back( std::move(pName) ); return mRegionNames.back().c_str(); }
};
