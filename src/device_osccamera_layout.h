
#pragma once

/**	\file	device_osccamera_layout.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declaration
#include "device_osccamera_device.h"

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
	void	UIConfigure				();
	void		UIConfigureLayout0	();
	void		UIConfigureLayout1	();
	void	UIReset					();		// Reset from device values

	// Main Layout: Events
	void	EventDeviceStatusChange					( HISender pSender, HKEvent pEvent );
	void	EventTabPanelChange						( HISender pSender, HKEvent pEvent );

	// Layout 0: Events
	void	EventEditNumberSamplingRateChange		( HISender pSender, HKEvent pEvent );
	void	EventListSamplingTypeChange				( HISender pSender, HKEvent pEvent );
	void	EventButtonAboutClick					( HISender pSender, HKEvent pEvent );

	// Layout 1: Events
	void	EventEditNumberPortChange				( HISender pSender, HKEvent pEvent );

private:
	FBTabPanel			mTabPanel;

	FBLayout			mLayoutGeneral;
		FBLabel				mLabelSamplingRate;
		FBEditNumber		mEditNumberSamplingRate;
		FBLabel				mLabelSamplingType;
		FBList				mListSamplingType;
		FBButton			mButtonAbout;

	FBLayout			mLayoutCommunication;
		FBLabel				mLabelPort;
		FBEditNumber		mEditNumberPort;

private:
	CDevice_OSCCamera*	mDevice;		//!< Handle onto device.
};
