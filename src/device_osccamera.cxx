
/**	\file	device_osccamera.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- SDK include
#include <fbsdk/fbsdk.h>

#ifdef KARCH_ENV_WIN
	#include <windows.h>
#endif

//--- Library declaration
FBLibraryDeclare( device_osccamera )
{
	FBLibraryRegister( CDevice_OSCCamera		);
	FBLibraryRegister( CDevice_OSCCamera_Layout	);
}
FBLibraryDeclareEnd;

/************************************************
 *	Library functions.
 ************************************************/
bool FBLibrary::LibInit()	{ return true; }
bool FBLibrary::LibOpen()	{ return true; }
bool FBLibrary::LibReady()	{ return true; }
bool FBLibrary::LibClose()	{ return true; }
bool FBLibrary::LibRelease(){ return true; }

/**
*	\mainpage	OSC Camera Tracker Device
*	\section	intro	Introduction
*	Receives the 6DoF Camera Tracker OSC stream (UDP) and drives a camera
*	transform and lens properties.
*/
