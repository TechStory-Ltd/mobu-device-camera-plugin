
#pragma once

/**	\file	device_osccamera_hardware.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- SDK include
#include <fbsdk/fbsdk.h>

#include <atomic>
#include <cstdint>
#include <vector>

#include "osc_camera_decoder.h"

//! Receives the OSC camera stream over UDP (FBTCPIP, no platform sockets).
//! Everything runs on the device IO thread - FetchData is called from DeviceIONotify, there is no receive thread.
class CDevice_OSCCamera_Hardware
{
public:
	//! a pose of the stream with the time it is estimated to have arrived
	struct ReceivedPose
	{
		OSCCamera::PoseMessage	pose;
		int64_t					time{ 0 };		//!< NowNanoseconds()
	};

public:
	CDevice_OSCCamera_Hardware();
	~CDevice_OSCCamera_Hardware();

	//! monotonic clock, the same one for every thread
	static int64_t	NowNanoseconds();

	//--- Communications
	bool		Open();						//!< create a UDP socket and bind it to the network port
	bool		Close();					//!< close the socket
	bool		IsOpen() const				{ return mSocketOpen; }

	//! drain all the pending datagrams.
	//! Every pose of the selected camera is kept (GetReceivedPose), in the order of arrival - the device ticks and
	//! the stream are not synchronized, so several poses may be waiting at one tick, dropping all but the last
	//! one would lose samples. The newest pose is also in GetPose.
	//! \return the number of poses received
	int			FetchData();

	int			GetReceivedPoseCount() const				{ return static_cast<int>(mReceived.size()); }
	const ReceivedPose&	GetReceivedPose(int pIndex) const	{ return mReceived[pIndex]; }

	//! \return true if a lens message arrived since the previous call
	bool		ConsumeLensChanged();

	//--- Settings (take effect on the next Open)
	void		SetNetworkPort(int pPort)	{ mNetworkPort = pPort; }
	int			GetNetworkPort() const		{ return mNetworkPort; }

	//! only poses with this camera id are accepted, -1 - accept any
	void		SetCameraId(int pCameraId)	{ mCameraId = pCameraId; }
	int			GetCameraId() const			{ return mCameraId; }

	void		SetVerbose(bool pVerbose)	{ mVerbose = pVerbose; }

	//--- Latest received data
	bool		HasPose() const				{ return mHasPose; }
	bool		HasLens() const				{ return mHasLens; }
	const OSCCamera::PoseMessage&	GetPose() const	{ return mPose; }
	const OSCCamera::LensMessage&	GetLens() const	{ return mLens; }
	const OSCCamera::DiagMessage&	GetDiag() const	{ return mDiag; }

	//--- Statistics, safe to read from any thread
	double		GetReceiveRate() const		{ return mReceiveRate; }	//!< poses per second, over the last second
	int			GetLostPackets() const		{ return mLostPackets; }	//!< gaps in the diag sequence, the app must have Stream diagnostics on

private:
	static constexpr int kMaxPacketsPerFetch = 256;		//!< keeps a flood from holding the IO thread, the rest is read next time
	static constexpr int64_t kMaxFetchGap = 250000000;	//!< ns, the longest time span the poses of a fetch can be spread over

	FBTCPIP		mTCPIP;
	int			mSocket{ 0 };
	bool		mSocketOpen{ false };

	int			mNetworkPort{ 4000 };
	int			mCameraId{ -1 };
	bool		mVerbose{ false };

	bool		mHasPose{ false };
	bool		mHasLens{ false };
	bool		mLensChanged{ false };
	OSCCamera::PoseMessage	mPose;
	OSCCamera::LensMessage	mLens;
	OSCCamera::DiagMessage	mDiag;

	std::vector<ReceivedPose>	mReceived;			//!< the poses of the last FetchData
	int64_t		mLastFetchTime{ 0 };

	// statistics
	int64_t		mRateWindowStart{ 0 };
	int			mRateWindowCount{ 0 };
	std::atomic<double>	mReceiveRate{ 0.0 };
	std::atomic<int>	mLostPackets{ 0 };
	bool		mHasDiagSequence{ false };
	int32_t		mLastDiagSequence{ 0 };

	uint8_t		mBuffer[OSCCamera::kMaxDatagramSize];
};
