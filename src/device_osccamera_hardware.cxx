
/**	\file	device_osccamera_hardware.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declaration
#include "device_osccamera_hardware.h"

#include <algorithm>
#include <chrono>

/************************************************
 *	Constructor / Destructor.
 ************************************************/
CDevice_OSCCamera_Hardware::CDevice_OSCCamera_Hardware()
{
	mReceived.reserve(kMaxPacketsPerFetch);
}

CDevice_OSCCamera_Hardware::~CDevice_OSCCamera_Hardware()
{
	Close();
}

int64_t CDevice_OSCCamera_Hardware::NowNanoseconds()
{
	return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

/************************************************
 *	Open device communications.
 ************************************************/
bool CDevice_OSCCamera_Hardware::Open()
{
	Close();

	int lSocket = 0;

	// non-blocking UDP socket
	if (!mTCPIP.CreateSocket(lSocket, kFBTCPIP_DGRAM, "udp", true))
	{
		FBTrace("OSC Camera: failed to create a socket\n");
		return false;
	}

	// INADDR_ANY = 0
	if (!mTCPIP.Bind(lSocket, 0, mNetworkPort))
	{
		FBTrace("OSC Camera: failed to bind port %d\n", mNetworkPort);
		mTCPIP.CloseSocket(lSocket);
		return false;
	}

	mSocket = lSocket;
	mSocketOpen = true;

	// stale data from the previous session must not be used
	mHasPose = false;
	mHasLens = false;
	mLensChanged = false;
	mReceived.clear();
	mLastFetchTime = 0;

	mRateWindowStart = 0;
	mRateWindowCount = 0;
	mReceiveRate = 0.0;
	mLostPackets = 0;
	mHasDiagSequence = false;

	return true;
}

/************************************************
 *	Close device communications.
 ************************************************/
bool CDevice_OSCCamera_Hardware::Close()
{
	if (mSocketOpen)
	{
		mTCPIP.CloseSocket(mSocket);
		mSocketOpen = false;
		mSocket = 0;
	}
	return true;
}

bool CDevice_OSCCamera_Hardware::ConsumeLensChanged()
{
	const bool lChanged = mLensChanged;
	mLensChanged = false;
	return lChanged;
}

/************************************************
 *	Read all the pending datagrams.
 ************************************************/
int CDevice_OSCCamera_Hardware::FetchData()
{
	mReceived.clear();

	if (!mSocketOpen)
	{
		return 0;
	}

	for (int i = 0; i < kMaxPacketsPerFetch; ++i)
	{
		// the return value of ReadDatagram is true only when exactly the requested number of bytes was read,
		//  a datagram is normally shorter than the buffer, so we rely on the number of bytes instead
		int bytesRead = 0;
		mTCPIP.ReadDatagram(mSocket, mBuffer, static_cast<int>(sizeof(mBuffer)), &bytesRead);

		if (bytesRead <= 0)
		{
			break; // nothing left to read
		}

		OSCCamera::DecodedMessage message;
		const OSCCamera::EMessageType type = OSCCamera::Decode(mBuffer, static_cast<size_t>(bytesRead), message);

		switch (type)
		{
		case OSCCamera::EMessageType::Pose:
			if (mCameraId < 0 || mCameraId == message.pose.cameraId)
			{
				mPose = message.pose;
				mHasPose = true;

				ReceivedPose lReceived;
				lReceived.pose = message.pose;
				mReceived.push_back(lReceived);

				if (mVerbose)
				{
					FBTrace("OSC Camera: [%d bytes] cam %d pan %.2f tilt %.2f roll %.2f pos %.3f %.3f %.3f zoom %d focus %d iris %d\n",
						bytesRead, mPose.cameraId, mPose.pan, mPose.tilt, mPose.roll, mPose.x, mPose.y, mPose.z, mPose.zoom, mPose.focus, mPose.iris);
				}
			}
			break;

		case OSCCamera::EMessageType::Lens:
			mLens = message.lens;
			mHasLens = true;
			mLensChanged = true;

			if (mVerbose)
			{
				FBTrace("OSC Camera: [%d bytes] lens calibrated zoom %d focus %d iris %d\n",
					bytesRead, mLens.zoomCalibrated, mLens.focusCalibrated, mLens.irisCalibrated);
			}
			break;

		case OSCCamera::EMessageType::Diag:
			mDiag = message.diag;

			// the sequence grows by one per frame, a bigger step is the frames that never arrived
			if (mHasDiagSequence)
			{
				const int32_t lStep = message.diag.sequence - mLastDiagSequence;
				if (lStep > 1)
				{
					mLostPackets += lStep - 1;
				}
			}
			mLastDiagSequence = message.diag.sequence;
			mHasDiagSequence = true;
			break;

		default:
			if (mVerbose)
			{
				FBTrace("OSC Camera: [%d bytes] message ignored\n", bytesRead);
			}
			break;
		}
	}

	const int64_t lNow = NowNanoseconds();
	const int lCount = static_cast<int>(mReceived.size());

	// The OS gives no arrival time of a datagram. The poses of this fetch arrived somewhere between the previous
	//  fetch and now - spread them evenly over that time, so the samples keep an even spacing when they were queued up.
	if (lCount > 0)
	{
		const int64_t lGapStart = (mLastFetchTime == 0) ? lNow : std::max(mLastFetchTime, lNow - kMaxFetchGap);
		for (int i = 0; i < lCount; ++i)
		{
			mReceived[i].time = lGapStart + (lNow - lGapStart) * (i + 1) / lCount;
		}
	}
	mLastFetchTime = lNow;

	// the receive rate over the last second, it drops to zero when the stream is silent
	if (mRateWindowStart == 0)
	{
		mRateWindowStart = lNow;
	}
	mRateWindowCount += lCount;

	const int64_t lElapsed = lNow - mRateWindowStart;
	if (lElapsed >= 1000000000LL)
	{
		mReceiveRate = static_cast<double>(mRateWindowCount) * 1e9 / static_cast<double>(lElapsed);
		mRateWindowStart = lNow;
		mRateWindowCount = 0;
	}

	return lCount;
}
