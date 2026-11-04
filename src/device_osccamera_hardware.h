
#pragma once

/**	\file	device_osccamera_hardware.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- SDK include
#include <fbsdk/fbsdk.h>

#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <vector>

#include "osc_camera_decoder.h"
#include "osc_camera_timeline.h"

//! Receives the OSC camera stream over UDP (FBTCPIP, no platform sockets).
//! The stream is read on the device IO thread - FetchData is called from DeviceIONotify, there is no receive thread.
//! The only thing done from another thread is SendToPhone (the marker link) and the snapshots of GetStatus / PopCommand.
class CDevice_OSCCamera_Hardware
{
public:
	//! a new sensor sample of the stream with the time it belongs to on our clock
	struct ReceivedPose
	{
		OSCCamera::PoseMessage	pose;
		int64_t					time{ 0 };		//!< NowNanoseconds() the sample was taken at, plus the best case delay of the network
	};

	//! a button press of the app, the repeats are already dropped
	struct ReceivedCommand
	{
		OSCCamera::ECommand		command{ OSCCamera::ECommand::Unknown };
		char					text[OSCCamera::kMaxCommandLength + 1]{};	//!< the command string as sent, also for the unknown ones
		int32_t					cameraId{ 0 };
		int32_t					seq{ 0 };
	};

	//! A snapshot of what the stream says, for the main thread.
	struct Status
	{
		bool					hasPhone{ false };			//!< a pose has arrived since the start: its sender is the phone
		kULong					phoneAddress{ 0 };			//!< as FBTCPIP gives it
		unsigned short			phonePort{ 0 };

		bool					hasActiveCamera{ false };
		int32_t					activeCamera{ 0 };			//!< the camera whose poses are used
		bool					otherCamera{ false };		//!< poses of a camera that is not used keep arriving
		int32_t					otherCameraId{ 0 };

		bool					hasLens{ false };			//!< a lens message of the active camera has arrived
		OSCCamera::LensMessage	lens;
		uint32_t				lensVersion{ 0 };			//!< grows with every lens message that differs from the previous one

		bool					linkHeard{ false };			//!< a /livetracker/link arrived lately
		OSCCamera::LinkMessage	link;

		bool					unitsHeard{ false };		//!< a /livetracker/units has arrived since the start: the stream names the unit of its positions
		OSCCamera::UnitsMessage	units;

		bool					linkRolesHeard{ false };	//!< a /livetracker/link/roles arrived lately (the phone shows its Live scene)
		OSCCamera::LinkRolesMessage	linkRoles;
		uint32_t				linkRolesCount{ 0 };		//!< grows with every report: a new number is a new report

		bool					outdatedLayout{ false };	//!< a pose or lens of an older app version has arrived lately

		//! what the newest pose of the camera says about its reference
		enum class EPoseKind : uint8_t
		{
			None,			//!< no pose yet
			Legacy,			//!< a /livetracker/pose: a world pose, there is no reference in the message
			World,			//!< a /livetracker/pose/ref in WORLD mode (the phone has no usable Ref, or is not following one)
			RefLocal		//!< a /livetracker/pose/ref relative to the Ref marker poseKey
		};
		EPoseKind				poseKind{ EPoseKind::None };
		char					poseKey[OSCCamera::kMaxReferenceKeyLength + 1]{};	//!< the Ref marker of a RefLocal pose
		int32_t					poseFollow{ OSCCamera::kFollowPosition };
		uint32_t				rejectedPoses{ 0 };			//!< poses dropped as duplicates, older sequences or late datagrams of a replaced session

		double					sampleRate{ 0.0 };			//!< sensor samples per second (the repeats are not counted)
		double					jitterMs{ 0.0 };			//!< the spread of the network delay seen over the last seconds (largest - smallest)
		double					jitterSpreadMs{ 0.0 };		//!< the same without the one late packet (99th - 1st percentile)

		double					delayMs{ 0.0 };				//!< the delay the live camera is drawn with (the last render)
		OSCCamera::Playback::Shares	frames;					//!< how the frames of the last seconds were drawn
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

	//! Drain all the pending datagrams (device IO thread).
	//! The poses go onto the timeline of the phone. Every new sensor sample is also kept (GetReceivedPose) for the recording, in the
	//! order of its capture - the repeats of a sample and the late ones are dropped there.
	//! \return the number of new samples
	int			FetchData();

	int			GetReceivedPoseCount() const				{ return static_cast<int>(mReceived.size()); }
	const ReceivedPose&	GetReceivedPose(int pIndex) const	{ return mReceived[pIndex]; }

	//! The live camera at \p pNow as the playback settings say: a fixed delay, one that follows the jitter of the network, or the newest sample.
	//! Also keeps the numbers of the playback for GetStatus (device IO thread)
	OSCCamera::Playback::Frame	Render(int64_t pNow, const OSCCamera::Playback::Settings& pPlayback, const OSCCamera::ConvertSettings& pSettings);

	//! The lens of the active camera, nullptr until it has arrived (device IO thread)
	const OSCCamera::LensMessage*	GetActiveLens() const;

	//! Take the oldest command received by FetchData. Thread safe: FetchData runs on the device IO thread, the commands
	//! have to be executed on the main thread (the transport must not be driven from the IO thread).
	//! \return false if there is none
	bool		PopCommand(ReceivedCommand& pCommand);

	//! A snapshot for the main thread. Thread safe.
	void		GetStatus(Status& pStatus) const;

	//! Send a datagram to the phone that streams (the address of its poses) at \p pPort. Thread safe.
	//! \return false if there is no phone yet or the socket is closed
	bool		SendToPhone(const void* pData, size_t pSize, unsigned short pPort);

	//--- Settings (the port takes effect on the next Open)
	void		SetNetworkPort(int pPort)	{ mNetworkPort = pPort; }
	int			GetNetworkPort() const		{ return mNetworkPort; }

	//! only poses with this camera id are accepted, -1 - accept any (the first one that streams, until it falls silent)
	void		SetCameraId(int pCameraId)	{ mCameraId = pCameraId; }
	int			GetCameraId() const			{ return mCameraId; }

	void		SetVerbose(bool pVerbose)	{ mVerbose = pVerbose; }

	//--- Latest received data
	bool		HasPose() const				{ return mTimeline.HasPose(); }
	bool		GetNewestPose(OSCCamera::PoseMessage& pPose) const	{ return mTimeline.GetNewest(pPose); }
	const OSCCamera::DiagMessage&	GetDiag() const	{ return mDiag; }

	//--- Statistics, safe to read from any thread
	double		GetReceiveRate() const		{ return mReceiveRate; }	//!< poses per second, over the last second
	int			GetLostPackets() const		{ return mLostPackets; }	//!< gaps in the diag sequence, the app must have Stream diagnostics on
	int64_t		GetLastPoseTime() const		{ return mLastPoseTime; }	//!< NowNanoseconds() of the last pose of the camera, 0 - none since the start

private:
	static constexpr int kMaxPacketsPerFetch = 256;		//!< keeps a flood from holding the IO thread, the rest is read next time
	static constexpr size_t kMaxQueuedCommands = 32;	//!< the main thread is not draining (busy UI): the newest are dropped
	static constexpr size_t kMaxLenses = 16;			//!< lens messages kept, one per camera id
	static constexpr int64_t kCameraSilence = 1000000000LL;		//!< ns, a camera that sends no pose for this long can be replaced by another
	static constexpr int64_t kStatusMemory = 5000000000LL;		//!< ns, a link or outdated layout message is forgotten after this

	FBTCPIP		mTCPIP;				//!< reading, device IO thread
	FBTCPIP		mSendTCPIP;			//!< writing, any thread (under mSendMutex)
	int			mSocket{ 0 };
	bool		mSocketOpen{ false };

	int			mNetworkPort{ 4000 };
	int			mCameraId{ -1 };
	bool		mVerbose{ false };

	// the stream (IO thread)
	OSCCamera::Timeline					mTimeline;
	OSCCamera::Playback					mPlayback;
	OSCCamera::StreamFilter				mFilter;		//!< session and output sequence of the active camera
	Status::EPoseKind					mPoseKind{ Status::EPoseKind::None };
	char								mPoseKey[OSCCamera::kMaxReferenceKeyLength + 1]{};
	int32_t								mPoseFollow{ OSCCamera::kFollowPosition };
	uint32_t							mRejectedPoses{ 0 };
	std::map<int32_t, OSCCamera::LensMessage>	mLenses;
	OSCCamera::DiagMessage				mDiag;
	bool		mHasActiveCamera{ false };
	int32_t		mActiveCamera{ 0 };
	int64_t		mActiveCameraTime{ 0 };		//!< arrival of the last pose of the active camera
	bool		mOtherCamera{ false };
	int32_t		mOtherCameraId{ 0 };
	int64_t		mOtherCameraTime{ 0 };
	int64_t		mLinkTime{ 0 };
	int64_t		mLinkRolesTime{ 0 };
	int64_t		mOutdatedTime{ 0 };

	std::vector<ReceivedPose>	mReceived;			//!< the new samples of the last FetchData

	// statistics
	int64_t		mRateWindowStart{ 0 };
	int			mRateWindowCount{ 0 };
	int			mSampleWindowCount{ 0 };
	std::atomic<double>	mReceiveRate{ 0.0 };
	std::atomic<int>	mLostPackets{ 0 };
	std::atomic<int64_t>	mLastPoseTime{ 0 };
	bool		mHasDiagSequence{ false };
	int32_t		mLastDiagSequence{ 0 };

	// what the main thread reads
	mutable std::mutex			mStatusMutex;
	Status						mStatus;

	// the phone's address, and the right to write to the socket
	std::mutex					mSendMutex;
	bool						mHasPhone{ false };
	kULong						mPhoneAddress{ 0 };
	unsigned short				mPhonePort{ 0 };

	// button commands: filled on the IO thread, taken on the main thread
	std::mutex					mCommandMutex;
	std::deque<ReceivedCommand>	mCommands;
	OSCCamera::CommandDeduplicator	mDeduplicator;		//!< IO thread only

	uint8_t		mBuffer[OSCCamera::kMaxDatagramSize];
};
