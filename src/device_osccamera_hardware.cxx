
/**	\file	device_osccamera_hardware.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declaration
#include "device_osccamera_hardware.h"

#include <algorithm>
#include <chrono>
#include <cstring>

namespace
{
	bool SameLens(const OSCCamera::LensMessage& a, const OSCCamera::LensMessage& b)
	{
		return a.cameraId == b.cameraId
			&& a.zoomCalibrated == b.zoomCalibrated && a.focusCalibrated == b.focusCalibrated && a.irisCalibrated == b.irisCalibrated
			&& a.zoomMin == b.zoomMin && a.zoomMax == b.zoomMax && a.focusMin == b.focusMin && a.focusMax == b.focusMax
			&& a.irisMin == b.irisMin && a.irisMax == b.irisMax
			&& a.sensorWidthMm == b.sensorWidthMm && a.sensorHeightMm == b.sensorHeightMm;
	}
}

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

	{
		std::lock_guard<std::mutex> lLock(mSendMutex);
		mSocket = lSocket;
		mSocketOpen = true;
		mHasPhone = false;
		mPhoneAddress = 0;
		mPhonePort = 0;
	}

	// stale data from the previous session must not be used
	mTimeline.Reset();
	mPlayback.Reset();
	mFilter.Reset();
	mPoseKind = Status::EPoseKind::None;
	mPoseKey[0] = 0;
	mPoseFollow = OSCCamera::kFollowPosition;
	mRejectedPoses = 0;
	mLenses.clear();
	mHasActiveCamera = false;
	mActiveCamera = 0;
	mActiveCameraTime = 0;
	mOtherCamera = false;
	mLinkTime = 0;
	mLinkRolesTime = 0;
	mOutdatedTime = 0;
	mReceived.clear();
	mLastPoseTime = 0;

	mRateWindowStart = 0;
	mRateWindowCount = 0;
	mSampleWindowCount = 0;
	mReceiveRate = 0.0;
	mLostPackets = 0;
	mHasDiagSequence = false;

	{
		std::lock_guard<std::mutex> lLock(mStatusMutex);
		mStatus = Status();
	}

	// a press from before the restart is not to be executed now
	mDeduplicator.Reset();
	{
		std::lock_guard<std::mutex> lLock(mCommandMutex);
		mCommands.clear();
	}

	return true;
}

/************************************************
 *	Close device communications.
 ************************************************/
bool CDevice_OSCCamera_Hardware::Close()
{
	std::lock_guard<std::mutex> lLock(mSendMutex);
	if (mSocketOpen)
	{
		mTCPIP.CloseSocket(mSocket);
		mSocketOpen = false;
		mSocket = 0;
		mHasPhone = false;
	}
	return true;
}

bool CDevice_OSCCamera_Hardware::PopCommand(ReceivedCommand& pCommand)
{
	std::lock_guard<std::mutex> lLock(mCommandMutex);
	if (mCommands.empty())
	{
		return false;
	}

	pCommand = mCommands.front();
	mCommands.pop_front();
	return true;
}

void CDevice_OSCCamera_Hardware::GetStatus(Status& pStatus) const
{
	std::lock_guard<std::mutex> lLock(mStatusMutex);
	pStatus = mStatus;
}

bool CDevice_OSCCamera_Hardware::SendToPhone(const void* pData, size_t pSize, unsigned short pPort)
{
	std::lock_guard<std::mutex> lLock(mSendMutex);
	if (!mSocketOpen || !mHasPhone || pData == nullptr || pSize == 0 || pPort == 0)
	{
		return false;
	}

	int lWritten = 0;
	return mSendTCPIP.WriteDatagram(mSocket, const_cast<void*>(pData), static_cast<int>(pSize), &lWritten, mPhoneAddress, pPort);
}

const OSCCamera::LensMessage* CDevice_OSCCamera_Hardware::GetActiveLens() const
{
	if (!mHasActiveCamera)
	{
		return nullptr;
	}
	const auto lFound = mLenses.find(mActiveCamera);
	return (lFound != mLenses.end()) ? &lFound->second : nullptr;
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

	bool lLensChanged = false;
	bool lPhoneChanged = false;
	kULong lPhoneAddress = 0;
	unsigned short lPhonePort = 0;
	int64_t lNow = NowNanoseconds();

	for (int i = 0; i < kMaxPacketsPerFetch; ++i)
	{
		// the return value of ReadDatagram is true only when exactly the requested number of bytes was read,
		//  a datagram is normally shorter than the buffer, so we rely on the number of bytes instead
		int bytesRead = 0;
		kULong lSender = 0;
		unsigned short lSenderPort = 0;
		mTCPIP.ReadDatagram(mSocket, mBuffer, static_cast<int>(sizeof(mBuffer)), &bytesRead, &lSender, &lSenderPort);

		if (bytesRead <= 0)
		{
			break; // nothing left to read
		}

		lNow = NowNanoseconds();

		OSCCamera::DecodedMessage message;
		const OSCCamera::EMessageType type = OSCCamera::Decode(mBuffer, static_cast<size_t>(bytesRead), message);

		switch (type)
		{
		case OSCCamera::EMessageType::Pose:
		{
			const int32_t lCamera = message.pose.cameraId;

			// one camera drives the device: the one chosen, or with "any" the first that streams, until it falls silent
			if (mCameraId >= 0)
			{
				if (lCamera != mCameraId)
				{
					mOtherCamera = true;
					mOtherCameraId = lCamera;
					mOtherCameraTime = lNow;
					break;
				}
				mHasActiveCamera = true;
				mActiveCamera = lCamera;
			}
			else if (!mHasActiveCamera || lNow - mActiveCameraTime > kCameraSilence)
			{
				if (!mHasActiveCamera || mActiveCamera != lCamera)
				{
					mTimeline.Reset();
					mPlayback.Reset();		// the samples of another phone are no use on this timeline
						mFilter.Reset();
				}
				mHasActiveCamera = true;
				mActiveCamera = lCamera;
				mOtherCamera = false;
			}
			else if (lCamera != mActiveCamera)
			{
				mOtherCamera = true;
				mOtherCameraId = lCamera;
				mOtherCameraTime = lNow;
				break;
			}
			mActiveCameraTime = lNow;

			// one stream session at a time, and in it only the newer datagrams (the samples repeated in them are dropped by the timeline)
			const OSCCamera::StreamFilter::Result lFiltered = mFilter.Accept(message.pose, lNow);
			if (lFiltered == OSCCamera::StreamFilter::Result::Stale || lFiltered == OSCCamera::StreamFilter::Result::Retired)
			{
				++mRejectedPoses;
				if (mVerbose)
				{
					FBTrace("OSC Camera: [%d bytes] cam %d session %d sequence %d dropped (%s)\n", bytesRead, message.pose.cameraId,
						message.pose.sessionId, message.pose.sequence, (lFiltered == OSCCamera::StreamFilter::Result::Stale) ? "not newer" : "session replaced");
				}
				break;
			}
			if (lFiltered == OSCCamera::StreamFilter::Result::NewSession)
			{
				// the samples of the stream that ended must not be blended with those of the new one
				mTimeline.Reset();
				mPlayback.Reset();
			}

			// the phone is where the poses come from, the marker link goes back there
			if (!lPhoneChanged || lPhoneAddress != lSender || lPhonePort != lSenderPort)
			{
				lPhoneChanged = true;
				lPhoneAddress = lSender;
				lPhonePort = lSenderPort;
			}

			mLastPoseTime = lNow;
			++mRateWindowCount;

			if (!message.pose.hasReference)
			{
				mPoseKind = Status::EPoseKind::Legacy;
				mPoseKey[0] = 0;
			}
			else if (message.pose.mode == OSCCamera::kModeRefLocal)
			{
				mPoseKind = Status::EPoseKind::RefLocal;
				std::memcpy(mPoseKey, message.pose.key, sizeof(mPoseKey));
				mPoseFollow = message.pose.follow;
			}
			else
			{
				mPoseKind = Status::EPoseKind::World;
				mPoseKey[0] = 0;
			}

			const OSCCamera::Timeline::AddInfo lInfo = mTimeline.Add(message.pose, lNow);
			if (lInfo.newest && (lInfo.result == OSCCamera::Timeline::AddResult::New || lInfo.result == OSCCamera::Timeline::AddResult::Reset))
			{
				ReceivedPose lReceived;
				lReceived.pose = message.pose;
				lReceived.time = std::min(mTimeline.ToLocalTime(message.pose.captureNanos), lNow);
				mReceived.push_back(lReceived);
				++mSampleWindowCount;
			}

			if (mVerbose)
			{
				const OSCCamera::PoseMessage& lPose = message.pose;
				FBTrace("OSC Camera: [%d bytes] cam %d pan %.2f tilt %.2f roll %.2f pos %.3f %.3f %.3f zoom %d focus %d iris %d capture %lld%s\n",
					bytesRead, lPose.cameraId, lPose.pan, lPose.tilt, lPose.roll, lPose.x, lPose.y, lPose.z, lPose.zoom, lPose.focus, lPose.iris,
					static_cast<long long>(lPose.captureNanos), (lInfo.result == OSCCamera::Timeline::AddResult::Repeat) ? " (repeat)" : "");
					if (lPose.hasReference)
					{
						FBTrace("OSC Camera:   ref session %d seq %d mode %s generation %d follow %s key '%s' local %.3f %.3f %.3f pan %.2f tilt %.2f roll %.2f\n",
							lPose.sessionId, lPose.sequence, (lPose.mode == OSCCamera::kModeRefLocal) ? "local" : "world", lPose.generation,
							(lPose.follow == OSCCamera::kFollowHeading) ? "heading" : "position", lPose.key,
							lPose.local[0], lPose.local[1], lPose.local[2], lPose.local[3], lPose.local[4], lPose.local[5]);
					}
			}
			break;
		}

		case OSCCamera::EMessageType::Lens:
		{
			// kept per camera: it arrives before the first pose, and two phones may stream to one port
			const auto lExisting = mLenses.find(message.lens.cameraId);
			if (lExisting != mLenses.end())
			{
				if (!SameLens(lExisting->second, message.lens))
				{
					lExisting->second = message.lens;
					lLensChanged = lLensChanged || (mHasActiveCamera && message.lens.cameraId == mActiveCamera);
				}
			}
			else if (mLenses.size() < kMaxLenses)
			{
				mLenses[message.lens.cameraId] = message.lens;
				lLensChanged = lLensChanged || (mHasActiveCamera && message.lens.cameraId == mActiveCamera);
			}

			if (mVerbose)
			{
				FBTrace("OSC Camera: [%d bytes] lens cam %d calibrated zoom %d focus %d iris %d sensor %.2f x %.2f mm\n",
					bytesRead, message.lens.cameraId, message.lens.zoomCalibrated, message.lens.focusCalibrated, message.lens.irisCalibrated,
					message.lens.sensorWidthMm, message.lens.sensorHeightMm);
			}
			break;
		}

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

		case OSCCamera::EMessageType::Command:
			// same camera filter as the poses, then the three copies of a press are reduced to one
			if ((mCameraId < 0 || mCameraId == message.cmd.cameraId)
				&& mDeduplicator.Accept(static_cast<uint32_t>(lSender), message.cmd.cameraId, message.cmd.seq, lNow))
			{
				ReceivedCommand lCommand;
				lCommand.command = OSCCamera::ParseCommand(message.cmd.command);
				std::memcpy(lCommand.text, message.cmd.command, sizeof(lCommand.text));
				lCommand.cameraId = message.cmd.cameraId;
				lCommand.seq = message.cmd.seq;

				{
					std::lock_guard<std::mutex> lLock(mCommandMutex);
					if (mCommands.size() < kMaxQueuedCommands)
					{
						mCommands.push_back(lCommand);
					}
				}

				if (mVerbose)
				{
					FBTrace("OSC Camera: [%d bytes] command cam %d press %d '%s'\n",
						bytesRead, lCommand.cameraId, lCommand.seq, lCommand.text);
				}
			}
			break;

		case OSCCamera::EMessageType::Link:
		{
			mLinkTime = lNow;
			std::lock_guard<std::mutex> lLock(mStatusMutex);
			mStatus.link = message.link;
			break;
		}

		case OSCCamera::EMessageType::LinkRoles:
		{
			mLinkRolesTime = lNow;
			std::lock_guard<std::mutex> lLock(mStatusMutex);
			mStatus.linkRoles = message.linkRoles;
			++mStatus.linkRolesCount;
			break;
		}

		case OSCCamera::EMessageType::Units:
		{
			// kept until the device restarts (mStatus is reset on start): the unit of the stream does not come and go
			std::lock_guard<std::mutex> lLock(mStatusMutex);
			mStatus.units = message.units;
			mStatus.unitsHeard = true;
			break;
		}

		case OSCCamera::EMessageType::OutdatedLayout:
			mOutdatedTime = lNow;
			break;

		default:
			if (mVerbose)
			{
				FBTrace("OSC Camera: [%d bytes] message ignored\n", bytesRead);
			}
			break;
		}
	}

	if (lPhoneChanged)
	{
		std::lock_guard<std::mutex> lLock(mSendMutex);
		mHasPhone = true;
		mPhoneAddress = lPhoneAddress;
		mPhonePort = lPhonePort;
	}

	lNow = NowNanoseconds();

	// the rates over the last second, they drop to zero when the stream is silent
	if (mRateWindowStart == 0)
	{
		mRateWindowStart = lNow;
	}
	const int64_t lElapsed = lNow - mRateWindowStart;
	double lSampleRate = -1.0;
	if (lElapsed >= 1000000000LL)
	{
		mReceiveRate = static_cast<double>(mRateWindowCount) * 1e9 / static_cast<double>(lElapsed);
		lSampleRate = static_cast<double>(mSampleWindowCount) * 1e9 / static_cast<double>(lElapsed);
		mRateWindowStart = lNow;
		mRateWindowCount = 0;
		mSampleWindowCount = 0;
	}

	// the snapshot of the main thread
	{
		std::lock_guard<std::mutex> lLock(mStatusMutex);

		if (lPhoneChanged)
		{
			mStatus.hasPhone = true;
			mStatus.phoneAddress = lPhoneAddress;
			mStatus.phonePort = lPhonePort;
		}

		mStatus.hasActiveCamera = mHasActiveCamera;
		mStatus.activeCamera = mActiveCamera;
		mStatus.otherCamera = mOtherCamera && (lNow - mOtherCameraTime <= kStatusMemory);
		mStatus.otherCameraId = mOtherCameraId;

		const OSCCamera::LensMessage* lLens = GetActiveLens();
		const bool lHadLens = mStatus.hasLens;
		mStatus.hasLens = (lLens != nullptr);
		if (lLens != nullptr)
		{
			if (lLensChanged || !lHadLens || mStatus.lens.cameraId != lLens->cameraId)
			{
				++mStatus.lensVersion;
			}
			mStatus.lens = *lLens;
		}

		mStatus.linkHeard = (mLinkTime != 0) && (lNow - mLinkTime <= kStatusMemory);
		mStatus.linkRolesHeard = (mLinkRolesTime != 0) && (lNow - mLinkRolesTime <= kStatusMemory);
		mStatus.outdatedLayout = (mOutdatedTime != 0) && (lNow - mOutdatedTime <= kStatusMemory);

			mStatus.poseKind = mPoseKind;
			std::memcpy(mStatus.poseKey, mPoseKey, sizeof(mStatus.poseKey));
			mStatus.poseFollow = mPoseFollow;
			mStatus.rejectedPoses = mRejectedPoses;

		if (lSampleRate >= 0.0)
		{
			mStatus.sampleRate = lSampleRate;
		}
		mStatus.jitterMs = mTimeline.JitterMilliseconds();
		mStatus.jitterSpreadMs = mTimeline.JitterSpreadMilliseconds();
	}

	return static_cast<int>(mReceived.size());
}


/************************************************
 *	The live camera, and the numbers of how it was drawn for the main thread.
 ************************************************/
OSCCamera::Playback::Frame CDevice_OSCCamera_Hardware::Render(const int64_t pNow, const OSCCamera::Playback::Settings& pPlayback, const OSCCamera::ConvertSettings& pSettings)
{
	const OSCCamera::Playback::Frame lFrame = mPlayback.Render(mTimeline, pNow, pPlayback, pSettings);

	const OSCCamera::Playback::Shares lShares = mPlayback.GetShares(pNow);
	std::lock_guard<std::mutex> lLock(mStatusMutex);
	mStatus.delayMs = static_cast<double>(lFrame.delay) * 1e-6;
	mStatus.frames = lShares;
	return lFrame;
}
