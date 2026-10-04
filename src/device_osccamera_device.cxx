
/**	\file	device_osccamera_device.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declaration
#include "device_osccamera_device.h"

#include <algorithm>
#include <chrono>
#include <cmath>

//--- Registration defines
#define CDEVICEOSCCAMERA__CLASS		CDEVICEOSCCAMERA__CLASSNAME
#define CDEVICEOSCCAMERA__NAME		CDEVICEOSCCAMERA__CLASSSTR
#define CDEVICEOSCCAMERA__LABEL		"OSC Camera Tracker Device"
#define CDEVICEOSCCAMERA__DESC		"OSC Camera Tracker Device"
#define CDEVICEOSCCAMERA__PREFIX	"OSCCamera"

//--- FiLMBOX implementation and registration
FBDeviceImplementation	(	CDEVICEOSCCAMERA__CLASS	);
FBRegisterDevice		(	CDEVICEOSCCAMERA__NAME,
							CDEVICEOSCCAMERA__CLASS,
							CDEVICEOSCCAMERA__LABEL,
							CDEVICEOSCCAMERA__DESC,
							"camera.png");	// Icon filename (default=Open Reality icon)


/************************************************
 *	FiLMBOX Constructor.
 ************************************************/
bool CDevice_OSCCamera::FBCreate()
{
	FBPropertyPublish(this, Port, "Port", nullptr, nullptr);
	FBPropertyPublish(this, CameraId, "Camera ID", nullptr, nullptr);
	FBPropertyPublish(this, LogPackets, "Log Packets", nullptr, nullptr);
	FBPropertyPublish(this, Timeout, "Timeout (ms)", nullptr, nullptr);
	FBPropertyPublish(this, ReceiveRate, "Receive Rate (Hz)", nullptr, nullptr);
	FBPropertyPublish(this, LostPackets, "Lost Packets", nullptr, nullptr);

	FBPropertyPublish(this, FocusAngleScale, "Focus Angle Scale", nullptr, nullptr);
	FBPropertyPublish(this, SpaceScale, "Space Scale", nullptr, nullptr);
	FBPropertyPublish(this, PositionSign, "Position Sign", nullptr, nullptr);
	FBPropertyPublish(this, RotationSign, "Rotation Sign", nullptr, nullptr);

	FBPropertyPublish(this, ZoomRawRange, "Zoom Raw Range", nullptr, nullptr);
	FBPropertyPublish(this, ZoomOutputRange, "Zoom Output Range", nullptr, nullptr);
	FBPropertyPublish(this, FocusRawRange, "Focus Raw Range", nullptr, nullptr);
	FBPropertyPublish(this, FocusOutputRange, "Focus Output Range", nullptr, nullptr);
	FBPropertyPublish(this, IrisRawRange, "Iris Raw Range", nullptr, nullptr);
	FBPropertyPublish(this, IrisOutputRange, "Iris Output Range", nullptr, nullptr);

	// defaults are the same as in OSCCamera::ConvertSettings
	const OSCCamera::ConvertSettings lDefaults;

	Port = 4000;
	CameraId = -1;
	LogPackets = false;
	Timeout = 500;

	ReceiveRate = 0.0;
	LostPackets = 0;
	ReceiveRate.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	LostPackets.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);

	FocusAngleScale = lDefaults.focusAngleScale;
	SpaceScale = lDefaults.spaceScale;
	PositionSign = FBVector3d(1.0, 1.0, 1.0);
	RotationSign = FBVector3d(1.0, 1.0, 1.0);

	ZoomRawRange = FBVector2d(lDefaults.zoomRaw.min, lDefaults.zoomRaw.max);
	ZoomOutputRange = FBVector2d(lDefaults.zoomOut.min, lDefaults.zoomOut.max);
	FocusRawRange = FBVector2d(lDefaults.focusRaw.min, lDefaults.focusRaw.max);
	FocusOutputRange = FBVector2d(lDefaults.focusOut.min, lDefaults.focusOut.max);
	IrisRawRange = FBVector2d(lDefaults.irisRaw.min, lDefaults.irisRaw.max);
	IrisOutputRange = FBVector2d(lDefaults.irisOut.min, lDefaults.irisOut.max);

	// Create animation nodes
	mNodeCamera_T = AnimationNodeOutCreate(0, "Translation", ANIMATIONNODE_TYPE_LOCAL_TRANSLATION);
	mNodeCamera_R = AnimationNodeOutCreate(1, "Rotation", ANIMATIONNODE_TYPE_LOCAL_ROTATION);
	mNodeCamera_FocalLength = AnimationNodeOutCreate(2, "Focal Length", ANIMATIONNODE_TYPE_NUMBER);
	mNodeCamera_FocusDistance = AnimationNodeOutCreate(3, "Focus Distance", ANIMATIONNODE_TYPE_NUMBER);
	mNodeIris					= AnimationNodeOutCreate( 4, "Iris",			ANIMATIONNODE_TYPE_NUMBER				);
	mNodeCamera_FocusAngle		= AnimationNodeOutCreate( 5, "Focus Angle",		ANIMATIONNODE_TYPE_NUMBER				);

	// Create model templates
	mTemplateRoot	= new FBModelTemplate( CDEVICEOSCCAMERA__PREFIX, "Reference",	kFBModelTemplateRoot	);
	mTemplateCamera	= new FBModelTemplate( CDEVICEOSCCAMERA__PREFIX, "Camera",		kFBModelTemplateCamera	);

	// Build model template hierarchy
	ModelTemplate.Children.Add(mTemplateRoot);
	mTemplateRoot->Children.Add(mTemplateCamera);

	// Bind the model templates to device's animation nodes, a binding matches the node type with the model node,
	//  only the transform has such a type - the lens nodes are connected to the camera properties explicitly
	mTemplateCamera->Bindings.Add(mNodeCamera_T);
	mTemplateCamera->Bindings.Add(mNodeCamera_R);

	// Set sampling rate to 60 Hz
	FBTime	lPeriod;
	lPeriod.SetSecondDouble(1.0/60.0);
	SamplingPeriod	= lPeriod;

	CommType = kFBCommTypeNetworkUDP;

	// by default ready to record
	RecordMode = true;

	// a valid output from the very start - the zero pose
	PublishOutput(ConvertPose(OSCCamera::PoseMessage(), GetConvertSettings(), nullptr));

	FBSystem& system = FBSystem::TheOne();
	system.OnUIIdle.Add(this, (FBCallback)&CDevice_OSCCamera::OnUIIdleEvent);

	return true;
}

/************************************************
 *	FiLMBOX Destructor.
 ************************************************/
void CDevice_OSCCamera::FBDestroy()
{
	FBSystem& system = FBSystem::TheOne();
	system.OnUIIdle.Remove(this, (FBCallback)&CDevice_OSCCamera::OnUIIdleEvent);
	mHardware.Close();
}


/************************************************
 *	Device operation.
 ************************************************/
bool CDevice_OSCCamera::DeviceOperation( kDeviceOperations pOperation )
{
	switch (pOperation)
	{
		case kOpInit:	return Init();
		case kOpStart:	return Start();
		case kOpStop:	return Stop();
		case kOpReset:	return Reset();
		case kOpDone:	return Done();
	}
	return FBDevice::DeviceOperation( pOperation );
}


/************************************************
 *	Initialization of device.
 ************************************************/
bool CDevice_OSCCamera::Init()
{
	return true;
}


/************************************************
 *	Device is put online.
 ************************************************/
bool CDevice_OSCCamera::Start()
{
	Status = "Opening UDP port";

	mHardware.SetNetworkPort(Port);
	mHardware.SetCameraId(CameraId);
	mHardware.SetVerbose(LogPackets);

	if (!mHardware.Open())
	{
		char lStatus[64];
		snprintf(lStatus, sizeof(lStatus), "Could not open UDP port %d", static_cast<int>(Port));
		Status = lStatus;
		return false;
	}

	HardwareVersionInfo = "OSC Camera Tracker, v1.0";
	Information = "";

	// a camera bound earlier (or loaded with the scene) gets its lens connected
	ConnectCameraLens(mTemplateCamera->Model);

	// the stream liveness is tracked from now on
	mLastPoseTime = 0;
	mStartTime = NowNanoseconds();
	mStreamState = EStreamState::Waiting;

	char lStatus[64];
	snprintf(lStatus, sizeof(lStatus), "Listening on UDP port %d", static_cast<int>(Port));
	Status = lStatus;
	return true;
}


/************************************************
 *	Device is stopped (offline).
 ************************************************/
bool CDevice_OSCCamera::Stop()
{
	mHardware.Close();
	Status = "Offline";
	return true;
}


/************************************************
 *	Removal of device.
 ************************************************/
bool CDevice_OSCCamera::Done()
{
	return true;
}


/************************************************
 *	Reset of device.
 ************************************************/
bool CDevice_OSCCamera::Reset()
{
	Stop();
	return Start();
}


/************************************************
 *	Real-Time Engine Evaluation.
 ************************************************/
bool CDevice_OSCCamera::AnimationNodeNotify(FBAnimationNode* pAnimationNode, FBEvaluateInfo* pEvaluateInfo)
{
	CameraOutput lOutput;
	{
		std::lock_guard<std::mutex> lLock(mOutputMutex);
		lOutput = mOutput;
	}

	mNodeCamera_T->WriteData(lOutput.position, pEvaluateInfo);
	mNodeCamera_R->WriteData(lOutput.rotation, pEvaluateInfo);
	mNodeCamera_FocalLength->WriteData(&lOutput.focalLength, pEvaluateInfo);
	mNodeCamera_FocusDistance->WriteData(&lOutput.focusDistance, pEvaluateInfo);
	mNodeIris->WriteData(&lOutput.iris, pEvaluateInfo);
	mNodeCamera_FocusAngle->WriteData(&lOutput.focusAngle, pEvaluateInfo);

	return true;
}


/************************************************
 *	Device Evaluation Notify.
 ************************************************/
bool CDevice_OSCCamera::DeviceEvaluationNotify( kTransportMode pMode, FBEvaluateInfo* pEvaluateInfo )
{
	return true;
}


/************************************************
 *	Real-Time Synchronous Device IO.
 ************************************************/
void CDevice_OSCCamera::DeviceIONotify( kDeviceIOs pAction, FBDeviceNotifyInfo &pDeviceNotifyInfo )
{
	switch (pAction)
	{
		// Input device
		case kIOStopModeRead:
		case kIOPlayModeRead:
		{
			// drain the socket: every pose that arrived since the previous tick, the ticks and the stream are not
			//  synchronized, so there can be none or several of them
			const int lCount = mHardware.FetchData();
			const bool lLensChanged = mHardware.ConsumeLensChanged();

			if (lCount > 0 || (lLensChanged && mHardware.HasPose()))
			{
				const OSCCamera::ConvertSettings lSettings = GetConvertSettings();
				const bool lHasLens = mHardware.HasLens();
				const OSCCamera::LensMessage lLens = mHardware.GetLens();
				const OSCCamera::LensMessage* lLensPtr = lHasLens ? &lLens : nullptr;

				CameraOutput lOutput;

				if (lCount > 0)
				{
					const int64_t lNow = CDevice_OSCCamera_Hardware::NowNanoseconds();

					// each pose is a sample: convert (in order, the angles are kept continuous), record and count it
					for (int i = 0; i < lCount; ++i)
					{
						const CDevice_OSCCamera_Hardware::ReceivedPose& lReceived = mHardware.GetReceivedPose(i);

						lOutput = ConvertPose(lReceived.pose, lSettings, lLensPtr);
						DeviceRecordFrame(pDeviceNotifyInfo, lOutput, static_cast<double>(lNow - lReceived.time) * 1e-9);
						AckOneSampleReceived();
					}

					mLastPoseTime = lNow;
				}
				else
				{
					// only the lens has changed, show it on the last pose
					lOutput = ConvertPose(mHardware.GetPose(), lSettings, lLensPtr);
				}

				// the live camera shows the newest sample
				PublishOutput(lOutput);
			}
		}
		break;

		default:
			break;
	}
}


/************************************************
 *	Monotonic time, the same clock on every thread.
 ************************************************/
int64_t CDevice_OSCCamera::NowNanoseconds()
{
	return CDevice_OSCCamera_Hardware::NowNanoseconds();
}


/************************************************
 *	UI idle (main thread): report whether the stream is alive.
 *	Status is set only on a change, it notifies the layout.
 ************************************************/
void CDevice_OSCCamera::OnUIIdleEvent(HIRegister pSender, HKEvent pEvent)
{
	if (!Online)
	{
		return;
	}

	// statistics from the IO thread, set only on a change
	const double lRate = mHardware.GetReceiveRate();
	if (std::fabs(lRate - static_cast<double>(ReceiveRate)) > 0.05)
	{
		ReceiveRate = lRate;
	}
	const int lLost = mHardware.GetLostPackets();
	if (lLost != static_cast<int>(LostPackets))
	{
		LostPackets = lLost;
	}

	const int64_t lTimeout = static_cast<int64_t>(std::max(static_cast<int>(Timeout), 0)) * 1000000LL;
	const int64_t lNow = NowNanoseconds();
	const int64_t lLastPose = mLastPoseTime;

	EStreamState lState;
	if (lLastPose != 0)
	{
		lState = (lNow - lLastPose <= lTimeout) ? EStreamState::Receiving : EStreamState::NoData;
	}
	else
	{
		// nothing received since the start, give the stream the same time to show up
		lState = (lNow - mStartTime <= lTimeout) ? EStreamState::Waiting : EStreamState::NoData;
	}

	if (lState == mStreamState)
	{
		return;
	}
	mStreamState = lState;

	char lStatus[64];
	switch (lState)
	{
		case EStreamState::Receiving:
			Status = "Receiving data";
			break;

		case EStreamState::NoData:
			snprintf(lStatus, sizeof(lStatus), "No data on UDP port %d", static_cast<int>(Port));
			Status = lStatus;
			break;

		default:
			snprintf(lStatus, sizeof(lStatus), "Listening on UDP port %d", static_cast<int>(Port));
			Status = lStatus;
			break;
	}
}


/************************************************
 *	Record a frame of the device (recording).
 ************************************************/
void CDevice_OSCCamera::DeviceRecordFrame( FBDeviceNotifyInfo &pDeviceNotifyInfo, const CameraOutput& pOutput, double pAgeSeconds )
{
	FBPlayerControl& playerControl = FBPlayerControl::TheOne();

	// keys are added only while playing, a node that is not recording returns no animation to record
	if (playerControl.GetTransportMode() != kFBTransportPlay)
	{
		return;
	}

	CameraOutput lOutput = pOutput;

	// the stream has no timestamps, the pose arrived a moment before this device notify
	FBTime lTime = pDeviceNotifyInfo.GetLocalTime();
	lTime.SetSecondDouble(lTime.GetSecondDouble() - std::min(std::max(pAgeSeconds, 0.0), 0.5));

	bool lUseTime = true;
	switch (SamplingMode.AsInt())
	{
		case kFBHardwareTimestamp:
		case kFBSoftwareTimestamp:
			lUseTime = true;
			break;

		case kFBHardwareFrequency:
		case kFBAutoFrequency:
			lUseTime = false;
			break;
	}

	auto lAddKey = [&](FBAnimationNode* pNode, double* pData)
	{
		if (FBAnimationNode* lData = pNode->GetAnimationToRecord())
		{
			if (lUseTime)
			{
				lData->KeyAdd(lTime, pData);
			}
			else
			{
				lData->KeyAdd(pData);
			}
		}
	};

	lAddKey(mNodeCamera_T, lOutput.position);
	lAddKey(mNodeCamera_R, lOutput.rotation);
	lAddKey(mNodeCamera_FocalLength, &lOutput.focalLength);
	lAddKey(mNodeCamera_FocusDistance, &lOutput.focusDistance);
	lAddKey(mNodeIris, &lOutput.iris);
	lAddKey(mNodeCamera_FocusAngle, &lOutput.focusAngle);
}


/************************************************
 *	Prepare the bound camera, so the lens is driven by the device.
 ************************************************/
bool CDevice_OSCCamera::ModelTemplateBindNotify(FBModel* pModel, int pIndex, FBModelTemplate* pModelTemplate)
{
	if (pModelTemplate == mTemplateCamera)
	{
		ConnectCameraLens(pModel);
	}
	else if (pModelTemplate == mTemplateRoot && pModel != nullptr)
	{
		pModel->Translation.SetAnimated(true);
	}

	return true;
}


/************************************************
 *	Plug the lens nodes into the camera properties.
 ************************************************/
void CDevice_OSCCamera::ConnectCameraLens(FBModel* pModel)
{
	if (pModel == nullptr || !FBIS(pModel, FBCamera))
	{
		return;
	}

	FBCamera* lCamera = static_cast<FBCamera*>(pModel);

	// the focal length (with the film size) defines the field of view, the specific distance is the focus
	lCamera->ApertureMode = kFBApertureFocalLength;
	lCamera->FocusDistanceSource = kFBFocusDistanceSpecificDistance;

	lCamera->FocalLength.SetAnimated(true);
	lCamera->FocusSpecificDistance.SetAnimated(true);
	lCamera->FocusAngle.SetAnimated(true);

	auto lConnect = [](FBAnimationNode* pSource, FBAnimationNode* pTarget)
	{
		if (pSource == nullptr || pTarget == nullptr)
		{
			return;
		}

		// already plugged in (a second connection would only add a duplicate source)
		for (int i = 0, lCount = pTarget->GetSrcCount(); i < lCount; ++i)
		{
			if (pTarget->GetSrc(i) == pSource)
			{
				return;
			}
		}

		if (!FBConnect(pSource, pTarget))
		{
			FBTrace("OSC Camera: failed to connect the lens node\n");
		}
	};

	lConnect(mNodeCamera_FocalLength, lCamera->FocalLength.GetAnimationNode());
	lConnect(mNodeCamera_FocusDistance, lCamera->FocusSpecificDistance.GetAnimationNode());
	lConnect(mNodeCamera_FocusAngle, lCamera->FocusAngle.GetAnimationNode());
}


/************************************************
 *	Settings for the conversion, from the device properties.
 ************************************************/
OSCCamera::ConvertSettings CDevice_OSCCamera::GetConvertSettings()
{
	OSCCamera::ConvertSettings lSettings;

	lSettings.spaceScale = SpaceScale;
	lSettings.focusAngleScale = FocusAngleScale;

	FBVector3d lVector3 = PositionSign;
	for (int i = 0; i < 3; ++i)
	{
		lSettings.positionSign[i] = lVector3[i];
	}
	lVector3 = RotationSign;
	for (int i = 0; i < 3; ++i)
	{
		lSettings.rotationSign[i] = lVector3[i];
	}

	FBVector2d lRange = ZoomRawRange;
	lSettings.zoomRaw = { lRange[0], lRange[1] };
	lRange = ZoomOutputRange;
	lSettings.zoomOut = { lRange[0], lRange[1] };
	lRange = FocusRawRange;
	lSettings.focusRaw = { lRange[0], lRange[1] };
	lRange = FocusOutputRange;
	lSettings.focusOut = { lRange[0], lRange[1] };
	lRange = IrisRawRange;
	lSettings.irisRaw = { lRange[0], lRange[1] };
	lRange = IrisOutputRange;
	lSettings.irisOut = { lRange[0], lRange[1] };

	return lSettings;
}


/************************************************
 *	Convert a received pose to the camera data (device IO thread).
 ************************************************/
CDevice_OSCCamera::CameraOutput CDevice_OSCCamera::ConvertPose(const OSCCamera::PoseMessage& pPose, const OSCCamera::ConvertSettings& pSettings, const OSCCamera::LensMessage* pLens)
{
	const OSCCamera::CameraState lState = OSCCamera::Convert(pPose, pLens, pSettings);

	CameraOutput lOutput;

	for (int i = 0; i < 3; ++i)
	{
		lOutput.position[i] = lState.position[i];
	}

	// the camera axes are the rows of the rotation matrix (MotionBuilder uses row vectors: local axis -> world),
	// convert them to euler angles
	FBMatrix lMatrix;
	lMatrix.Identity();
	for (int i = 0; i < 3; ++i)
	{
		lMatrix[i] = lState.forward[i];		// local X
		lMatrix[4 + i] = lState.up[i];		// local Y
		lMatrix[8 + i] = lState.right[i];	// local Z
	}

	FBRVector lRotation;
	FBMatrixToRotation(lRotation, lMatrix);

	for (int i = 0; i < 3; ++i)
	{
		lOutput.rotation[i] = lRotation[i];

		// keep the angles continuous, otherwise the keys jump by 360 when an angle crosses +-180
		if (mHasLastRotation)
		{
			lOutput.rotation[i] = OSCCamera::UnwrapAngle(lOutput.rotation[i], mLastRotation[i]);
		}
		mLastRotation[i] = lOutput.rotation[i];
	}
	mHasLastRotation = true;

	lOutput.focalLength = lState.focalLength;
	lOutput.focusDistance = lState.focusDistance;
	lOutput.focusAngle = lState.focusAngle;
	lOutput.iris = lState.iris;

	return lOutput;
}


/************************************************
 *	Publish the camera data to the evaluation.
 ************************************************/
void CDevice_OSCCamera::PublishOutput(const CameraOutput& pOutput)
{
	std::lock_guard<std::mutex> lLock(mOutputMutex);
	mOutput = pOutput;
}
