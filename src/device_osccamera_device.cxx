
/**	\file	device_osccamera_device.cxx
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- Class declaration
#include "device_osccamera_device.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

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
	FBPropertyPublish(this, RemoteControl, "Remote Control", nullptr, nullptr);
	FBPropertyPublish(this, LastCommand, "Last Command", nullptr, nullptr);
	FBPropertyPublish(this, PlaybackMode, "Playback Mode", nullptr, nullptr);
	FBPropertyPublish(this, PlaybackDelay, "Playback Delay (ms)", nullptr, nullptr);
	FBPropertyPublish(this, MaxAutoDelay, "Max Auto Delay (ms)", nullptr, nullptr);
	FBPropertyPublish(this, Responsiveness, "Responsiveness", nullptr, nullptr);
	FBPropertyPublish(this, ApplySensor, "Apply Sensor Size", nullptr, nullptr);

	FBPropertyPublish(this, MarkerLink, "Marker Link", nullptr, nullptr);
	FBPropertyPublish(this, LinkPort, "Link Port", nullptr, nullptr);
	FBPropertyPublish(this, Markers, "Markers", nullptr, nullptr);
	FBPropertyPublish(this, MarkerSettings, "Marker Settings", nullptr, nullptr);
	FBPropertyPublish(this, RefFollowHeading, "Ref Follows Heading", nullptr, nullptr);
	FBPropertyPublish(this, SetLookAt, "Set Look-at", nullptr, nullptr);
	FBPropertyPublish(this, LookAt, "Look-at (%)", nullptr, nullptr);
	FBPropertyPublish(this, MirrorPhone, "Mirror the Phone", nullptr, nullptr);
	FBPropertyPublish(this, PhoneRoles, "Phone Roles", nullptr, nullptr);
	FBPropertyPublish(this, ReferenceLocal, "Reference-Local Camera", nullptr, nullptr);
	FBPropertyPublish(this, CameraSource, "Camera Source", nullptr, nullptr);

	FBPropertyPublish(this, Jitter, "Jitter (ms)", nullptr, nullptr);
	FBPropertyPublish(this, DelayInUse, "Delay In Use (ms)", nullptr, nullptr);
	FBPropertyPublish(this, LostPackets, "Lost Packets", nullptr, nullptr);
	FBPropertyPublish(this, SensorSize, "Sensor Size (mm)", nullptr, nullptr);
	FBPropertyPublish(this, PhoneAddress, "Phone Address", nullptr, nullptr);
	FBPropertyPublish(this, PhoneLink, "Phone Link", nullptr, nullptr);
	FBPropertyPublish(this, StreamUnits, "Stream Units", nullptr, nullptr);

	FBPropertyPublish(this, FocusAngleScale, "Focus Angle Scale", nullptr, nullptr);
	FBPropertyPublish(this, SpaceScale, "Space Scale", nullptr, nullptr);

	FBPropertyPublish(this, ZoomRawRange, "Zoom Raw Range", nullptr, nullptr);
	FBPropertyPublish(this, ZoomOutputRange, "Zoom Output Range", nullptr, nullptr);
	FBPropertyPublish(this, FocusRawRange, "Focus Raw Range", nullptr, nullptr);
	FBPropertyPublish(this, FocusOutputRange, "Focus Output Range", nullptr, nullptr);
	FBPropertyPublish(this, IrisRawRange, "Iris Raw Range", nullptr, nullptr);
	FBPropertyPublish(this, IrisOutputRange, "Iris Output Range", nullptr, nullptr);

	Port = 4000;
	CameraId = -1;
	LogPackets = false;
	Timeout = 500;
	RemoteControl = true;
	LastCommand = "";

	// the delay follows the jitter of the network: the least that lets the camera be drawn between two real samples
	PlaybackMode = static_cast<int>(OSCCamera::Playback::Mode::Auto);
	PlaybackMode.SetMinMax(0.0, 2.0, true, true);
	PlaybackDelay = 0;
	PlaybackDelay.SetMinMax(0.0, 500.0, true, true);
	MaxAutoDelay = 60;
	MaxAutoDelay.SetMinMax(0.0, 500.0, true, true);
	Responsiveness = static_cast<int>(OSCCamera::Playback::Responsiveness::Balanced);
	Responsiveness.SetMinMax(0.0, 2.0, true, true);
	ApplySensor = true;

	MarkerLink = false;
	LinkPort = 4001;
	MarkerSettings = "";
	Markers.SetFilter(FBModel::GetInternalClassId());
	// the table of the device edits it, saved with the device in the scene
	MarkerSettings.ModifyPropertyFlag(kFBPropertyFlagHideProperty, true);

	// the same defaults as the Blender add-on: the Ref is an offset, the strength is the operator's, the phone is mirrored
	RefFollowHeading = false;
	SetLookAt = false;
	LookAt = 50;
	LookAt.SetMinMax(0.0, 100.0, true, true);
	MirrorPhone = true;
	PhoneRoles = "";
	PhoneRoles.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);

	// like the Blender add-on's Reference-local camera: on by default
	ReferenceLocal = true;
	CameraSource = "";
	CameraSource.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);

	Jitter = 0.0;
	DelayInUse = 0.0;
	LostPackets = 0;
	SensorSize = FBVector2d(0.0, 0.0);
	PhoneAddress = "";
	PhoneLink = "";
	StreamUnits = "";
	LastCommand.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	Jitter.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	DelayInUse.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	LostPackets.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	SensorSize.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	PhoneAddress.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	PhoneLink.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);
	StreamUnits.ModifyPropertyFlag(kFBPropertyFlagReadOnly, true);

	ResetCalibration();

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

	// every pose carries the time its sensor sample was taken, the keys are put at that time
	SamplingMode	= kFBHardwareTimestamp;

	CommType = kFBCommTypeNetworkUDP;

	// by default ready to record
	RecordMode = true;

	// a valid output from the very start - the zero pose
	{
		RotationTracker lTracker;
		PublishOutput(ToOutput(OSCCamera::Convert(OSCCamera::PoseMessage(), nullptr, GetConvertSettings()), lTracker));
	}

	FBSystem& system = FBSystem::TheOne();
	system.OnUIIdle.Add(this, (FBCallback)&CDevice_OSCCamera::OnUIIdleEvent);
	FBApplication::TheOne().OnFileOpenCompleted.Add(this, (FBCallback)&CDevice_OSCCamera::OnFileOpenCompletedEvent);

	return true;
}

/************************************************
 *	FiLMBOX Destructor.
 ************************************************/
void CDevice_OSCCamera::FBDestroy()
{
	FBSystem& system = FBSystem::TheOne();
	system.OnUIIdle.Remove(this, (FBCallback)&CDevice_OSCCamera::OnUIIdleEvent);
	FBApplication::TheOne().OnFileOpenCompleted.Remove(this, (FBCallback)&CDevice_OSCCamera::OnFileOpenCompletedEvent);

	// the phone is told that the link ends
	if (mMarkersSending)
	{
		SendMarkerBye();
		mMarkersSending = false;
	}
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

	HardwareVersionInfo = "OSC Camera Tracker, v1.2";
	Information = "";

	// a camera bound earlier (or loaded with the scene) gets its lens connected
	ConnectCameraLens(mTemplateCamera->Model);

	// the stream liveness is tracked from now on
	mStartTime = NowNanoseconds();
	mStreamState = EStreamState::Waiting;

	// the first output of a stream does not continue the angles of the previous one
	mLiveRotation = RotationTracker();
	mRecordRotation = RotationTracker();

	// the sensor of the app is applied again, the camera may have been changed while offline
	mSensorApplied = false;
	mAppliedLensVersion = 0;

	mMarkersSending = false;

	mReferenceUse = kReferenceNotUsed;
	{
		std::lock_guard<std::mutex> lLock(mOutputMutex);
		mLocalCamera.valid = false;
		mEvalRotation = RotationTracker();
	}
	mShownCameraSource.clear();
	CameraSource = "";

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
	// the phone is told that the link ends, it closes the live scene
	if (mMarkersSending)
	{
		SendMarkerBye();
		mMarkersSending = false;
	}

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

		if (mLocalCamera.valid)
		{
			const OSCCamera::BlendedPose lPose = PlaceByReference(mLocalCamera.pose, pEvaluateInfo);
			if (mReferenceUse == kReferenceComposed)
			{
				lOutput = ToOutput(OSCCamera::ConvertBlended(lPose, mLocalCamera.hasLens ? &mLocalCamera.lens : nullptr, mLocalCamera.settings), mEvalRotation);
			}
		}
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
			// drain the socket: the poses go onto the timeline of the phone, the ticks and the stream are not synchronized,
			//  so there can be none or several new sensor samples
			// the filter and the trace are read by the fetch on this thread, a change in the layout applies at once
			mHardware.SetCameraId(CameraId);
			mHardware.SetVerbose(LogPackets);

			const int lCount = mHardware.FetchData();

			const OSCCamera::ConvertSettings lSettings = GetConvertSettings();
			const OSCCamera::LensMessage* lLens = mHardware.GetActiveLens();
			const int64_t lNow = CDevice_OSCCamera_Hardware::NowNanoseconds();

			// each new sensor sample is recorded at the time it was taken, not at the time the packet happened to arrive
			for (int i = 0; i < lCount; ++i)
			{
				const CDevice_OSCCamera_Hardware::ReceivedPose& lReceived = mHardware.GetReceivedPose(i);

				const CameraOutput lOutput = ToOutput(OSCCamera::Convert(lReceived.pose, lLens, lSettings), mRecordRotation);
				DeviceRecordFrame(pDeviceNotifyInfo, lOutput, static_cast<double>(lNow - lReceived.time) * 1e-9);
				AckOneSampleReceived();
			}

			// the live camera is rendered at every tick, a little behind the stream so it is between two samples
			if (mHardware.HasPose())
			{
				const OSCCamera::Playback::Frame lFrame = mHardware.Render(lNow, GetPlaybackSettings(), lSettings);
				if (lFrame.render.state != OSCCamera::Timeline::RenderState::None)
				{
					// the world pose (also the fallback); a pose relative to the Ref is handed to the evaluation, which places it by the Ref
					PublishOutput(ToOutput(OSCCamera::ConvertBlended(lFrame.render.pose, lLens, lSettings), mLiveRotation));

					std::lock_guard<std::mutex> lLock(mOutputMutex);
					mLocalCamera.valid = lFrame.render.pose.local && ReferenceLocal;
					if (mLocalCamera.valid)
					{
						mLocalCamera.pose = lFrame.render.pose;
						mLocalCamera.hasLens = (lLens != nullptr);
						if (lLens != nullptr)
						{
							mLocalCamera.lens = *lLens;
						}
						mLocalCamera.settings = lSettings;
					}
					else
					{
						mReferenceUse = lFrame.render.pose.local ? kReferenceDisabled : kReferenceNotUsed;
					}
				}
			}
		}
		break;

		default:
			break;
	}
}


/************************************************
 *	A scene was opened (main thread).
 *	A device saved online is started while the file loads: the camera is not bound yet, so its lens is not connected and the sensor
 *	is not applied. The same restart as turning Online off and on, now that the scene is complete.
 ************************************************/
void CDevice_OSCCamera::OnFileOpenCompletedEvent(HIRegister pSender, HKEvent pEvent)
{
	mLensModel = nullptr;
	if (Online)
	{
		Reset();
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

	// a camera bound (or bound again) after the start gets the lens nodes; ModelTemplateBindNotify is not called for every binding
	if (mTemplateCamera->Model != mLensModel)
	{
		ConnectCameraLens(mTemplateCamera->Model);
	}

	// the button commands of the app, the transport is driven from here and never from the device IO thread
	ProcessCommands();

	CDevice_OSCCamera_Hardware::Status lHardwareStatus;
	mHardware.GetStatus(lHardwareStatus);

	// statistics from the IO thread, set only on a change
	UpdateStatistics(lHardwareStatus);
	ApplySensorSize(lHardwareStatus);
	ApplyStreamUnits(lHardwareStatus);
	RefreshReferenceTargets();
	ShowCameraSource(lHardwareStatus);
	ProcessMarkers(lHardwareStatus);

	const int64_t lTimeout = static_cast<int64_t>(std::max(static_cast<int>(Timeout), 0)) * 1000000LL;
	const int64_t lNow = NowNanoseconds();
	const int64_t lLastPose = mHardware.GetLastPoseTime();

	EStreamState lState;
	if (lLastPose != 0)
	{
		lState = (lNow - lLastPose <= lTimeout) ? EStreamState::Receiving : EStreamState::NoData;
	}
	else if (lHardwareStatus.outdatedLayout)
	{
		// the app sends poses, but of a layout that is not read
		lState = EStreamState::OutdatedApp;
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

	char lStatus[96];
	switch (lState)
	{
		case EStreamState::Receiving:
			Status = "Receiving data";
			break;

		case EStreamState::NoData:
			snprintf(lStatus, sizeof(lStatus), "No data on UDP port %d", static_cast<int>(Port));
			Status = lStatus;
			break;

		case EStreamState::OutdatedApp:
			Status = "Old stream layout: update the tracker app";
			break;

		default:
			snprintf(lStatus, sizeof(lStatus), "Listening on UDP port %d", static_cast<int>(Port));
			Status = lStatus;
			break;
	}
}


/************************************************
 *	The numbers of the stream (main thread).
 ************************************************/
void CDevice_OSCCamera::UpdateStatistics(const CDevice_OSCCamera_Hardware::Status& pStatus)
{
	if (std::fabs(pStatus.jitterMs - static_cast<double>(Jitter)) > 0.5)
	{
		Jitter = pStatus.jitterMs;
	}
	if (std::fabs(pStatus.delayMs - static_cast<double>(DelayInUse)) > 0.5)
	{
		DelayInUse = pStatus.delayMs;
	}
	const int lLost = mHardware.GetLostPackets();
	if (lLost != static_cast<int>(LostPackets))
	{
		LostPackets = lLost;
	}

	if (pStatus.hasLens)
	{
		const FBVector2d lSensor = SensorSize;
		if (std::fabs(lSensor[0] - pStatus.lens.sensorWidthMm) > 1e-3 || std::fabs(lSensor[1] - pStatus.lens.sensorHeightMm) > 1e-3)
		{
			SensorSize = FBVector2d(pStatus.lens.sensorWidthMm, pStatus.lens.sensorHeightMm);
		}
	}

	std::string lAddress;
	if (pStatus.hasPhone)
	{
		// FBTCPIP gives the address as it is in the packet: the first number of the address is the lowest byte
		char lText[32];
		snprintf(lText, sizeof(lText), "%u.%u.%u.%u", static_cast<unsigned>(pStatus.phoneAddress & 0xFF), static_cast<unsigned>((pStatus.phoneAddress >> 8) & 0xFF),
			static_cast<unsigned>((pStatus.phoneAddress >> 16) & 0xFF), static_cast<unsigned>((pStatus.phoneAddress >> 24) & 0xFF));
		lAddress = lText;
	}
	if (lAddress != mShownPhoneAddress)
	{
		mShownPhoneAddress = lAddress;
		PhoneAddress = lAddress.c_str();
	}

	// the hints of the stream go to the Information of the device
	std::string lInformation;
	if (pStatus.otherCamera)
	{
		lInformation = "Camera ID " + std::to_string(pStatus.otherCameraId) + " also streams to this port, it is ignored";
	}
	if (lInformation != static_cast<const char*>(Information))
	{
		Information = lInformation.c_str();
	}
}


/************************************************
 *	Execute the commands of the app buttons (main thread).
 ************************************************/
void CDevice_OSCCamera::ProcessCommands()
{
	CDevice_OSCCamera_Hardware::ReceivedCommand lCommand;
	while (mHardware.PopCommand(lCommand))
	{
		// with the remote control off the queue is still emptied - a press must not wait to be executed later
		if (!RemoteControl)
		{
			continue;
		}

		LastCommand = lCommand.text;
		ExecuteCommand(lCommand.command);
	}
}


/************************************************
 *	Drive the transport (main thread).
 *	The commands are explicit (no toggles), so running one twice does no harm. An unknown command (a custom button of
 *	the app) is only reported in LastCommand.
 ************************************************/
void CDevice_OSCCamera::ExecuteCommand(OSCCamera::ECommand pCommand)
{
	FBPlayerControl& lPlayer = FBPlayerControl::TheOne();

	if (pCommand != OSCCamera::ECommand::Unknown && lPlayer.IsLocked())
	{
		FBTrace("OSC Camera: transport is locked, command ignored\n");
		return;
	}

	bool lDone = true;
	switch (pCommand)
	{
		case OSCCamera::ECommand::Play:			lDone = lPlayer.Play();			break;
		case OSCCamera::ECommand::Stop:			lDone = lPlayer.Stop();			break;
		case OSCCamera::ECommand::Record:
			// already recording: nothing to start
			if (!lPlayer.IsRecording)
			{
				lDone = lPlayer.Record();
			}
			break;
		case OSCCamera::ECommand::GotoStart:	lDone = lPlayer.GotoStart();	break;
		case OSCCamera::ECommand::GotoEnd:		lDone = lPlayer.GotoEnd();		break;
		case OSCCamera::ECommand::StepForward:	lDone = lPlayer.StepForward();	break;
		case OSCCamera::ECommand::StepBack:		lDone = lPlayer.StepBackward();	break;
		default:								break;
	}

	if (!lDone)
	{
		FBTrace("OSC Camera: the transport refused the command\n");
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

	// the pose was taken pAgeSeconds before this device notify (the capture time of the phone on our clock),
	//  the timestamp modes key it at that time, the frequency modes at the next sample of the take
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
	mLensModel = pModel;
	if (pModel == nullptr || !FBIS(pModel, FBCamera))
	{
		return;
	}

	FBCamera* lCamera = static_cast<FBCamera*>(pModel);

	// the rotation is the stable XYZ euler of the camera frame (the tracker zero pose is zero), the pre-rotation turns it
	// into the scene, where the camera looks along -Z as in the app's FBX export (see osc_camera_math.h)
	lCamera->RotationActive = true;
	lCamera->RotationOrder = kFBEulerXYZ;
	lCamera->PreRotation = FBVector3d(0.0, OSCCamera::kCameraPreRotationY, 0.0);

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

	// a newly bound camera takes the sensor of the app
	mSensorApplied = false;
}


/************************************************
 *	Settings for the conversion, from the device properties.
 ************************************************/
/************************************************
 *	How the live camera is drawn, from the properties.
 ************************************************/
OSCCamera::Playback::Settings CDevice_OSCCamera::GetPlaybackSettings()
{
	OSCCamera::Playback::Settings lSettings;
	lSettings.mode = static_cast<OSCCamera::Playback::Mode>(std::min(std::max(static_cast<int>(PlaybackMode), 0), 2));
	lSettings.responsiveness = static_cast<OSCCamera::Playback::Responsiveness>(std::min(std::max(static_cast<int>(Responsiveness), 0), 2));
	lSettings.delay = static_cast<int64_t>(std::max(static_cast<int>(PlaybackDelay), 0)) * 1000000LL;
	lSettings.maxDelay = static_cast<int64_t>(std::max(static_cast<int>(MaxAutoDelay), 0)) * 1000000LL;
	return lSettings;
}


OSCCamera::ConvertSettings CDevice_OSCCamera::GetConvertSettings()
{
	OSCCamera::ConvertSettings lSettings;

	lSettings.streamUnitCm = mStreamUnitCm;
	lSettings.spaceScale = SpaceScale;
	lSettings.focusAngleScale = FocusAngleScale;

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
 *	The calibration back to the defaults, the same as in OSCCamera::ConvertSettings.
 ************************************************/
void CDevice_OSCCamera::ResetCalibration()
{
	const OSCCamera::ConvertSettings lDefaults;

	FocusAngleScale = lDefaults.focusAngleScale;
	SpaceScale = lDefaults.spaceScale;

	ZoomRawRange = FBVector2d(lDefaults.zoomRaw.min, lDefaults.zoomRaw.max);
	ZoomOutputRange = FBVector2d(lDefaults.zoomOut.min, lDefaults.zoomOut.max);
	FocusRawRange = FBVector2d(lDefaults.focusRaw.min, lDefaults.focusRaw.max);
	FocusOutputRange = FBVector2d(lDefaults.focusOut.min, lDefaults.focusOut.max);
	IrisRawRange = FBVector2d(lDefaults.irisRaw.min, lDefaults.irisRaw.max);
	IrisOutputRange = FBVector2d(lDefaults.irisOut.min, lDefaults.irisOut.max);
}


/************************************************
 *	Convert a received pose to the camera data (device IO thread).
 ************************************************/
CDevice_OSCCamera::CameraOutput CDevice_OSCCamera::ToOutput(const OSCCamera::CameraState& pState, RotationTracker& pTracker)
{
	CameraOutput lOutput;

	for (int i = 0; i < 3; ++i)
	{
		lOutput.position[i] = pState.position[i];
	}

	// the camera axes are the rows of the rotation matrix (MotionBuilder uses row vectors: local axis -> world),
	// convert them to euler angles
	FBMatrix lMatrix;
	lMatrix.Identity();
	for (int i = 0; i < 3; ++i)
	{
		lMatrix[i] = pState.forward[i];		// local X
		lMatrix[4 + i] = pState.up[i];		// local Y
		lMatrix[8 + i] = pState.right[i];	// local Z
	}

	FBRVector lRotation;
	FBMatrixToRotation(lRotation, lMatrix);

	for (int i = 0; i < 3; ++i)
	{
		lOutput.rotation[i] = lRotation[i];

		// keep the angles continuous, otherwise the keys jump by 360 when an angle crosses +-180
		if (pTracker.has)
		{
			lOutput.rotation[i] = OSCCamera::UnwrapAngle(lOutput.rotation[i], pTracker.last[i]);
		}
		pTracker.last[i] = lOutput.rotation[i];
	}
	pTracker.has = true;

	lOutput.focalLength = pState.focalLength;
	lOutput.focusDistance = pState.focusDistance;
	lOutput.focusAngle = pState.focusAngle;
	lOutput.iris = pState.iris;

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


/************************************************
 *	The sensor of the app on the camera (main thread).
 *	The focal length gives the same field of view as in the app only on the same film back.
 ************************************************/
/************************************************
 *	The unit of the stream's positions. The pose is defined as metres, so that is the unit until the stream says it; the scene is
 *	in centimetres, so the conversion is the unit of the stream in centimetres (100 for metres), times the Space Scale property.
 ************************************************/
void CDevice_OSCCamera::ApplyStreamUnits(const CDevice_OSCCamera_Hardware::Status& pStatus)
{
	double lUnitCm = OSCCamera::kDefaultStreamUnitCm;
	std::string lText = "m (assumed)";

	if (pStatus.unitsHeard)
	{
		const double lKnown = OSCCamera::CentimetresPerUnit(pStatus.units.unit);
		char lGain[32] = "";
		if (std::fabs(pStatus.units.spaceScale - 1.0f) > 1e-3f)
		{
			snprintf(lGain, sizeof(lGain), ", phone x%g", static_cast<double>(pStatus.units.spaceScale));
		}

		if (lKnown > 0.0)
		{
			lUnitCm = lKnown;
			lText = std::string(pStatus.units.unit) + lGain;
		}
		else
		{
			lText = std::string("unknown '") + pStatus.units.unit + "', m assumed" + lGain;
		}
	}

	mStreamUnitCm = lUnitCm;
	if (lText != mShownStreamUnits)
	{
		mShownStreamUnits = lText;
		StreamUnits = lText.c_str();
	}
}


void CDevice_OSCCamera::ApplySensorSize(const CDevice_OSCCamera_Hardware::Status& pStatus)
{
	if (!ApplySensor)
	{
		mSensorApplied = false;
		return;
	}

	if (!pStatus.hasLens || (mSensorApplied && pStatus.lensVersion == mAppliedLensVersion))
	{
		return;
	}

	if (!OSCCamera::IsUsableSensor(pStatus.lens.sensorWidthMm, pStatus.lens.sensorHeightMm))
	{
		return;
	}

	FBModel* lModel = mTemplateCamera->Model;
	if (lModel == nullptr || !FBIS(lModel, FBCamera))
	{
		return;
	}

	SetFilmSize(static_cast<FBCamera*>(lModel), pStatus.lens.sensorWidthMm, pStatus.lens.sensorHeightMm);
	mAppliedLensVersion = pStatus.lensVersion;
	mSensorApplied = true;
}

void CDevice_OSCCamera::SetFilmSize(FBCamera* pCamera, double pWidthMm, double pHeightMm)
{
	// the film size properties are in inches or in millimetres, depending on the MotionBuilder version: a standard film back says which
	if (!mFilmUnitsKnown)
	{
		pCamera->FilmBackType = kFBFilmBack35mmAcademy;		// 0.864 inch x 0.63 inch = 21.95 mm x 16 mm
		const double lAcademyWidth = pCamera->FilmSizeWidth;
		mFilmUnitsPerMm = (lAcademyWidth > 0.0 && lAcademyWidth < 5.0) ? (1.0 / 25.4) : 1.0;
		mFilmUnitsKnown = true;
	}

	pCamera->FilmBackType = kFBFilmBackCustom;
	pCamera->FilmSizeWidth = pWidthMm * mFilmUnitsPerMm;
	pCamera->FilmSizeHeight = pHeightMm * mFilmUnitsPerMm;
}


/************************************************
 *	The models of the Markers list (main thread).
 ************************************************/
int CDevice_OSCCamera::GetMarkerCount()
{
	return Markers.GetCount();
}

FBModel* CDevice_OSCCamera::GetMarkerModel(int pIndex)
{
	FBComponent* lComponent = (pIndex >= 0 && pIndex < Markers.GetCount()) ? Markers.GetAt(pIndex) : nullptr;
	return (lComponent != nullptr && FBIS(lComponent, FBModel)) ? static_cast<FBModel*>(lComponent) : nullptr;
}

static std::string ModelName(FBModel* pModel)
{
	return (pModel != nullptr) ? std::string(pModel->Name.AsString()) : std::string();
}

/// The settings of the table for the current list. They are read from the saved text again only when the text or the list changed.
const std::vector<OSCCamera::Marker::Settings>& CDevice_OSCCamera::ReconciledSettings(const std::vector<std::string>& pNames)
{
	const std::string lText = static_cast<const char*>(MarkerSettings);
	if (!mHasCachedSettings || lText != mCachedSettingsText || pNames != mCachedModelNames)
	{
		mCachedSettings = OSCCamera::Marker::Reconcile(OSCCamera::Marker::ParseSettings(lText), pNames);
		mCachedSettingsText = lText;
		mCachedModelNames = pNames;
		mHasCachedSettings = true;
	}
	return mCachedSettings;
}

std::vector<OSCCamera::Marker::Settings> CDevice_OSCCamera::GetMarkerSettings()
{
	std::vector<std::string> lNames;
	for (int i = 0, lCount = GetMarkerCount(); i < lCount; ++i)
	{
		lNames.push_back(ModelName(GetMarkerModel(i)));
	}
	return ReconciledSettings(lNames);
}

void CDevice_OSCCamera::SetMarkerSettings(const std::vector<OSCCamera::Marker::Settings>& pSettings)
{
	// the model name of a row is the hint that finds it again when the list changes
	std::vector<OSCCamera::Marker::Settings> lRows = pSettings;
	for (size_t i = 0; i < lRows.size(); ++i)
	{
		lRows[i].model = ModelName(GetMarkerModel(static_cast<int>(i)));
	}

	const std::string lText = OSCCamera::Marker::SerializeSettings(lRows);
	if (lText != static_cast<const char*>(MarkerSettings))
	{
		MarkerSettings = lText.c_str();
	}
}

int CDevice_OSCCamera::AddMarkerModels(const std::vector<FBModel*>& pModels)
{
	// the settings of the models that are in the list are kept by the model, the new ones start with the defaults
	std::vector<OSCCamera::Marker::Settings> lRows = GetMarkerSettings();

	int lAdded = 0;
	for (FBModel* lModel : pModels)
	{
		if (lModel == nullptr || lModel == mTemplateRoot->Model || lModel == mTemplateCamera->Model)
		{
			continue;
		}

		bool lInList = false;
		for (int i = 0, lCount = GetMarkerCount(); i < lCount; ++i)
		{
			lInList = lInList || (GetMarkerModel(i) == lModel);
		}
		if (lInList)
		{
			continue;
		}

		if (Markers.Add(lModel) >= 0)
		{
			lRows.push_back(OSCCamera::Marker::DefaultSettings(ModelName(lModel), static_cast<int>(lRows.size())));
			++lAdded;
		}
	}

	SetMarkerSettings(lRows);
	return lAdded;
}

void CDevice_OSCCamera::RemoveMarkerAt(int pIndex)
{
	if (pIndex < 0 || pIndex >= GetMarkerCount())
	{
		return;
	}

	std::vector<OSCCamera::Marker::Settings> lRows = GetMarkerSettings();
	if (static_cast<size_t>(pIndex) < lRows.size())
	{
		lRows.erase(lRows.begin() + pIndex);
	}

	Markers.RemoveAt(pIndex);
	SetMarkerSettings(lRows);
}

/// The size of the model, from its bounding box: along its X the width, along its Z the depth, along its Y the height.
bool CDevice_OSCCamera::MeasureMarkerModel(int pIndex, OSCCamera::Marker::Settings& pSettings)
{
	FBModel* lModel = GetMarkerModel(pIndex);
	if (lModel == nullptr)
	{
		return false;
	}

	FBVector3d lMin, lMax, lScaling;
	lModel->GetBoundingBox(lMin, lMax);
	lModel->GetVector(lScaling, kModelScaling, true);

	// the scene is in centimetres
	const double lWidth = std::fabs(lMax[0] - lMin[0]) * std::fabs(lScaling[0]) * 0.01;
	const double lHeight = std::fabs(lMax[1] - lMin[1]) * std::fabs(lScaling[1]) * 0.01;
	const double lDepth = std::fabs(lMax[2] - lMin[2]) * std::fabs(lScaling[2]) * 0.01;
	if (!(lWidth > 0.0) || !(lHeight > 0.0) || !(lDepth > 0.0))
	{
		return false;
	}

	pSettings.width = lWidth;
	pSettings.depth = lDepth;
	pSettings.height = lHeight;
	return true;
}


/************************************************
 *	The marker link (main thread).
 ************************************************/
namespace
{
	//! The inverse of the global transform of the Reference model, identity without one. The camera of the device is placed relative
	//! to the Reference model, so the markers go through its inverse.
	void ReferenceInverseOf(FBModel* pReference, FBMatrix& pInverse, FBEvaluateInfo* pEvaluateInfo)
	{
		pInverse.Identity();
		if (pReference != nullptr)
		{
			FBMatrix lGlobal;
			pReference->GetMatrix(lGlobal, kModelTransformation, true, pEvaluateInfo);
			FBMatrixInverse(pInverse, lGlobal);
		}
	}

	//! A model of the scene as the phone gets it: in the frame of the Reference model, in wire units (OSC_PROTOCOL.md section 10.3).
	//! \param pEvaluateInfo	the evaluation to read the transform of, nullptr - the one of the scene
	OSCCamera::Marker::Position PlaceModel(FBModel* pModel, const FBMatrix& pReferenceInverse, const OSCCamera::Marker::Settings& pSettings,
		const OSCCamera::ConvertSettings& pConvert, FBEvaluateInfo* pEvaluateInfo)
	{
		FBMatrix lGlobal;
		pModel->GetMatrix(lGlobal, kModelTransformation, true, pEvaluateInfo);

		// row vectors (the axes are the rows, the position is the last row): world = local * parent, so local = world * inverse(parent)
		double lRelative[16];
		for (int lRow = 0; lRow < 4; ++lRow)
		{
			for (int lColumn = 0; lColumn < 4; ++lColumn)
			{
				double lSum = 0.0;
				for (int k = 0; k < 4; ++k)
				{
					lSum += lGlobal[lRow * 4 + k] * pReferenceInverse[k * 4 + lColumn];
				}
				lRelative[lRow * 4 + lColumn] = lSum;
			}
		}

		const double lPosition[3] = { lRelative[12], lRelative[13], lRelative[14] };
		double lXAxis[3] = { lRelative[0], lRelative[1], lRelative[2] };
		const double lLength = std::sqrt(lXAxis[0] * lXAxis[0] + lXAxis[1] * lXAxis[1] + lXAxis[2] * lXAxis[2]);
		if (lLength > 1e-12)
		{
			for (double& lComponent : lXAxis)
			{
				lComponent /= lLength;
			}
		}

		return OSCCamera::Marker::Place(lPosition, lXAxis, pSettings, pConvert);
	}
}

void CDevice_OSCCamera::ListMarkerModels(std::vector<std::string>& pListNames, std::vector<std::string>& pNames, std::vector<size_t>& pRows)
{
	// the models of the list, the others are skipped; the table has a row for every entry of the list
	pListNames.clear();
	pNames.clear();
	pRows.clear();
	for (int i = 0, lCount = GetMarkerCount(); i < lCount; ++i)
	{
		pListNames.push_back(ModelName(GetMarkerModel(i)));
	}

	for (size_t i = 0; i < pListNames.size() && pNames.size() < static_cast<size_t>(OSCCamera::Marker::kMaxMarkers); ++i)
	{
		if (GetMarkerModel(static_cast<int>(i)) != nullptr)
		{
			pNames.push_back(pListNames[i]);
			pRows.push_back(i);
		}
	}
}

bool CDevice_OSCCamera::BuildMarkerStates(std::vector<OSCCamera::Marker::State>& pStates, std::vector<size_t>* pRows)
{
	namespace M = OSCCamera::Marker;

	pStates.clear();
	if (pRows != nullptr)
	{
		pRows->clear();
	}

	std::vector<std::string> lListNames;
	std::vector<std::string> lNames;
	std::vector<size_t> lRows;
	ListMarkerModels(lListNames, lNames, lRows);
	const std::vector<M::Settings>& lSettings = ReconciledSettings(lListNames);

	FBMatrix lReferenceInverse;
	ReferenceInverseOf(mTemplateRoot->Model, lReferenceInverse, nullptr);

	const OSCCamera::ConvertSettings lConvert = GetConvertSettings();

	// the key is the model, so it stays the same when the operator renames the marker in the table
	const std::vector<std::string> lKeys = M::MakeKeys(lNames);

	for (size_t k = 0; k < lRows.size(); ++k)
	{
		FBModel* lModel = GetMarkerModel(static_cast<int>(lRows[k]));
		const M::Settings& lRow = lSettings[lRows[k]];

		M::State lState;
		lState.key = lKeys[k];
		lState.name = lRow.name.empty() ? lNames[k] : lRow.name;
		lState.shape = lRow.shape;
		lState.color = lRow.color;
		lState.width = lRow.width;
		lState.depth = lRow.depth;
		lState.height = lRow.height;
		lState.anchor = lRow.anchor;
		lState.visible = lRow.visible;
		lState.roles = lRow.roles;
		lState.position = PlaceModel(lModel, lReferenceInverse, lRow, lConvert, nullptr);
		pStates.push_back(lState);
		if (pRows != nullptr)
		{
			pRows->push_back(lRows[k]);
		}
	}

	return !pStates.empty();
}

/************************************************
 *	The camera relative to its Ref marker.
 ************************************************/
void CDevice_OSCCamera::RefreshReferenceTargets()
{
	namespace M = OSCCamera::Marker;

	// the list changes rarely, a few times a second is enough
	const int64_t lNow = NowNanoseconds();
	if (lNow - mReferencesRefreshed < 100000000LL)
	{
		return;
	}
	mReferencesRefreshed = lNow;

	std::vector<ReferenceTarget> lTargets;
	if (ReferenceLocal && GetMarkerCount() > 0)
	{
		std::vector<std::string> lListNames;
		std::vector<std::string> lNames;
		std::vector<size_t> lRows;
		ListMarkerModels(lListNames, lNames, lRows);
		const std::vector<M::Settings>& lSettings = ReconciledSettings(lListNames);
		const std::vector<std::string> lKeys = M::MakeKeys(lNames);

		for (size_t k = 0; k < lRows.size(); ++k)
		{
			ReferenceTarget lTarget;
			lTarget.key = lKeys[k];
			lTarget.model = GetMarkerModel(static_cast<int>(lRows[k]));
			lTarget.settings = lSettings[lRows[k]];
			lTargets.push_back(std::move(lTarget));
		}
	}

	std::lock_guard<std::mutex> lLock(mReferenceMutex);
	mReferenceTargets = std::move(lTargets);
	mReferenceModel = mTemplateRoot->Model;
}

OSCCamera::BlendedPose CDevice_OSCCamera::PlaceByReference(const OSCCamera::BlendedPose& pPose, FBEvaluateInfo* pEvaluateInfo)
{
	namespace M = OSCCamera::Marker;

	if (!pPose.local)
	{
		mReferenceUse = kReferenceNotUsed;
		return pPose;
	}

	if (!ReferenceLocal)
	{
		mReferenceUse = kReferenceDisabled;
		return pPose;
	}

	// the target is copied, the evaluation of the model is not done under the lock of the main thread
	ReferenceTarget lTarget;
	FBModel* lReference = nullptr;
	bool lFound = false;
	{
		std::lock_guard<std::mutex> lLock(mReferenceMutex);
		for (const ReferenceTarget& lCandidate : mReferenceTargets)
		{
			if (lCandidate.model != nullptr && lCandidate.key == pPose.key)
			{
				lTarget = lCandidate;
				lFound = true;
				break;
			}
		}
		lReference = mReferenceModel;
	}

	if (!lFound)
	{
		mReferenceUse = kReferenceUnresolved;
		return pPose;
	}

	// the same placement the Ref is sent to the phone with: the phone has worked out the local pose against it
	FBMatrix lReferenceInverse;
	ReferenceInverseOf(lReference, lReferenceInverse, pEvaluateInfo);
	const M::Position lPlace = PlaceModel(lTarget.model, lReferenceInverse, lTarget.settings, GetConvertSettings(), pEvaluateInfo);

	OSCCamera::ReferenceFrame lFrame;
	lFrame.position[0] = lPlace.x;
	lFrame.position[1] = lPlace.y;
	lFrame.position[2] = lPlace.z;
	lFrame.yaw = lPlace.yaw;

	mReferenceUse = kReferenceComposed;
	return OSCCamera::ComposeReference(pPose, lFrame);
}

void CDevice_OSCCamera::ShowCameraSource(const CDevice_OSCCamera_Hardware::Status& pStatus)
{
	using Kind = CDevice_OSCCamera_Hardware::Status::EPoseKind;

	std::string lText;
	switch (pStatus.poseKind)
	{
		case Kind::Legacy:
			lText = "World pose (the app sends the former /livetracker/pose)";
			break;

		case Kind::World:
			lText = "World pose";
			break;

		case Kind::RefLocal:
		{
			const std::string lKey = pStatus.poseKey;
			const std::string lFollow = (pStatus.poseFollow == OSCCamera::kFollowHeading) ? "position + heading" : "position";
			switch (mReferenceUse)
			{
				case kReferenceComposed:
					lText = "Relative to Ref '" + lKey + "' (" + lFollow + ")";
					break;

				case kReferenceDisabled:
					lText = "World pose: Reference-Local Camera is off (the phone follows Ref '" + lKey + "')";
					break;

				case kReferenceUnresolved:
					lText = "World pose: Ref '" + lKey + "' is not in the Markers list";
					break;

				default:
					lText = "Relative to Ref '" + lKey + "'";
					break;
			}
			break;
		}

		default:
			break;
	}

	if (lText != mShownCameraSource)
	{
		mShownCameraSource = lText;
		CameraSource = lText.c_str();
	}
}

void CDevice_OSCCamera::SendMarkerBye()
{
	// once or twice, UDP may lose one
	const OSCCamera::Marker::Datagram lBye = OSCCamera::Marker::EncodeBye();
	for (int i = 0; i < 2; ++i)
	{
		mHardware.SendToPhone(lBye.data(), lBye.size(), static_cast<unsigned short>(static_cast<int>(LinkPort)));
	}
}

void CDevice_OSCCamera::ProcessMarkers(const CDevice_OSCCamera_Hardware::Status& pStatus)
{
	namespace M = OSCCamera::Marker;

	auto lShow = [this](const std::string& pText)
	{
		if (pText != mShownPhoneLink)
		{
			mShownPhoneLink = pText;
			PhoneLink = pText.c_str();
		}
	};

	if (!MarkerLink)
	{
		if (mMarkersSending)
		{
			SendMarkerBye();
			mMarkersSending = false;
		}
		lShow("");
		ShowPhoneRoles(pStatus);
		return;
	}

	if (GetMarkerCount() == 0)
	{
		if (mMarkersSending)
		{
			SendMarkerBye();
			mMarkersSending = false;
		}
		lShow("Add models to the Markers list");
		ShowPhoneRoles(pStatus);
		return;
	}

	if (!pStatus.hasPhone)
	{
		lShow("Waiting for the first pose: the phone is found by it");
		ShowPhoneRoles(pStatus);
		return;
	}

	const int64_t lNow = NowNanoseconds();
	if (!mMarkersSending)
	{
		mMarkerSender.Reset();
		mMarkersSending = true;
		mMarkersStarted = lNow;

		// the roles start again with every link, a report from before it says nothing about it
		mRoleSync = M::RoleSync();
		mRolesReportsSeen = pStatus.linkRolesCount;
		mHasRolesReport = false;
	}

	if (mMarkerSender.Due(lNow))
	{
		std::vector<M::State> lStates;
		std::vector<size_t> lRows;
		if (BuildMarkerStates(lStates, &lRows))
		{
			ProcessRolesReport(pStatus, lStates, lRows, lNow);

			// the scene time is the position of the timeline in microseconds
			FBTime lTime = FBSystem::TheOne().LocalTime;
			const int64_t lSceneTimeUs = static_cast<int64_t>(std::llround(lTime.GetSecondDouble() * 1.0e6));

			std::vector<M::Datagram> lOut;
			const M::Datagram lRoles = mRoleSync.Message();
			mMarkerSender.Step(lNow, lStates, lSceneTimeUs, lOut, &lRoles);

			const unsigned short lPort = static_cast<unsigned short>(std::min(std::max(static_cast<int>(LinkPort), 1), 65535));
			for (const M::Datagram& lDatagram : lOut)
			{
				mHardware.SendToPhone(lDatagram.data(), lDatagram.size(), lPort);
			}
		}
	}

	lShow(M::LinkStatusText(pStatus.linkHeard, pStatus.link.state, pStatus.link.markers, static_cast<double>(lNow - mMarkersStarted) * 1e-9));
	ShowPhoneRoles(pStatus);
}


/************************************************
 *	The roles of the markers (main thread), the same rules as the Blender add-on (rolesync.py).
 ************************************************/
void CDevice_OSCCamera::ProcessRolesReport(const CDevice_OSCCamera_Hardware::Status& pStatus, const std::vector<OSCCamera::Marker::State>& pStates,
	const std::vector<size_t>& pRows, int64_t pNow)
{
	namespace M = OSCCamera::Marker;

	// the device's roles now: a role changed in the table or the properties since the last step is the artist's change
	mRoleSync.Local(M::RolesOf(pStates, RefFollowHeading ? M::kRefFollowHeading : M::kRefFollowPosition,
		SetLookAt ? static_cast<int>(LookAt) : M::kLookAtPhone), pNow);

	if (pStatus.linkRolesCount == mRolesReportsSeen)
	{
		return;
	}
	mRolesReportsSeen = pStatus.linkRolesCount;
	if (!pStatus.linkRolesHeard)
	{
		return;
	}

	M::RolesReport lReport;
	lReport.rev = pStatus.linkRoles.rev;
	lReport.roles.focus = pStatus.linkRoles.focus;
	lReport.roles.aim = pStatus.linkRoles.aim;
	lReport.roles.ref = pStatus.linkRoles.ref;
	lReport.roles.refFollow = pStatus.linkRoles.refFollow;
	lReport.roles.lookAt = pStatus.linkRoles.lookAt;
	lReport.overridden = pStatus.linkRoles.overridden;
	mRolesReport = lReport;
	mHasRolesReport = true;

	const int lMask = mRoleSync.Report(lReport, MirrorPhone, pNow);
	if (lMask != 0)
	{
		ApplyPhoneRoles(lReport.roles, lMask, pStates, pRows);
		mRoleSync.Mirrored(lReport.roles, lMask);
	}
}

void CDevice_OSCCamera::ApplyPhoneRoles(const OSCCamera::Marker::Roles& pRoles, int pMask, const std::vector<OSCCamera::Marker::State>& pStates,
	const std::vector<size_t>& pRows)
{
	namespace M = OSCCamera::Marker;

	std::vector<M::Settings> lRows = GetMarkerSettings();

	// the key each row is sent with: a marker may hold two or three roles, every slot is written as it is (section 10.10, step 6)
	std::vector<std::string> lKeys(lRows.size());
	for (size_t k = 0; k < pStates.size() && k < pRows.size(); ++k)
	{
		if (pRows[k] < lKeys.size())
		{
			lKeys[pRows[k]] = pStates[k].key;
		}
	}
	M::MirrorRoles(lRows, lKeys, pRoles, pMask);
	SetMarkerSettings(lRows);

	if (pMask & (1 << M::kSlotFollow))
	{
		RefFollowHeading = (pRoles.refFollow == M::kRefFollowHeading);
	}
	if ((pMask & (1 << M::kSlotLookAt)) && pRoles.lookAt >= 0)
	{
		LookAt = std::min(pRoles.lookAt, 100);
	}
}

void CDevice_OSCCamera::ShowPhoneRoles(const CDevice_OSCCamera_Hardware::Status& pStatus)
{
	std::string lText;
	if (MarkerLink && mMarkersSending)
	{
		if (!mHasRolesReport || !pStatus.linkRolesHeard)
		{
			lText = "The phone reports its roles while its Live scene shows";
		}
		else
		{
			lText = OSCCamera::Marker::RolesStatusText(mRolesReport, mRoleSync.Differs(mRolesReport));
			if (lText.empty())
			{
				lText = "The phone has these roles";
			}
		}
	}

	if (lText != mShownPhoneRoles)
	{
		mShownPhoneRoles = lText;
		PhoneRoles = lText.c_str();
	}
}

void CDevice_OSCCamera::SendRolesAgain()
{
	if (mMarkersSending)
	{
		mRoleSync.SendAgain(NowNanoseconds());
	}
}
