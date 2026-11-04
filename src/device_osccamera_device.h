
#pragma once

/**	\file	device_osccamera_device.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*/

//--- SDK include
#include <fbsdk/fbsdk.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

//--- Class declaration
#include "device_osccamera_hardware.h"
#include "osc_camera_math.h"
#include "osc_marker_link.h"

//--- Registration defines
#define CDEVICEOSCCAMERA__CLASSNAME		CDevice_OSCCamera
#define CDEVICEOSCCAMERA__CLASSSTR		"CDevice_OSCCamera"

//! Camera tracker device - receives OSC over UDP and drives a camera (transform and lens).
class CDevice_OSCCamera : public FBDevice
{
	//--- FiLMBOX declaration
	FBDeviceDeclare(CDevice_OSCCamera, FBDevice);
public:
	//--- FiLMBOX Construction/Destruction
	virtual bool FBCreate() override;		//!< FiLMBOX constructor.
	virtual void FBDestroy() override;		//!< FiLMBOX destructor.

	//--- The following will be called by the real-time engine.
	virtual bool AnimationNodeNotify	(	FBAnimationNode* pAnimationNode,	FBEvaluateInfo* pEvaluateInfo			) override;	//!< Real-time evaluation for node.
	virtual void DeviceIONotify			(	kDeviceIOs  pAction,				FBDeviceNotifyInfo &pDeviceNotifyInfo	) override;	//!< Notification of/for Device IO.
	virtual bool DeviceEvaluationNotify	(	kTransportMode pMode,				FBEvaluateInfo* pEvaluateInfo			) override;	//!< Evaluation the device (write to hardware).
	virtual bool DeviceOperation		(	kDeviceOperations pOperation												) override;	//!< Operate device.

	//! the camera gets bound to the template - prepare its lens properties to be driven by the device
	virtual bool ModelTemplateBindNotify(FBModel* pModel, int pIndex, FBModelTemplate* pModelTemplate) override;

	//! main thread: shows in the device Status whether the stream is alive
	void		OnUIIdleEvent(HIRegister pSender, HKEvent pEvent);

	//! a scene saved with the device online starts it while the file is still loading, before the camera is bound: start it again
	void		OnFileOpenCompletedEvent(HIRegister pSender, HKEvent pEvent);

	//--- Initialisation/Shutdown
	bool		Init();			//!< Initialize/create device.
	bool		Start();		//!< Start device (online).
	bool		Reset();		//!< Reset device.
	bool		Stop();			//!< Stop device (offline).
	bool		Done();			//!< Remove device.

public:

	FBPropertyInt				Port;			//!< UDP port to listen to (the Target Port set in the phone app)
	FBPropertyInt				CameraId;		//!< accept only this camera id from the stream, -1 - any
	FBPropertyBool				LogPackets;		//!< trace the received messages into the MotionBuilder log
	FBPropertyInt				Timeout;		//!< ms without a pose after which the stream is reported as "No data"
	FBPropertyBool				RemoteControl;	//!< act on the transport commands (play, stop, record...) sent by the app's OSC buttons
	FBPropertyString			LastCommand;	//!< the last command received from the app, also a custom one - a script can react to its change

	// the live camera (a recorded take keys every sample at the time it was taken, whatever these are)
	FBPropertyInt				PlaybackMode;	//!< OSCCamera::Playback::Mode: 0 - Auto, the delay follows the jitter of the network; 1 - Fixed, Playback Delay; 2 - Newest, the newest sample as received
	FBPropertyInt				PlaybackDelay;	//!< ms the live camera is shown behind the stream, to interpolate between the samples around it; with Auto the least delay
	FBPropertyInt				MaxAutoDelay;	//!< ms, the most delay Auto may add
	FBPropertyInt				Responsiveness;	//!< OSCCamera::Playback::Responsiveness: 0 - Responsive, 1 - Balanced, 2 - Smooth, how much of the jitter Auto covers
	FBPropertyBool				ApplySensor;	//!< set the film back of the camera to the sensor the app uses

	// the marker link: the objects of the scene the phone can aim at and hold focus on
	FBPropertyBool				MarkerLink;		//!< send the markers to the phone
	FBPropertyInt				LinkPort;		//!< the marker link port of the phone (Settings > Stream > Scene link in the app)
	FBPropertyListObject		Markers;		//!< the models sent as markers
	FBPropertyString			MarkerSettings;	//!< what the table of the device says about the markers (shape, color, size, role...), saved with the scene

	// the roles of the markers on the phone (Focus, Aim, Ref are set per marker in the table, in any combination), OSC_PROTOCOL.md section 10.10
	FBPropertyBool				RefFollowHeading;	//!< the Ref marker passes on its heading too (a camera on a vehicle), off - only its position (an offset)
	FBPropertyBool				SetLookAt;		//!< the device sets the look-at strength, off - the operator chooses it on the phone
	FBPropertyInt				LookAt;			//!< %, how far the phone turns the camera toward the Aim marker
	FBPropertyBool				MirrorPhone;	//!< a role the operator picks on the phone is taken into the device too, so both ends agree
	FBPropertyString			PhoneRoles;		//!< what the phone says about the roles, read only

	// the camera relative to its Ref marker, OSC_PROTOCOL.md section 2.8
	FBPropertyBool				ReferenceLocal;	//!< the live camera is the pose relative to the Ref marker, placed by the Ref as it is in the scene now; off - the world pose of the stream
	FBPropertyString			CameraSource;	//!< which pose the live camera follows and why, read only

	// statistics, read only
	FBPropertyDouble			Jitter;			//!< ms, the spread of the network delay seen over the last seconds (largest - smallest)
	FBPropertyDouble			DelayInUse;		//!< ms, the delay the live camera is drawn with now
	FBPropertyInt				LostPackets;	//!< frames missing in the stream, counted from the app diagnostics sequence (turn Stream diagnostics on in the app)
	FBPropertyVector2d			SensorSize;		//!< mm, the sensor of the app as sent by the stream
	FBPropertyString			PhoneAddress;	//!< where the poses come from
	FBPropertyString			PhoneLink;		//!< what the phone says about the marker link
	FBPropertyString			StreamUnits;	//!< the unit the stream says its positions are in (metres, converted to the scene's centimetres here), and the phone's own Space scale

	FBPropertyDouble			FocusAngleScale;	//!< multiplier of the focus angle (depth of field cone) computed from the iris, focal length and focus distance
	FBPropertyDouble			SpaceScale;		//!< an extra multiplier on the position, after the stream's unit (metres) is turned into the scene's centimetres: 1 by default

	// lens channels that are not calibrated in the app are remapped from the raw encoder range to the output range,
	// calibrated channels are taken as physical values, the ranges are not used
	FBPropertyVector2d			ZoomRawRange;
	FBPropertyVector2d			ZoomOutputRange;		//!< focal length, mm
	FBPropertyVector2d			FocusRawRange;
	FBPropertyVector2d			FocusOutputRange;		//!< focus distance, cm
	FBPropertyVector2d			IrisRawRange;
	FBPropertyVector2d			IrisOutputRange;		//!< f-number

public:
	// --- the markers, for the table of the layout (main thread)

	//! the models of the Markers list, nullptr for an entry that is not a model
	int							GetMarkerCount();
	FBModel*					GetMarkerModel(int pIndex);

	//! the settings of every model of the list, in the order of the list
	std::vector<OSCCamera::Marker::Settings>	GetMarkerSettings();
	void						SetMarkerSettings(const std::vector<OSCCamera::Marker::Settings>& pSettings);

	//! add models to the list (a model that is in it already is skipped), they get the default settings; at most 16 are sent
	int							AddMarkerModels(const std::vector<FBModel*>& pModels);
	void						RemoveMarkerAt(int pIndex);

	//! fill the size of a marker from the bounding box of its model
	bool						MeasureMarkerModel(int pIndex, OSCCamera::Marker::Settings& pSettings);

	//! "Send roles to phone": the phone gives every role back to the device's, also those the operator changed on the phone
	void						SendRolesAgain();

	//! the calibration properties (scales, signs, lens ranges) back to the defaults
	void						ResetCalibration();

public:
	FBModelTemplate*			mTemplateRoot;		//!< Root model template.
	FBModelTemplate*			mTemplateCamera;	//!< Camera model template.

	FBAnimationNode*			mNodeCamera_T;			//!< Camera animation node (translation).
	FBAnimationNode*			mNodeCamera_R;			//!< Camera animation node (rotation).
	FBAnimationNode*			mNodeCamera_FocalLength;	//!< Focal length, mm. Connected to the camera by ConnectCameraLens (a template binding matches only the transform).
	FBAnimationNode*			mNodeCamera_FocusDistance;	//!< Focus distance, cm. Connected to the camera by ConnectCameraLens.
	FBAnimationNode*			mNodeIris;				//!< Iris, f-number. Not bound to the camera (it has no such property), for a relation constraint.
	FBAnimationNode*			mNodeCamera_FocusAngle;		//!< Camera focus angle (the depth of field cone), degrees. Computed from the iris. Connected by ConnectCameraLens.

private:
	//! the converted camera data, ready to be written to the animation nodes
	struct CameraOutput
	{
		double position[3]{ 0.0, 0.0, 0.0 };	//!< cm
		double rotation[3]{ 0.0, 0.0, 0.0 };	//!< euler XYZ, degrees
		double focalLength{ 35.0 };				//!< mm
		double focusDistance{ 100.0 };			//!< cm
		double focusAngle{ 0.0 };				//!< degrees
		double iris{ 2.8 };						//!< f-number
	};

	//! keeps the euler angles of a series of outputs continuous, otherwise the keys jump by 360 when an angle crosses +-180
	struct RotationTracker
	{
		bool	has{ false };
		double	last[3]{ 0.0, 0.0, 0.0 };
	};

	//! what the live camera did with the Ref of its last pose (written on the IO thread, shown by the main thread)
	enum EReferenceUse : int
	{
		kReferenceNotUsed = 0,		//!< the pose is a world one
		kReferenceComposed,			//!< the local pose is placed by the Ref marker
		kReferenceDisabled,			//!< a local pose, but Reference-Local is off: the world pose is used
		kReferenceUnresolved		//!< a local pose, but its Ref is not a marker of the Markers list: the world pose is used
	};

	//! a marker of the Markers list as the live camera needs it to find its Ref (the table of the main thread, read by the IO thread)
	struct ReferenceTarget
	{
		std::string							key;
		FBModel*							model{ nullptr };
		OSCCamera::Marker::Settings			settings;
	};

	enum class EStreamState : uint8_t
	{
		Waiting,		//!< online, no pose received yet
		Receiving,
		NoData,			//!< the stream is silent for longer than the timeout
		OutdatedApp		//!< the app sends the stream layout of an older version
	};

	static int64_t				NowNanoseconds();	//!< monotonic clock, the same one for the IO and the main thread

	//! prepare the camera lens properties and plug the focal length and focus nodes into them
	void						ConnectCameraLens(FBModel* pModel);

	OSCCamera::ConvertSettings	GetConvertSettings();
	OSCCamera::Playback::Settings	GetPlaybackSettings();	//!< how the live camera is drawn: Playback Mode, Playback Delay, Max Auto Delay, Responsiveness

	//! the commands queued by the IO thread are executed here, main thread only (UI idle)
	void						ProcessCommands();
	void						ExecuteCommand(OSCCamera::ECommand pCommand);

	//! the numbers shown by the device and the sensor of the app, main thread only (UI idle)
	void						UpdateStatistics(const CDevice_OSCCamera_Hardware::Status& pStatus);
	void						ApplySensorSize(const CDevice_OSCCamera_Hardware::Status& pStatus);
	void						ApplyStreamUnits(const CDevice_OSCCamera_Hardware::Status& pStatus);
	void						SetFilmSize(FBCamera* pCamera, double pWidthMm, double pHeightMm);

	//! The Ref of a local pose placed in the scene, device IO thread. The camera is the pose relative to the marker the phone names (its key),
	//! at the place the marker has now (its evaluated transform): C_world = R_scene * C_local. Anything that cannot be resolved leaves the
	//! enclosed world pose, so the camera never jumps to a made-up place.
	OSCCamera::BlendedPose		PlaceByReference(const OSCCamera::BlendedPose& pPose, FBEvaluateInfo* pEvaluateInfo);

	//! the Markers list and the Reference model for PlaceByReference, main thread only (UI idle)
	void						RefreshReferenceTargets();
	void						ShowCameraSource(const CDevice_OSCCamera_Hardware::Status& pStatus);

	//! the models of the Markers list that are sent: at most kMaxMarkers, the entries that are not a model are skipped
	//! \param pListNames	out, the name of every entry of the list
	//! \param pNames		out, the name of every entry that is a model, at most kMaxMarkers
	//! \param pRows		out, the place in the list of each of those
	void						ListMarkerModels(std::vector<std::string>& pListNames, std::vector<std::string>& pNames, std::vector<size_t>& pRows);

	//! the marker link, main thread only (UI idle)
	void						ProcessMarkers(const CDevice_OSCCamera_Hardware::Status& pStatus);
	void						SendMarkerBye();
	//! \param pRows	out, the row of the table of every state
	bool						BuildMarkerStates(std::vector<OSCCamera::Marker::State>& pStates, std::vector<size_t>* pRows = nullptr);
	void						ProcessRolesReport(const CDevice_OSCCamera_Hardware::Status& pStatus, const std::vector<OSCCamera::Marker::State>& pStates,
									const std::vector<size_t>& pRows, int64_t pNow);
	//! write roles that came from the phone (the slots of \p pMask) into the table and the properties
	void						ApplyPhoneRoles(const OSCCamera::Marker::Roles& pRoles, int pMask, const std::vector<OSCCamera::Marker::State>& pStates,
									const std::vector<size_t>& pRows);
	void						ShowPhoneRoles(const CDevice_OSCCamera_Hardware::Status& pStatus);
	const std::vector<OSCCamera::Marker::Settings>&	ReconciledSettings(const std::vector<std::string>& pNames);

	//! the camera data of a state, the angles continued from the previous output of the tracker
	CameraOutput				ToOutput(const OSCCamera::CameraState& pState, RotationTracker& pTracker);
	void						PublishOutput(const CameraOutput& pOutput);		//!< make the data available for the evaluation

	//! add a key of a pose that arrived pAgeSeconds before this device notify
	void						DeviceRecordFrame(FBDeviceNotifyInfo& pDeviceNotifyInfo, const CameraOutput& pOutput, double pAgeSeconds);

private:
	CDevice_OSCCamera_Hardware	mHardware;			//!< Hardware member.

	// stream liveness: the IO thread stamps the arrival of a pose (the hardware), the main thread (UI idle) turns it into the Status
	int64_t						mStartTime{ 0 };		//!< NowNanoseconds() of the start
	EStreamState				mStreamState{ EStreamState::Waiting };	//!< main thread only

	// the conversion runs on the device IO thread, the animation nodes are written by the evaluation thread
	std::mutex					mOutputMutex;
	CameraOutput				mOutput;

	// a camera relative to its Ref is placed by the Ref in the evaluation (AnimationNodeNotify), not when the pose is read: the Ref is then
	// the one of the same evaluation as the camera, with no frame between them. mOutput is the world pose it falls back to.
	struct LocalCamera
	{
		bool						valid{ false };
		OSCCamera::BlendedPose		pose;
		OSCCamera::LensMessage		lens;
		bool						hasLens{ false };
		OSCCamera::ConvertSettings	settings;
	};
	LocalCamera					mLocalCamera;		//!< under mOutputMutex
	RotationTracker				mEvalRotation;		//!< the camera placed by the Ref, under mOutputMutex
	RotationTracker				mLiveRotation;		//!< the rendered camera, IO thread only
	RotationTracker				mRecordRotation;	//!< the recorded samples, IO thread only

	// the sensor of the app on the camera, main thread only
	FBModel*					mLensModel{ nullptr };		//!< the camera the lens nodes were connected to
	uint32_t					mAppliedLensVersion{ 0 };
	bool						mSensorApplied{ false };
	bool						mFilmUnitsKnown{ false };
	double						mFilmUnitsPerMm{ 1.0 / 25.4 };	//!< the film size properties of the camera are in inches or in millimetres, probed once

	// the Ref of the live camera: filled by the main thread, read by the IO thread
	std::mutex					mReferenceMutex;
	std::vector<ReferenceTarget>	mReferenceTargets;
	FBModel*					mReferenceModel{ nullptr };		//!< the Reference model the camera is placed by (the markers are sent relative to it)
	std::atomic<int>			mReferenceUse{ kReferenceNotUsed };
	int64_t						mReferencesRefreshed{ 0 };
	std::string					mShownCameraSource;

	// the marker link, main thread only
	OSCCamera::Marker::Sender	mMarkerSender;
	bool						mMarkersSending{ false };
	int64_t						mMarkersStarted{ 0 };
	std::string					mCachedSettingsText;
	std::vector<std::string>	mCachedModelNames;
	std::vector<OSCCamera::Marker::Settings>	mCachedSettings;
	bool						mHasCachedSettings{ false };
	std::string					mShownPhoneLink;
	OSCCamera::Marker::RoleSync	mRoleSync;
	uint32_t					mRolesReportsSeen{ 0 };		//!< the report count of the hardware last taken in
	bool						mHasRolesReport{ false };
	OSCCamera::Marker::RolesReport	mRolesReport;			//!< the phone's latest, for the status
	std::string					mShownPhoneRoles;
	std::string					mShownPhoneAddress;
	std::string					mShownStreamUnits;
	std::atomic<double>			mStreamUnitCm{ OSCCamera::kDefaultStreamUnitCm };	//!< centimetres in one unit of the stream's positions, set on the main thread, read when converting
};
