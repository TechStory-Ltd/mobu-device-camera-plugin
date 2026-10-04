
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

//--- Class declaration
#include "device_osccamera_hardware.h"
#include "osc_camera_math.h"

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

	// statistics, read only
	FBPropertyDouble			ReceiveRate;	//!< poses received per second (the last second)
	FBPropertyInt				LostPackets;	//!< frames missing in the stream, counted from the app diagnostics sequence (turn Stream diagnostics on in the app)

	FBPropertyDouble			FocusAngleScale;	//!< multiplier of the focus angle (depth of field cone) computed from the iris, focal length and focus distance
	FBPropertyDouble			SpaceScale;		//!< wire position units to centimetres, 100 - the app sends metres, 1 - the app's Space scale is 100
	FBPropertyVector3d			PositionSign;	//!< per axis flip of the wire position (x right, y forward, z up)
	FBPropertyVector3d			RotationSign;	//!< per angle flip of the wire pan, tilt, roll

	// lens channels that are not calibrated in the app are remapped from the raw encoder range to the output range,
	// calibrated channels are taken as physical values, the ranges are not used
	FBPropertyVector2d			ZoomRawRange;
	FBPropertyVector2d			ZoomOutputRange;		//!< focal length, mm
	FBPropertyVector2d			FocusRawRange;
	FBPropertyVector2d			FocusOutputRange;		//!< focus distance, cm
	FBPropertyVector2d			IrisRawRange;
	FBPropertyVector2d			IrisOutputRange;		//!< f-number

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

	enum class EStreamState : uint8_t
	{
		Waiting,		//!< online, no pose received yet
		Receiving,
		NoData			//!< the stream is silent for longer than the timeout
	};

	static int64_t				NowNanoseconds();	//!< monotonic clock, the same one for the IO and the main thread

	//! prepare the camera lens properties and plug the focal length and focus nodes into them
	void						ConnectCameraLens(FBModel* pModel);

	OSCCamera::ConvertSettings	GetConvertSettings();

	//! convert a received pose to the camera data, device IO thread
	CameraOutput				ConvertPose(const OSCCamera::PoseMessage& pPose, const OSCCamera::ConvertSettings& pSettings, const OSCCamera::LensMessage* pLens);
	void						PublishOutput(const CameraOutput& pOutput);		//!< make the data available for the evaluation

	//! add a key of a pose that arrived pAgeSeconds before this device notify
	void						DeviceRecordFrame(FBDeviceNotifyInfo& pDeviceNotifyInfo, const CameraOutput& pOutput, double pAgeSeconds);

private:
	CDevice_OSCCamera_Hardware	mHardware;			//!< Hardware member.

	// stream liveness: the IO thread stamps the arrival of a pose, the main thread (UI idle) turns it into the Status
	std::atomic<int64_t>		mLastPoseTime{ 0 };		//!< NowNanoseconds() of the last pose, 0 - none since start
	int64_t						mStartTime{ 0 };		//!< NowNanoseconds() of the start
	EStreamState				mStreamState{ EStreamState::Waiting };	//!< main thread only

	// the conversion runs on the device IO thread, the animation nodes are written by the evaluation thread
	std::mutex					mOutputMutex;
	CameraOutput				mOutput;
	bool						mHasLastRotation{ false };
	double						mLastRotation[3]{ 0.0, 0.0, 0.0 };	//!< previous euler, IO thread only, to keep the angles continuous
};
