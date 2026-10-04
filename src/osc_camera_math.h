#pragma once

/**	\file	osc_camera_math.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
* Conversion of the decoded OSC camera data (see osc_camera_decoder.h) into MotionBuilder units and frame.
* Header-only, no SDK dependency, so it can be unit tested standalone. The last step - camera axes to euler
* angles - needs the SDK (FBMatrixToRotation) and is done by the device.
*
* Frames
*  wire (the tracker app, "Default" axis mapping): X right, Y forward, Z up, right handed
*  MotionBuilder world:                             X forward, Y up, Z right, right handed, centimetres
*  a MotionBuilder camera looks along its local +X with local +Y up and local +Z right, so the tracker zero pose
*  (pan = tilt = roll = 0) is exactly the zero rotation of a camera, and pan, tilt, roll come out as the rotations
*  about Y, Z, X. Mapping the forward to -Z instead would put every pose on the Y = 90 gimbal lock of the XYZ euler order.
*/

#include "osc_camera_decoder.h"

#include <algorithm>
#include <cmath>

namespace OSCCamera
{
	constexpr double kPi = 3.14159265358979323846;

	struct Range
	{
		double min{ 0.0 };
		double max{ 1.0 };
	};

	//! everything the conversion needs from the device properties
	struct ConvertSettings
	{
		double spaceScale{ 1.0 };			//!< wire position units to centimetres (100 scale in the app)

		double positionSign[3]{ 1.0, 1.0, 1.0 };	//!< per axis flip of the wire position (x, y, z), fixes a mismatching axis mapping
		double rotationSign[3]{ 1.0, 1.0, 1.0 };	//!< per angle flip of the wire pan, tilt, roll

		// a channel that is not calibrated in the app has no physical units, its raw encoder value is remapped
		Range zoomRaw{ 0.0, 100.0 };
		Range zoomOut{ 18.0, 200.0 };		//!< focal length, mm
		Range focusRaw{ 0.0, 100.0 };
		Range focusOut{ 30.0, 1000.0 };		//!< focus distance, cm
		Range irisRaw{ 0.0, 100.0 };
		Range irisOut{ 1.4, 22.0 };			//!< f-number

		double focusAngleScale{ 1.0 };		//!< multiplier of the computed camera focus angle, to taste
	};

	//! camera data in the MotionBuilder frame
	struct CameraState
	{
		double position[3]{ 0.0, 0.0, 0.0 };	//!< world position, cm

		// camera axes in the world, they make the rows of the camera rotation matrix
		double forward[3]{ 1.0, 0.0, 0.0 };	//!< local X
		double up[3]{ 0.0, 1.0, 0.0 };			//!< local Y
		double right[3]{ 0.0, 0.0, 1.0 };		//!< local Z

		double focalLength{ 35.0 };				//!< mm
		double focusDistance{ 100.0 };			//!< cm
		double iris{ 2.8 };						//!< f-number
		double focusAngle{ 0.0 };				//!< degrees, the camera depth of field cone, see ComputeFocusAngle
	};

	inline double DegToRad(const double degrees)
	{
		return degrees * kPi / 180.0;
	}

	//! linear remap of a raw value from \p in to \p out, clamped to the input range
	inline double Remap(const double raw, const Range& in, const Range& out)
	{
		if (in.max == in.min)
		{
			return out.min;
		}
		const double t = std::min(std::max((raw - in.min) / (in.max - in.min), 0.0), 1.0);
		return out.min + t * (out.max - out.min);
	}

	//! keep an euler angle (degrees) continuous - pick the equivalent of \p angle that is closest to \p previous,
	//! otherwise the keys jump by 360 when the angle crosses +-180
	inline double UnwrapAngle(const double angle, const double previous)
	{
		return angle + 360.0 * std::round((previous - angle) / 360.0);
	}

	namespace detail
	{
		inline void Cross(const double a[3], const double b[3], double out[3])
		{
			out[0] = a[1] * b[2] - a[2] * b[1];
			out[1] = a[2] * b[0] - a[0] * b[2];
			out[2] = a[0] * b[1] - a[1] * b[0];
		}

		//! wire frame (x right, y forward, z up) to the MotionBuilder world (x forward, y up, z right)
		//! a cyclic permutation, so it stays a proper rotation (no mirroring)
		inline void WireToWorld(const double w[3], double out[3])
		{
			out[0] = w[1];
			out[1] = w[2];
			out[2] = w[0];
		}
	}

	/// build the camera axes from the wire angles (degrees)
	/// yaw about the world up, then pitch, then roll about the camera's own axes, so there is no gimbal swap past +-90 of yaw.
	/// pan grows when the camera turns right, tilt when it looks up, roll when it rolls right (clockwise seen from behind).
	inline void ComputeCameraAxes(const double pan, const double tilt, const double roll, CameraState& state)
	{
		const double p = DegToRad(pan);
		const double t = DegToRad(tilt);
		const double r = DegToRad(roll);

		const double sp = std::sin(p), cp = std::cos(p);
		const double st = std::sin(t), ct = std::cos(t);
		const double sr = std::sin(r), cr = std::cos(r);

		// wire frame axes after yaw, x right, y forward, z up
		const double fwd1[3] = { sp, cp, 0.0 };
		const double right1[3] = { cp, -sp, 0.0 };
		const double up1[3] = { 0.0, 0.0, 1.0 };

		// pitch about the right axis
		double fwd2[3], up2[3];
		for (int i = 0; i < 3; ++i)
		{
			fwd2[i] = fwd1[i] * ct + up1[i] * st;
			up2[i] = up1[i] * ct - fwd1[i] * st;
		}

		// roll about the forward axis
		double up3[3], right3[3];
		for (int i = 0; i < 3; ++i)
		{
			up3[i] = up2[i] * cr + right1[i] * sr;
			right3[i] = right1[i] * cr - up2[i] * sr;
		}

		detail::WireToWorld(fwd2, state.forward);
		detail::WireToWorld(up3, state.up);
		detail::WireToWorld(right3, state.right);
	}

	/// the angle of the camera depth of field cone: the aperture (focal length / f-number) seen from the focus point.
	/// a wider aperture (a smaller f-number) gives a bigger angle - more blur, a longer lens or a closer focus too.
	/// \param focalLength mm
	/// \param fNumber
	/// \param focusDistance cm
	/// \return degrees
	inline double ComputeFocusAngle(const double focalLength, const double fNumber, const double focusDistance)
	{
		// a degenerate lens or focus gives no blur
		if (focalLength <= 0.0 || fNumber <= 0.0 || focusDistance <= 0.0)
		{
			return 0.0;
		}

		const double apertureDiameter = focalLength / fNumber;			// mm
		const double distance = focusDistance * 10.0;					// cm -> mm

		return 2.0 * std::atan(apertureDiameter / (2.0 * distance)) * 180.0 / kPi;
	}

	/// convert a pose and the lens info into a camera state
	/// \param lens the latest /livetracker/lens message, or nullptr if none has arrived yet - a channel is then treated as not calibrated
	inline CameraState Convert(const PoseMessage& pose, const LensMessage* lens, const ConvertSettings& settings)
	{
		CameraState state;

		// position
		const double wire[3] = {
			settings.spaceScale * settings.positionSign[0] * static_cast<double>(pose.x),
			settings.spaceScale * settings.positionSign[1] * static_cast<double>(pose.y),
			settings.spaceScale * settings.positionSign[2] * static_cast<double>(pose.z) };
		detail::WireToWorld(wire, state.position);

		// rotation
		ComputeCameraAxes(
			settings.rotationSign[0] * static_cast<double>(pose.pan),
			settings.rotationSign[1] * static_cast<double>(pose.tilt),
			settings.rotationSign[2] * static_cast<double>(pose.roll),
			state);

		// lens: physical values are sent multiplied by 1000 (micrometres, millimetres, milli f-stops)
		if (lens != nullptr && lens->zoomCalibrated)
		{
			state.focalLength = static_cast<double>(pose.zoom) * 0.001;			// mm
		}
		else
		{
			state.focalLength = Remap(static_cast<double>(pose.zoom), settings.zoomRaw, settings.zoomOut);
		}
		state.focalLength = std::max(state.focalLength, 1.0);					// a zero focal length is degenerate

		if (lens != nullptr && lens->focusCalibrated)
		{
			state.focusDistance = static_cast<double>(pose.focus) * 0.001 * 100.0;	// mm -> m -> cm
		}
		else
		{
			state.focusDistance = Remap(static_cast<double>(pose.focus), settings.focusRaw, settings.focusOut);
		}

		if (lens != nullptr && lens->irisCalibrated)
		{
			state.iris = static_cast<double>(pose.iris) * 0.001;
		}
		else
		{
			state.iris = Remap(static_cast<double>(pose.iris), settings.irisRaw, settings.irisOut);
		}

		state.focusAngle = settings.focusAngleScale * ComputeFocusAngle(state.focalLength, state.iris, state.focusDistance);

		return state;
	}
}
