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
*  MotionBuilder scene:                             X right, Y up, Z back (the forward of the tracker is -Z), right handed,
*                                                   centimetres - the frame of the app's FBX export. The positions.
*  camera rotation frame:                           X forward, Y up, Z right - the scene turned -90 about Y. The rotations.
*  a MotionBuilder camera looks along its local +X with local +Y up and local +Z right, so in the rotation frame the tracker
*  zero pose (pan = tilt = roll = 0) is exactly the zero rotation of a camera, and pan, tilt, roll come out as the rotations
*  about Y, Z, X, far from the Y = 90 gimbal lock of the XYZ euler order. The device gives the camera the pre-rotation
*  kCameraPreRotationY about Y, which turns the rotation frame into the scene: the camera then looks along -Z as in the FBX.
*/

#include "osc_camera_decoder.h"

#include <algorithm>
#include <cmath>

namespace OSCCamera
{
	constexpr double kPi = 3.14159265358979323846;
	constexpr double kCameraPreRotationY = 90.0;	//!< degrees, the pre-rotation of the camera: its rotation frame to the scene

	struct Range
	{
		double min{ 0.0 };
		double max{ 1.0 };
	};

	//! everything the conversion needs from the device properties
	struct ConvertSettings
	{
		double streamUnitCm{ kDefaultStreamUnitCm };	//!< centimetres in one unit of the stream's positions: 100, the stream is in metres (the stream names its unit)
		double spaceScale{ 1.0 };			//!< an extra multiplier on the position, after the stream's unit is turned into the scene's centimetres: 1 by default

		//! centimetres in one position unit of the stream times the extra multiplier; the marker positions go the other way through the same factor
		double CentimetresPerWireUnit() const { return streamUnitCm * spaceScale; }

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
		double position[3]{ 0.0, 0.0, 0.0 };	//!< scene position (x right, y up, z back), cm

		// camera axes in the rotation frame (the scene before the camera pre-rotation), they make the rows of the camera rotation matrix
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

		//! wire frame (x right, y forward, z up) to the camera rotation frame (x forward, y up, z right)
		//! a cyclic permutation, so it stays a proper rotation (no mirroring)
		inline void WireToWorld(const double w[3], double out[3])
		{
			out[0] = w[1];
			out[1] = w[2];
			out[2] = w[0];
		}

		//! wire frame (x right, y forward, z up) to the MotionBuilder scene (x right, y up, z back), as the FBX export:
		//! WireToWorld turned by kCameraPreRotationY about Y, still a proper rotation
		inline void WireToScene(const double w[3], double out[3])
		{
			out[0] = w[0];
			out[1] = w[2];
			out[2] = -w[1];
		}

		//! the inverse of WireToScene
		inline void SceneToWire(const double m[3], double out[3])
		{
			out[0] = m[0];
			out[1] = -m[2];
			out[2] = m[1];
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

	namespace detail
	{
		inline void ConvertPosition(const double x, const double y, const double z, const ConvertSettings& settings, CameraState& state)
		{
			// the stream's unit (metres) to the scene's centimetres, then the extra multiplier
			const double lToScene = settings.CentimetresPerWireUnit();
			const double wire[3] = {
				lToScene * x,
				lToScene * y,
				lToScene * z };
			WireToScene(wire, state.position);
		}

		//! lens: physical values are sent multiplied by 1000 (micrometres, millimetres, milli f-stops)
		inline void ConvertLens(const double zoom, const double focus, const double iris, const LensMessage* lens, const ConvertSettings& settings, CameraState& state)
		{
			if (lens != nullptr && lens->zoomCalibrated)
			{
				state.focalLength = zoom * 0.001;			// mm
			}
			else
			{
				state.focalLength = Remap(zoom, settings.zoomRaw, settings.zoomOut);
			}
			state.focalLength = std::max(state.focalLength, 1.0);					// a zero focal length is degenerate

			if (lens != nullptr && lens->focusCalibrated)
			{
				state.focusDistance = focus * 0.001 * 100.0;	// mm -> m -> cm
			}
			else
			{
				state.focusDistance = Remap(focus, settings.focusRaw, settings.focusOut);
			}

			if (lens != nullptr && lens->irisCalibrated)
			{
				state.iris = iris * 0.001;
			}
			else
			{
				state.iris = Remap(iris, settings.irisRaw, settings.irisOut);
			}

			state.focusAngle = settings.focusAngleScale * ComputeFocusAngle(state.focalLength, state.iris, state.focusDistance);
		}
	}

	/// convert a pose and the lens info into a camera state
	/// \param lens the latest /livetracker/lens message of the camera, or nullptr if none has arrived yet - a channel is then treated as not calibrated
	inline CameraState Convert(const PoseMessage& pose, const LensMessage* lens, const ConvertSettings& settings)
	{
		CameraState state;

		detail::ConvertPosition(pose.x, pose.y, pose.z, settings, state);

		ComputeCameraAxes(static_cast<double>(pose.pan), static_cast<double>(pose.tilt), static_cast<double>(pose.roll), state);

		detail::ConvertLens(pose.zoom, pose.focus, pose.iris, lens, settings, state);
		return state;
	}

	// --- blending of two samples (the receive timeline renders between the samples around a time)

	//! a rotation as a unit quaternion
	struct Quat
	{
		double w{ 1.0 };
		double x{ 0.0 };
		double y{ 0.0 };
		double z{ 0.0 };
	};

	/// quaternion of the rotation whose columns are the camera axes (forward, up, right) in the world
	inline Quat QuatFromAxes(const double forward[3], const double up[3], const double right[3])
	{
		// matrix m[row][col] with the axes as columns
		const double m00 = forward[0], m10 = forward[1], m20 = forward[2];
		const double m01 = up[0],      m11 = up[1],      m21 = up[2];
		const double m02 = right[0],   m12 = right[1],   m22 = right[2];

		Quat q;
		const double trace = m00 + m11 + m22;
		if (trace > 0.0)
		{
			const double s = std::sqrt(trace + 1.0) * 2.0;
			q.w = 0.25 * s;
			q.x = (m21 - m12) / s;
			q.y = (m02 - m20) / s;
			q.z = (m10 - m01) / s;
		}
		else if (m00 > m11 && m00 > m22)
		{
			const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
			q.w = (m21 - m12) / s;
			q.x = 0.25 * s;
			q.y = (m01 + m10) / s;
			q.z = (m02 + m20) / s;
		}
		else if (m11 > m22)
		{
			const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
			q.w = (m02 - m20) / s;
			q.x = (m01 + m10) / s;
			q.y = 0.25 * s;
			q.z = (m12 + m21) / s;
		}
		else
		{
			const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
			q.w = (m10 - m01) / s;
			q.x = (m02 + m20) / s;
			q.y = (m12 + m21) / s;
			q.z = 0.25 * s;
		}
		return q;
	}

	/// the camera axes (columns of the rotation matrix) of a unit quaternion
	inline void AxesFromQuat(const Quat& q, double forward[3], double up[3], double right[3])
	{
		const double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
		const double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
		const double wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;

		forward[0] = 1.0 - 2.0 * (yy + zz);	forward[1] = 2.0 * (xy + wz);			forward[2] = 2.0 * (xz - wy);
		up[0] = 2.0 * (xy - wz);			up[1] = 1.0 - 2.0 * (xx + zz);			up[2] = 2.0 * (yz + wx);
		right[0] = 2.0 * (xz + wy);			right[1] = 2.0 * (yz - wx);				right[2] = 1.0 - 2.0 * (xx + yy);
	}

	/// spherical interpolation along the shortest arc
	inline Quat Slerp(const Quat& a, Quat b, const double t)
	{
		double dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
		if (dot < 0.0)
		{
			b = Quat{ -b.w, -b.x, -b.y, -b.z };
			dot = -dot;
		}

		double ka, kb;
		if (dot > 0.9995)
		{
			// nearly the same rotation: a linear blend is accurate and avoids dividing by sin(0)
			ka = 1.0 - t;
			kb = t;
		}
		else
		{
			const double angle = std::acos(dot);
			const double s = std::sin(angle);
			ka = std::sin((1.0 - t) * angle) / s;
			kb = std::sin(t * angle) / s;
		}

		Quat q{ ka * a.w + kb * b.w, ka * a.x + kb * b.x, ka * a.y + kb * b.y, ka * a.z + kb * b.z };
		const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
		if (length > 0.0)
		{
			q.w /= length; q.x /= length; q.y /= length; q.z /= length;
		}
		return q;
	}

	/// the product a * b: the rotation b first, then a (both in the same frame)
	inline Quat QuatMultiply(const Quat& a, const Quat& b)
	{
		Quat q{
			a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
			a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
			a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
			a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w };
		const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
		if (length > 0.0)
		{
			q.w /= length; q.x /= length; q.y /= length; q.z /= length;
		}
		return q;
	}

	//! a pose that can be blended: the wire position, the rotation as a quaternion and the lens integers as numbers
	struct BlendedPose
	{
		double wire[3]{ 0.0, 0.0, 0.0 };	//!< as received, in the unit of the stream (metres, the Space scale gain of the app already in them). The world pose: the one to use when there is no Ref to compose with
		Quat rotation;
		double zoom{ 0.0 };
		double focus{ 0.0 };
		double iris{ 0.0 };

		// the camera relative to its Ref marker, when the sample says so (REF_LOCAL): wire[] and rotation are then the enclosed world pose
		bool local{ false };
		int follow{ kFollowPosition };		//!< EReferenceFollow: what of the Ref the camera takes
		int32_t generation{ 0 };
		char key[kMaxReferenceKeyLength + 1]{};	//!< the marker key of the Ref
		double localWire[3]{ 0.0, 0.0, 0.0 };
		Quat localRotation;
	};

	/// the rotation of the wire angles (degrees)
	inline Quat AnglesToQuat(const double pan, const double tilt, const double roll)
	{
		CameraState axes;
		ComputeCameraAxes(pan, tilt, roll, axes);
		return QuatFromAxes(axes.forward, axes.up, axes.right);
	}

	inline BlendedPose ToBlended(const PoseMessage& pose)
	{
		BlendedPose blended;
		blended.wire[0] = pose.x;
		blended.wire[1] = pose.y;
		blended.wire[2] = pose.z;
		blended.rotation = AnglesToQuat(pose.pan, pose.tilt, pose.roll);

		blended.zoom = pose.zoom;
		blended.focus = pose.focus;
		blended.iris = pose.iris;

		if (IsReferenceLocal(pose))
		{
			blended.local = true;
			blended.follow = pose.follow;
			blended.generation = pose.generation;
			std::memcpy(blended.key, pose.key, sizeof(blended.key));
			for (int i = 0; i < 3; ++i)
			{
				blended.localWire[i] = pose.local[i];
			}
			blended.localRotation = AnglesToQuat(pose.local[3], pose.local[4], pose.local[5]);
		}
		return blended;
	}

	/// \param a, b	samples that can be blended (CanBlend of the messages they come from): both local ones relative to one Ref, or world ones
	inline BlendedPose Blend(const BlendedPose& a, const BlendedPose& b, const double t)
	{
		BlendedPose out;
		for (int i = 0; i < 3; ++i)
		{
			out.wire[i] = a.wire[i] + (b.wire[i] - a.wire[i]) * t;
		}
		out.rotation = Slerp(a.rotation, b.rotation, t);
		out.zoom = a.zoom + (b.zoom - a.zoom) * t;
		out.focus = a.focus + (b.focus - a.focus) * t;
		out.iris = a.iris + (b.iris - a.iris) * t;

		if (a.local && b.local)
		{
			out.local = true;
			out.follow = a.follow;
			out.generation = a.generation;
			std::memcpy(out.key, a.key, sizeof(out.key));
			for (int i = 0; i < 3; ++i)
			{
				out.localWire[i] = a.localWire[i] + (b.localWire[i] - a.localWire[i]) * t;
			}
			out.localRotation = Slerp(a.localRotation, b.localRotation, t);
		}
		return out;
	}

	//! The Ref marker as the phone sees it, in the stream frame (OSC_PROTOCOL.md section 10.3): the same numbers as the marker is sent with.
	struct ReferenceFrame
	{
		double position[3]{ 0.0, 0.0, 0.0 };	//!< wire units, the anchor of the marker (the middle of its base, a point itself)
		double yaw{ 0.0 };						//!< degrees, counter-clockwise from the stream +X, about the vertical
	};

	/// The camera of a sample relative to its Ref in the world: C_world = R_scene * C_local (section 2.8). The Ref passes on its anchor
	/// translation, and only with Follow heading its yaw about the vertical - never its scale or a tilt.
	/// A sample that is not local is returned as it is. The result is a world pose (local is false).
	inline BlendedPose ComposeReference(const BlendedPose& pose, const ReferenceFrame& reference)
	{
		if (!pose.local)
		{
			return pose;
		}

		BlendedPose out = pose;
		out.local = false;

		const bool heading = (pose.follow == kFollowHeading);
		const double yaw = heading ? DegToRad(reference.yaw) : 0.0;
		const double c = std::cos(yaw);
		const double s = std::sin(yaw);

		// counter-clockwise about the wire +Z (up)
		out.wire[0] = reference.position[0] + pose.localWire[0] * c - pose.localWire[1] * s;
		out.wire[1] = reference.position[1] + pose.localWire[0] * s + pose.localWire[1] * c;
		out.wire[2] = reference.position[2] + pose.localWire[2];

		// the same turn in the camera frame of the world: wire +Z is the up (Y) of MotionBuilder, a cyclic permutation keeps the turn's sign
		const Quat turn{ std::cos(0.5 * yaw), 0.0, std::sin(0.5 * yaw), 0.0 };
		out.rotation = QuatMultiply(turn, pose.localRotation);
		return out;
	}

	/// convert a (blended) pose and the lens info into a camera state
	inline CameraState ConvertBlended(const BlendedPose& pose, const LensMessage* lens, const ConvertSettings& settings)
	{
		CameraState state;

		detail::ConvertPosition(pose.wire[0], pose.wire[1], pose.wire[2], settings, state);
		AxesFromQuat(pose.rotation, state.forward, state.up, state.right);
		detail::ConvertLens(pose.zoom, pose.focus, pose.iris, lens, settings, state);
		return state;
	}

	// --- sensor

	/// the sensor size the app sends (millimetres) is a plausible, finite film back
	inline bool IsUsableSensor(const float widthMm, const float heightMm)
	{
		return std::isfinite(widthMm) && std::isfinite(heightMm) && widthMm >= 1.0f && widthMm <= 100.0f && heightMm >= 1.0f && heightMm <= 100.0f;
	}
}
