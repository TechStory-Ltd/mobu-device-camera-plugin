/**	\file	osc_camera_math_test.cpp
*	Standalone test for osc_camera_math.h (no MotionBuilder SDK needed).
*	Build target: device_oscCamera_math_test (excluded from the default build), the exit code is the number of failed checks.
*/

#include "osc_camera_math.h"

#include <cstdio>

static int g_failures = 0;

#define CHECK(cond) \
	do { if (!(cond)) { ++g_failures; std::printf("FAILED %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

static bool Near(double a, double b, double eps = 1e-9)
{
	return std::fabs(a - b) < eps;
}

static bool Near3(const double a[3], double x, double y, double z, double eps = 1e-9)
{
	return Near(a[0], x, eps) && Near(a[1], y, eps) && Near(a[2], z, eps);
}

static OSCCamera::CameraState Axes(double pan, double tilt, double roll)
{
	OSCCamera::CameraState s;
	OSCCamera::ComputeCameraAxes(pan, tilt, roll, s);
	return s;
}

static void TestAxes()
{
	// zero angles: the camera looks along the wire forward, which is the world +X - the zero rotation of a MotionBuilder camera
	OSCCamera::CameraState s = Axes(0, 0, 0);
	CHECK(Near3(s.forward, 1, 0, 0));
	CHECK(Near3(s.up, 0, 1, 0));
	CHECK(Near3(s.right, 0, 0, 1));

	// pan grows when the camera turns right: 90 - looks along the world +Z, which is the right of the zero pose
	s = Axes(90, 0, 0);
	CHECK(Near3(s.forward, 0, 0, 1));
	CHECK(Near3(s.up, 0, 1, 0));
	CHECK(Near3(s.right, -1, 0, 0));

	// pan -90 looks left
	s = Axes(-90, 0, 0);
	CHECK(Near3(s.forward, 0, 0, -1));

	// tilt grows when the camera looks up
	s = Axes(0, 90, 0);
	CHECK(Near3(s.forward, 0, 1, 0));
	CHECK(Near3(s.up, -1, 0, 0));
	CHECK(Near3(s.right, 0, 0, 1));

	s = Axes(0, 30, 0);
	CHECK(s.forward[1] > 0.0);		// world up component

	// roll grows when the camera rolls right: the top of the camera tilts to the right
	s = Axes(0, 0, 90);
	CHECK(Near3(s.forward, 1, 0, 0));
	CHECK(Near3(s.up, 0, 0, 1));
	CHECK(Near3(s.right, 0, -1, 0));

	// yaw is about the world up whatever the pitch: pan 90 + tilt 45 must still turn the heading, not the pitched axis
	s = Axes(90, 45, 0);
	CHECK(Near(s.forward[0], 0.0));
	CHECK(Near(s.forward[1], std::sin(OSCCamera::DegToRad(45))));
	CHECK(Near(s.forward[2], std::cos(OSCCamera::DegToRad(45))));
}

static void TestAxesAreARightHandedRotation()
{
	// for any angles the axes must stay orthonormal and right = forward x up (a proper rotation, no mirroring)
	for (double pan = -180; pan <= 180; pan += 37)
	{
		for (double tilt = -90; tilt <= 90; tilt += 29)
		{
			for (double roll = -180; roll <= 180; roll += 41)
			{
				const OSCCamera::CameraState s = Axes(pan, tilt, roll);

				double cross[3];
				OSCCamera::detail::Cross(s.forward, s.up, cross);
				CHECK(Near3(s.right, cross[0], cross[1], cross[2], 1e-9));

				auto dot = [](const double a[3], const double b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
				CHECK(Near(dot(s.forward, s.forward), 1.0));
				CHECK(Near(dot(s.up, s.up), 1.0));
				CHECK(Near(dot(s.forward, s.up), 0.0));
			}
		}
	}
}

static void TestPosition()
{
	OSCCamera::PoseMessage pose;
	pose.x = 1.0f; pose.y = 2.0f; pose.z = 3.0f;

	OSCCamera::ConvertSettings settings;	// default scale 100: the app sends metres, the scene is in cm

	// wire (right, forward, up) -> world (forward, up, right)
	OSCCamera::CameraState s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near3(s.position, 200.0, 300.0, 100.0));

	// the app already sends centimetres (Space scale = 100 in the app)
	settings.spaceScale = 1.0;
	s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near3(s.position, 2.0, 3.0, 1.0));

	// per axis sign, here the wire forward
	settings.positionSign[1] = -1.0;
	s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near3(s.position, -2.0, 3.0, 1.0));
}

static void TestRotationSign()
{
	OSCCamera::PoseMessage pose;
	pose.pan = 90.0f;

	OSCCamera::ConvertSettings settings;
	CHECK(Near3(OSCCamera::Convert(pose, nullptr, settings).forward, 0, 0, 1));

	settings.rotationSign[0] = -1.0;
	CHECK(Near3(OSCCamera::Convert(pose, nullptr, settings).forward, 0, 0, -1));
}

static void TestLens()
{
	OSCCamera::PoseMessage pose;
	pose.zoom = 85000;		// 85 mm calibrated
	pose.focus = 3500;		// 3.5 m calibrated
	pose.iris = 2800;		// f/2.8 calibrated

	OSCCamera::ConvertSettings settings;

	OSCCamera::LensMessage lens;
	lens.zoomCalibrated = lens.focusCalibrated = lens.irisCalibrated = true;

	OSCCamera::CameraState s = OSCCamera::Convert(pose, &lens, settings);
	CHECK(Near(s.focalLength, 85.0));
	CHECK(Near(s.focusDistance, 350.0));		// cm
	CHECK(Near(s.iris, 2.8));

	// channels switch independently, as soon as the lens message says so
	lens.zoomCalibrated = false;
	lens.focusCalibrated = true;
	lens.irisCalibrated = false;
	pose.zoom = 50;			// the middle of the default 0..100 raw range
	pose.iris = 100;		// the top of the raw range
	s = OSCCamera::Convert(pose, &lens, settings);
	CHECK(Near(s.focalLength, 18.0 + 0.5 * (200.0 - 18.0)));
	CHECK(Near(s.focusDistance, 350.0));
	CHECK(Near(s.iris, 22.0));

	// no lens message yet: every channel is remapped from the raw range
	pose.focus = 0;
	s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near(s.focalLength, 109.0));
	CHECK(Near(s.focusDistance, 30.0));

	// raw values outside of the range are clamped
	pose.zoom = 100000;
	s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near(s.focalLength, 200.0));

	// user defined ranges
	settings.zoomRaw = { 0.0, 1000.0 };
	settings.zoomOut = { 10.0, 20.0 };
	pose.zoom = 500;
	s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near(s.focalLength, 15.0));

	// a degenerate (empty) raw range must not divide by zero
	settings.zoomRaw = { 5.0, 5.0 };
	s = OSCCamera::Convert(pose, nullptr, settings);
	CHECK(Near(s.focalLength, 10.0));

	// a zero calibrated focal length is clamped to a valid one
	lens.zoomCalibrated = true;
	pose.zoom = 0;
	s = OSCCamera::Convert(pose, &lens, settings);
	CHECK(s.focalLength >= 1.0);
}

static void TestFocusAngle()
{
	// 50 mm at f/2 focused at 2 m: the aperture is 25 mm, seen from 2000 mm
	const double angle = OSCCamera::ComputeFocusAngle(50.0, 2.0, 200.0);
	CHECK(Near(angle, 2.0 * std::atan(25.0 / 4000.0) * 180.0 / OSCCamera::kPi));
	CHECK(angle > 0.7 && angle < 0.73);

	// inverted to the f-number: a wider aperture, a bigger angle
	CHECK(OSCCamera::ComputeFocusAngle(50.0, 1.4, 200.0) > angle);
	CHECK(OSCCamera::ComputeFocusAngle(50.0, 8.0, 200.0) < angle);

	// a longer lens or a closer focus: more blur
	CHECK(OSCCamera::ComputeFocusAngle(100.0, 2.0, 200.0) > angle);
	CHECK(OSCCamera::ComputeFocusAngle(50.0, 2.0, 100.0) > angle);

	// degenerate input gives no blur, never a NaN or a division by zero
	CHECK(OSCCamera::ComputeFocusAngle(50.0, 0.0, 200.0) == 0.0);
	CHECK(OSCCamera::ComputeFocusAngle(50.0, 2.0, 0.0) == 0.0);
	CHECK(OSCCamera::ComputeFocusAngle(0.0, 2.0, 200.0) == 0.0);

	// through the conversion, calibrated lens: 50 mm f/2 at 2 m
	OSCCamera::PoseMessage pose;
	pose.zoom = 50000;
	pose.focus = 2000;
	pose.iris = 2000;
	OSCCamera::LensMessage lens;
	lens.zoomCalibrated = lens.focusCalibrated = lens.irisCalibrated = true;

	OSCCamera::ConvertSettings settings;
	OSCCamera::CameraState s = OSCCamera::Convert(pose, &lens, settings);
	CHECK(Near(s.focusAngle, angle));

	settings.focusAngleScale = 2.5;
	s = OSCCamera::Convert(pose, &lens, settings);
	CHECK(Near(s.focusAngle, 2.5 * angle));
}

static void TestUnwrap()
{
	CHECK(Near(OSCCamera::UnwrapAngle(-179.0, 179.0), 181.0));
	CHECK(Near(OSCCamera::UnwrapAngle(179.0, -179.0), -181.0));
	CHECK(Near(OSCCamera::UnwrapAngle(10.0, 20.0), 10.0));
	CHECK(Near(OSCCamera::UnwrapAngle(-170.0, 540.0), 550.0));	// already wound up: stay on the same turn
}

int main()
{
	TestAxes();
	TestAxesAreARightHandedRotation();
	TestPosition();
	TestRotationSign();
	TestLens();
	TestFocusAngle();
	TestUnwrap();

	std::printf(g_failures == 0 ? "all math tests passed\n" : "%d check(s) failed\n", g_failures);
	return g_failures;
}
