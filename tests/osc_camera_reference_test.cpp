/**	\file	osc_camera_reference_test.cpp
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
*	Standalone test for the camera relative to its Ref marker (OSC_PROTOCOL.md section 2.8): ComposeReference of osc_camera_math.h and the
*	blending rules and the stream filter of osc_camera_timeline.h (no MotionBuilder SDK needed).
*	The expected numbers are the ones of the app's shared fixtures (app/src/test/resources/ref_local/transform_fixtures.json, "compose").
*	Build target: device_oscCamera_reference_test (excluded from the default build), the exit code is the number of failed checks.
*/

#include "osc_camera_timeline.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static int g_failures = 0;

#define CHECK(cond) \
	do { if (!(cond)) { ++g_failures; std::printf("FAILED %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

using namespace OSCCamera;

static const int64_t ms = 1000000LL;
static const int64_t second = 1000 * ms;

// the poses of the wire are float32 (the fixtures are doubles): 1e-6 is a few units of the last place of a value of 30
static bool Near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

//! a camera sample relative to the Ref \p key, the world fields are far from the local ones to show which is used
static PoseMessage LocalPose(const int64_t capture, const float x, const float y, const float z, const float pan, const float tilt, const float roll,
	const char* key = "Cart", const int generation = 1, const int follow = kFollowHeading, const int session = 1, const int sequence = 0)
{
	PoseMessage pose;
	pose.hasReference = true;
	pose.mode = kModeRefLocal;
	pose.captureNanos = capture;
	pose.sessionId = session;
	pose.sequence = sequence;
	pose.generation = generation;
	pose.follow = follow;
	std::strncpy(pose.key, key, kMaxReferenceKeyLength);
	pose.local[0] = x; pose.local[1] = y; pose.local[2] = z;
	pose.local[3] = pan; pose.local[4] = tilt; pose.local[5] = roll;
	pose.x = 1000.0f + x;		// the world pose, placeholders for the checks
	pose.y = 1000.0f + y;
	pose.z = 1000.0f + z;
	return pose;
}

static PoseMessage WorldPose(const int64_t capture, const float x, const int session = 1, const int sequence = 0)
{
	PoseMessage pose;
	pose.hasReference = true;
	pose.mode = kModeWorld;
	pose.captureNanos = capture;
	pose.sessionId = session;
	pose.sequence = sequence;
	pose.x = x;
	return pose;
}

//! MotionBuilder (forward, up, right) of a vector of the wire frame (right, forward, up)
static void ToWorldAxes(const double wire[3], double out[3])
{
	out[0] = wire[1];
	out[1] = wire[2];
	out[2] = wire[0];
}

struct ComposeCase
{
	const char* name;
	double offset[3];
	double yaw;
	int follow;
	float local[6];
	double position[3];		//!< expected, wire
	double forward[3];		//!< expected axes, wire
	double up[3];
	double right[3];
};

static void CheckCompose(const ComposeCase& c)
{
	const ConvertSettings settings;
	PoseMessage pose = LocalPose(1, c.local[0], c.local[1], c.local[2], c.local[3], c.local[4], c.local[5], "Cart", 1, c.follow);

	ReferenceFrame frame;
	for (int i = 0; i < 3; ++i)
	{
		frame.position[i] = c.offset[i];
	}
	frame.yaw = c.yaw;

	const BlendedPose composed = ComposeReference(ToBlended(pose), frame);
	CHECK(!composed.local);
	for (int i = 0; i < 3; ++i)
	{
		if (!Near(composed.wire[i], c.position[i]))
		{
			std::printf("  %s: position %d is %.12f, expected %.12f\n", c.name, i, composed.wire[i], c.position[i]);
		}
		CHECK(Near(composed.wire[i], c.position[i]));
	}

	double forward[3], up[3], right[3], expected[3];
	AxesFromQuat(composed.rotation, forward, up, right);
	ToWorldAxes(c.forward, expected);
	CHECK(Near(forward[0], expected[0]) && Near(forward[1], expected[1]) && Near(forward[2], expected[2]));
	ToWorldAxes(c.up, expected);
	CHECK(Near(up[0], expected[0]) && Near(up[1], expected[1]) && Near(up[2], expected[2]));
	ToWorldAxes(c.right, expected);
	CHECK(Near(right[0], expected[0]) && Near(right[1], expected[1]) && Near(right[2], expected[2]));
}

static void TestComposeFixtures()
{
	// heading 90 degrees, camera at the zeroed origin: it stands on the Ref and looks along its heading (-X in the wire frame)
	CheckCompose({ "heading 90, at the origin", { 5, 2, 1 }, 90.0, kFollowHeading, { 0, 0, 0, 0, 0, 0 },
		{ 5, 2, 1 }, { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } });

	// a step of 2 forward is a step along the heading
	CheckCompose({ "heading 90, 2 forward", { 5, 2, 1 }, 90.0, kFollowHeading, { 0, 2, 0, 0, 0, 0 },
		{ 3, 2, 1 }, { -1, 0, 0 }, { 0, 0, 1 }, { 0, 1, 0 } });

	// position only: the camera is moved, not turned
	CheckCompose({ "position only", { 5, 2, 1 }, 0.0, kFollowPosition, { 1, 2, 0.5f, 30, 10, 5 },
		{ 6, 4, 1.5 }, { 0.4924038765061041, 0.8528685319524432, 0.17364817766693036 },
		{ -0.011014609657371395, -0.19338934904742244, 0.9810602621904069 }, { 0.8702971336134901, -0.48499054308336637, -0.0858316511774313 } });

	// heading 180 flips the camera round
	CheckCompose({ "heading 180", { 0, 0, 0 }, 180.0, kFollowHeading, { 1, 0, 0, 0, 0, 0 },
		{ -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 }, { -1, 0, 0 } });

	// a heading and the opposite pan cancel
	CheckCompose({ "heading and pan cancel", { 0, 0, 0 }, -90.0, kFollowHeading, { 0, 0, 0, -90, 0, 0 },
		{ 0, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { 1, 0, 0 } });

	// all of it at once (seeded 0 of the fixtures: heading -224.9, pan -28.8, tilt 51.7, roll 26.7)
	CheckCompose({ "seeded 0", { -34.69951372600981, 30.818747825480685, -30.933739544090756 }, -224.93789930631405, kFollowHeading,
		{ -3.9886589924007385f, -4.743060616576194f, -0.07548425932114178f, -28.816796544682518f, 51.656056691180254f, 26.691928838976423f },
		{ -28.525837383423717, 31.358882056175425, -31.0092238034119 },
		{ -0.17226018994071912, -0.5959854754740963, 0.7843007968792989 },
		{ -0.2369612056272803, 0.7978938076070424, 0.5542696625380121 },
		{ -0.9561254174998455, -0.09037026505577904, -0.2786707756556024 } });
}

static void TestComposeFollow()
{
	const ConvertSettings settings;
	ReferenceFrame frame;
	frame.position[0] = 10.0; frame.position[1] = 20.0; frame.position[2] = 30.0;
	frame.yaw = 90.0;

	// a Ref that follows the position only does not turn the camera, whatever its heading is
	const BlendedPose local = ToBlended(LocalPose(1, 1, 2, 3, 15, -10, 5, "Cart", 1, kFollowPosition));
	const BlendedPose placed = ComposeReference(local, frame);
	CHECK(Near(placed.wire[0], 11.0) && Near(placed.wire[1], 22.0) && Near(placed.wire[2], 33.0));
	CHECK(Near(std::fabs(placed.rotation.w * local.localRotation.w + placed.rotation.x * local.localRotation.x
		+ placed.rotation.y * local.localRotation.y + placed.rotation.z * local.localRotation.z), 1.0));

	// the same camera with the heading: the offset turns to the Ref's heading, the vertical stays
	const BlendedPose heading = ComposeReference(ToBlended(LocalPose(1, 1, 2, 3, 15, -10, 5, "Cart", 1, kFollowHeading)), frame);
	CHECK(Near(heading.wire[0], 10.0 - 2.0) && Near(heading.wire[1], 20.0 + 1.0) && Near(heading.wire[2], 33.0));

	// the world pose is where the camera is without a Ref
	const BlendedPose world = ToBlended(WorldPose(1, 5.0f));
	CHECK(!world.local);
	const BlendedPose same = ComposeReference(world, frame);
	CHECK(Near(same.wire[0], 5.0) && Near(same.wire[1], 0.0));

	// a local sample keeps the enclosed world pose as the fallback
	const BlendedPose fallback = ToBlended(LocalPose(1, 1, 2, 3, 0, 0, 0));
	CHECK(fallback.local && Near(fallback.wire[0], 1001.0) && Near(fallback.localWire[0], 1.0));
}

static void TestBlendLocal()
{
	const ConvertSettings settings;
	const BlendedPose a = ToBlended(LocalPose(1, 0, 0, 0, 0, 0, 0));
	const BlendedPose b = ToBlended(LocalPose(2, 2, 4, 6, 90, 0, 0));
	const BlendedPose half = Blend(a, b, 0.5);
	CHECK(half.local && half.follow == kFollowHeading && half.generation == 1 && std::strcmp(half.key, "Cart") == 0);
	CHECK(Near(half.localWire[0], 1.0) && Near(half.localWire[1], 2.0) && Near(half.localWire[2], 3.0));
	CHECK(Near(half.wire[0], 1000.0 + 1.0));

	// 45 degrees of pan half way: composed with a Ref at the origin looking along +X it points half way between forward and right
	ReferenceFrame frame;
	const BlendedPose placed = ComposeReference(half, frame);
	double forward[3], up[3], right[3];
	AxesFromQuat(placed.rotation, forward, up, right);
	CHECK(Near(forward[0], std::cos(kPi / 4)) && Near(forward[2], std::sin(kPi / 4)));
}

static void TestTimelineKeepsReferencesApart()
{
	const ConvertSettings settings;
	const int64_t arrival = 100 * second;

	// local to Cart, then local to Car at the same spot of the timeline: the camera stays at the first until the second is due
	{
		Timeline t;
		t.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0, "Cart", 1), arrival);
		t.Add(LocalPose(1100 * ms, 10, 0, 0, 0, 0, 0, "Car", 1), arrival + 100 * ms);
		t.Add(LocalPose(1200 * ms, 20, 0, 0, 0, 0, 0, "Car", 1), arrival + 200 * ms);

		// 50 ms into the gap between them
		Timeline::RenderInfo info = t.Render(arrival + 50 * ms, 0, settings);
		CHECK(info.state == Timeline::RenderState::Held);
		CHECK(info.pose.local && std::strcmp(info.pose.key, "Cart") == 0 && Near(info.pose.localWire[0], 0.0));

		// past the change the two samples of the same Ref are blended
		info = t.Render(arrival + 150 * ms, 0, settings);
		CHECK(info.state == Timeline::RenderState::Interpolated);
		CHECK(std::strcmp(info.pose.key, "Car") == 0 && Near(info.pose.localWire[0], 15.0));
	}

	// the generation and the follow of the Ref are part of its identity
	{
		Timeline t;
		t.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0, "Cart", 1, kFollowHeading), arrival);
		t.Add(LocalPose(1100 * ms, 10, 0, 0, 0, 0, 0, "Cart", 2, kFollowHeading), arrival + 100 * ms);
		CHECK(t.Render(arrival + 50 * ms, 0, settings).state == Timeline::RenderState::Held);

		Timeline u;
		u.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0, "Cart", 1, kFollowHeading), arrival);
		u.Add(LocalPose(1100 * ms, 10, 0, 0, 0, 0, 0, "Cart", 1, kFollowPosition), arrival + 100 * ms);
		CHECK(u.Render(arrival + 50 * ms, 0, settings).state == Timeline::RenderState::Held);

		Timeline v;
		v.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0, "Cart", 1, kFollowHeading, 1), arrival);
		v.Add(LocalPose(1100 * ms, 10, 0, 0, 0, 0, 0, "Cart", 1, kFollowHeading, 2), arrival + 100 * ms);
		CHECK(v.Render(arrival + 50 * ms, 0, settings).state == Timeline::RenderState::Held);
	}

	// local to world: held too; and the world poses are blended whatever the Ref of the phone was
	{
		Timeline t;
		t.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0), arrival);
		t.Add(WorldPose(1100 * ms, 50.0f), arrival + 100 * ms);
		Timeline::RenderInfo info = t.Render(arrival + 50 * ms, 0, settings);
		CHECK(info.state == Timeline::RenderState::Held && info.pose.local);

		Timeline w;
		w.Add(WorldPose(1000 * ms, 0.0f), arrival);
		w.Add(WorldPose(1100 * ms, 10.0f), arrival + 100 * ms);
		info = w.Render(arrival + 50 * ms, 0, settings);
		CHECK(info.state == Timeline::RenderState::Interpolated && !info.pose.local && Near(info.pose.wire[0], 5.0));
	}

	// extrapolation past the newest sample needs the last two to be of one Ref, else the last one is held
	{
		Timeline t;
		t.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0, "Cart", 1), arrival);
		t.Add(LocalPose(1033 * ms, 1, 0, 0, 0, 0, 0, "Cart", 1), arrival + 33 * ms);
		CHECK(t.Render(arrival + 50 * ms, 0, settings).state == Timeline::RenderState::Extrapolated);

		Timeline u;
		u.Add(LocalPose(1000 * ms, 0, 0, 0, 0, 0, 0, "Cart", 1), arrival);
		u.Add(LocalPose(1033 * ms, 1, 0, 0, 0, 0, 0, "Car", 1), arrival + 33 * ms);
		const Timeline::RenderInfo info = u.Render(arrival + 50 * ms, 0, settings);
		CHECK(info.state == Timeline::RenderState::Held && Near(info.pose.localWire[0], 1.0));
	}
}

static void TestStreamFilter()
{
	StreamFilter f;
	using R = StreamFilter::Result;
	const int64_t t0 = 50 * second;

	// the former pose has no session: always taken
	PoseMessage legacy;
	CHECK(f.Accept(legacy, t0) == R::Accepted);
	CHECK(f.Accept(legacy, t0) == R::Accepted);

	CHECK(f.Accept(WorldPose(1, 0, 7, 10), t0) == R::Accepted);
	CHECK(f.Accept(WorldPose(1, 0, 7, 11), t0 + 10 * ms) == R::Accepted);
	CHECK(f.Accept(WorldPose(1, 0, 7, 11), t0 + 20 * ms) == R::Stale);		// a duplicate
	CHECK(f.Accept(WorldPose(1, 0, 7, 9), t0 + 30 * ms) == R::Stale);		// older
	CHECK(f.Accept(WorldPose(1, 0, 7, 15), t0 + 40 * ms) == R::Accepted);	// a gap is a loss, not an error
	CHECK(f.Accept(WorldPose(1, 0, 7, 14), t0 + 50 * ms) == R::Stale);		// and what was lost does not come back after a later one

	// a new session replaces the old one; the late datagrams of the old are dropped for a while
	CHECK(f.Accept(WorldPose(1, 0, 8, 0), t0 + 60 * ms) == R::NewSession);
	CHECK(f.Accept(WorldPose(1, 0, 8, 1), t0 + 70 * ms) == R::Accepted);
	CHECK(f.Accept(WorldPose(1, 0, 7, 16), t0 + 80 * ms) == R::Retired);
	CHECK(f.Accept(WorldPose(1, 0, 8, 2), t0 + 90 * ms) == R::Accepted);

	// after that long the old id is just another new session (the phone numbers its sessions again)
	CHECK(f.Accept(WorldPose(1, 0, 7, 0), t0 + 90 * ms + StreamFilter::kRetiredMemory + ms) == R::NewSession);

	// the int32 sequence wraps
	StreamFilter g;
	CHECK(g.Accept(WorldPose(1, 0, 3, 2147483646), t0) == R::Accepted);
	CHECK(g.Accept(WorldPose(1, 0, 3, 2147483647), t0 + ms) == R::Accepted);
	CHECK(g.Accept(WorldPose(1, 0, 3, static_cast<int32_t>(0x80000000u)), t0 + 2 * ms) == R::Accepted);		// the next after int32 max
	CHECK(g.Accept(WorldPose(1, 0, 3, static_cast<int32_t>(0x80000001u)), t0 + 3 * ms) == R::Accepted);
	CHECK(g.Accept(WorldPose(1, 0, 3, 2147483647), t0 + 4 * ms) == R::Stale);

	// a phone that restarts with the same session id and its sequence at 0: taken after a second of silence
	StreamFilter h;
	CHECK(h.Accept(WorldPose(1, 0, 5, 1000), t0) == R::Accepted);
	CHECK(h.Accept(WorldPose(1, 0, 5, 0), t0 + 100 * ms) == R::Stale);
	CHECK(h.Accept(WorldPose(1, 0, 5, 0), t0 + 100 * ms + StreamFilter::kSilence + ms) == R::Accepted);
	CHECK(h.Accept(WorldPose(1, 0, 5, 1), t0 + 100 * ms + StreamFilter::kSilence + 2 * ms) == R::Accepted);

	f.Reset();
	CHECK(f.Accept(WorldPose(1, 0, 7, 1), t0) == R::Accepted);
}

int main()
{
	TestComposeFixtures();
	TestComposeFollow();
	TestBlendLocal();
	TestTimelineKeepsReferencesApart();
	TestStreamFilter();

	std::printf(g_failures == 0 ? "all reference tests passed\n" : "%d check(s) failed\n", g_failures);
	return g_failures;
}
