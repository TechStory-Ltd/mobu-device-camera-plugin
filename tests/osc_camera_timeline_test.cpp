/**	\file	osc_camera_timeline_test.cpp
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
*	Standalone test for osc_camera_timeline.h (the timeline and the playback of the live camera) and the blending of osc_camera_math.h
*	(no MotionBuilder SDK needed). The recorded stream in tests/fixtures is replayed by TestRecordedCapture.
*	Build target: device_oscCamera_timeline_test (excluded from the default build), the exit code is the number of failed checks.
*/

#include "osc_camera_timeline.h"
#include "osc_camera_decoder.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static int g_failures = 0;

#define CHECK(cond) \
	do { if (!(cond)) { ++g_failures; std::printf("FAILED %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

using OSCCamera::Timeline;

static const int64_t ms = 1000000LL;
static const int64_t second = 1000 * ms;

static bool Near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

static OSCCamera::PoseMessage Pose(int64_t capture, float x, float pan = 0.0f, int32_t zoom = 0)
{
	OSCCamera::PoseMessage pose;
	pose.captureNanos = capture;
	pose.x = x;
	pose.pan = pan;
	pose.zoom = zoom;
	return pose;
}

static void TestBlendMath()
{
	using namespace OSCCamera;
	ConvertSettings settings;

	// a quaternion round trip keeps the camera axes
	const PoseMessage pose = Pose(1, 0.0f, 33.0f);
	PoseMessage tilted = pose;
	tilted.tilt = -20.0f;
	tilted.roll = 12.0f;
	const CameraState direct = Convert(tilted, nullptr, settings);
	const CameraState viaQuat = ConvertBlended(ToBlended(tilted), nullptr, settings);
	for (int i = 0; i < 3; ++i)
	{
		CHECK(Near(direct.forward[i], viaQuat.forward[i], 1e-9));
		CHECK(Near(direct.up[i], viaQuat.up[i], 1e-9));
		CHECK(Near(direct.right[i], viaQuat.right[i], 1e-9));
		CHECK(Near(direct.position[i], viaQuat.position[i], 1e-9));
	}
	CHECK(Near(direct.focalLength, viaQuat.focalLength));

	// half way between pan 0 and pan 90 is pan 45, also over the +-180 wrap (the short way: 170 -> -170 passes 180)
	const BlendedPose a = ToBlended(Pose(1, 0.0f, 0.0f));
	const BlendedPose b = ToBlended(Pose(2, 0.0f, 90.0f));
	const CameraState half = ConvertBlended(Blend(a, b, 0.5), nullptr, settings);
	const CameraState expected = Convert(Pose(1, 0.0f, 45.0f), nullptr, settings);
	for (int i = 0; i < 3; ++i)
	{
		CHECK(Near(half.forward[i], expected.forward[i], 1e-9));
	}

	const BlendedPose c = ToBlended(Pose(1, 0.0f, 170.0f));
	const BlendedPose d = ToBlended(Pose(2, 0.0f, -170.0f));
	const CameraState wrap = ConvertBlended(Blend(c, d, 0.5), nullptr, settings);
	const CameraState expectedWrap = Convert(Pose(1, 0.0f, 180.0f), nullptr, settings);
	for (int i = 0; i < 3; ++i)
	{
		CHECK(Near(wrap.forward[i], expectedWrap.forward[i], 1e-9));
	}

	// the ends of a blend
	const CameraState start = ConvertBlended(Blend(a, b, 0.0), nullptr, settings);
	const CameraState end = ConvertBlended(Blend(a, b, 1.0), nullptr, settings);
	const CameraState expectedStart = Convert(Pose(1, 0.0f, 0.0f), nullptr, settings);
	const CameraState expectedEnd = Convert(Pose(1, 0.0f, 90.0f), nullptr, settings);
	for (int i = 0; i < 3; ++i)
	{
		CHECK(Near(start.forward[i], expectedStart.forward[i], 1e-9));
		CHECK(Near(end.forward[i], expectedEnd.forward[i], 1e-9));
	}

	// lens integers blend as numbers
	const BlendedPose e = ToBlended(Pose(1, 0.0f, 0.0f, 24000));
	const BlendedPose f = ToBlended(Pose(2, 0.0f, 0.0f, 48000));
	CHECK(Near(Blend(e, f, 0.5).zoom, 36000.0));

	// sensor plausibility
	CHECK(OSCCamera::IsUsableSensor(24.89f, 14.0f));
	CHECK(!OSCCamera::IsUsableSensor(0.0f, 0.0f));
	CHECK(!OSCCamera::IsUsableSensor(1000.0f, 14.0f));
	CHECK(!OSCCamera::IsUsableSensor(std::nanf(""), 14.0f));
}

static void TestOrderAndRepeats()
{
	Timeline t;
	CHECK(!t.HasPose());

	// a sample every 33.3 ms, each one sent twice (60 Hz stream of a 30 Hz sensor)
	int64_t arrival = 100 * second;
	Timeline::AddInfo info = t.Add(Pose(1000 * ms, 1.0f), arrival);
	CHECK(info.result == Timeline::AddResult::New && info.newest);
	info = t.Add(Pose(1000 * ms, 1.0f), arrival + 16 * ms);
	CHECK(info.result == Timeline::AddResult::Repeat && !info.newest);
	info = t.Add(Pose(1033 * ms, 2.0f), arrival + 33 * ms);
	CHECK(info.result == Timeline::AddResult::New && info.newest);
	CHECK(t.SampleCount() == 2);

	// a packet that arrives late is put in its place and is not the newest
	info = t.Add(Pose(1100 * ms, 4.0f), arrival + 100 * ms);
	CHECK(info.newest);
	info = t.Add(Pose(1066 * ms, 3.0f), arrival + 105 * ms);
	CHECK(info.result == Timeline::AddResult::New && !info.newest);
	CHECK(t.SampleCount() == 4);

	// the newest sample is still the newest one
	OSCCamera::PoseMessage newest;
	CHECK(t.GetNewest(newest) && newest.x == 4.0f);
}

static void TestUnstamped()
{
	Timeline t;
	const OSCCamera::ConvertSettings settings;

	// no sensor sample yet (captureNanos 0): held, but not on the timeline
	Timeline::AddInfo info = t.Add(Pose(0, 5.0f), 10 * second);
	CHECK(info.result == Timeline::AddResult::Unstamped && !info.newest);
	CHECK(t.SampleCount() == 0 && t.HasPose());

	Timeline::RenderInfo r = t.Render(10 * second, 50 * ms, settings);
	CHECK(r.state == Timeline::RenderState::Held && Near(r.pose.wire[0], 5.0));

	// a stamped sample takes over
	t.Add(Pose(500 * ms, 7.0f), 10 * second);
	r = t.Render(10 * second, 0, settings);
	CHECK(Near(r.pose.wire[0], 7.0));
}

static void TestOffsetAndRender()
{
	Timeline t;
	const OSCCamera::ConvertSettings settings;

	// phone clock is 5000 s ahead of ours; samples 30 ms apart arrive with 20 ms of network delay
	const int64_t base = 5000 * second;
	for (int i = 0; i < 10; ++i)
	{
		const int64_t capture = base + i * 30 * ms;
		t.Add(Pose(capture, static_cast<float>(i)), capture - 5000 * second + 20 * ms + 1000 * second);	// arrival on our clock
	}
	// offset = arrival - capture = 20 ms - 4000 s... only the differences matter: the render time maps back to capture
	Timeline::RenderInfo r;

	// half way between sample 4 and 5, rendered with a delay of 25 ms: the time asked is 'now - 25 ms'
	const int64_t arrival4 = base + 4 * 30 * ms - 5000 * second + 20 * ms + 1000 * second;
	const int64_t now = arrival4 + 15 * ms + 25 * ms;	// (now - delay) maps to capture of 4 + 15 ms
	r = t.Render(now, 25 * ms, settings);
	CHECK(r.state == Timeline::RenderState::Interpolated);
	CHECK(Near(r.pose.wire[0], 4.5, 1e-6));

	// before the oldest sample: held on it
	r = t.Render(arrival4 - 10 * second, 0, settings);
	CHECK(r.state == Timeline::RenderState::Held && Near(r.pose.wire[0], 0.0));

	// past the newest: a short extrapolation (1 sample per 30 ms), then held
	const int64_t arrival9 = base + 9 * 30 * ms - 5000 * second + 20 * ms + 1000 * second;
	r = t.Render(arrival9 + 15 * ms, 0, settings);
	CHECK(r.state == Timeline::RenderState::Extrapolated && Near(r.pose.wire[0], 9.5, 1e-6));
	r = t.Render(arrival9 + 200 * ms, 0, settings);
	CHECK(r.state == Timeline::RenderState::Held && Near(r.pose.wire[0], 9.0));
}

static void TestOffsetFollowsMinimum()
{
	Timeline t;

	// a late packet does not move the offset up, a faster one moves it down at once
	int64_t arrival = 50 * second;
	t.Add(Pose(1000 * ms, 0.0f), arrival);
	const int64_t first = t.ToLocalTime(0);

	t.Add(Pose(1033 * ms, 0.0f), arrival + 33 * ms + 40 * ms);		// 40 ms late
	CHECK(t.ToLocalTime(0) == first);
	CHECK(t.JitterMilliseconds() > 39.0 && t.JitterMilliseconds() < 41.0);

	t.Add(Pose(1066 * ms, 0.0f), arrival + 66 * ms - 5 * ms);		// 5 ms faster than the first
	CHECK(t.ToLocalTime(0) == first - 5 * ms);

	// the window forgets: after the fast packets are older than 5 s, a slower path is taken up gradually (2 ms per second)
	Timeline s;
	s.Add(Pose(1000 * ms, 0.0f), 10 * second);
	const int64_t before = s.ToLocalTime(0);
	// the path got 100 ms slower, packets keep arriving for 7 seconds
	int64_t last = 0;
	for (int i = 1; i <= 210; ++i)
	{
		const int64_t cap = 1000 * ms + i * 33 * ms;
		s.Add(Pose(cap, 0.0f), 10 * second + i * 33 * ms + 100 * ms);
		last = s.ToLocalTime(0);
	}
	CHECK(last > before);							// it moved up ...
	CHECK(last - before < 20 * ms);					// ... but slowly (about 2 ms per second), not by the 100 ms at once
}

static void TestReboot()
{
	Timeline t;
	for (int i = 0; i < 5; ++i)
	{
		t.Add(Pose(5000 * second + i * 33 * ms, 1.0f), 100 * second + i * 33 * ms);
	}
	CHECK(t.SampleCount() == 5);

	// the phone restarted: its clock starts from a small number
	Timeline::AddInfo info = t.Add(Pose(2 * second, 9.0f), 130 * second);
	CHECK(info.result == Timeline::AddResult::Reset && info.newest);
	CHECK(t.SampleCount() == 1);

	// a small step back is only a late packet, not a reboot
	Timeline u;
	u.Add(Pose(5000 * second, 1.0f), 100 * second);
	info = u.Add(Pose(5000 * second - 100 * ms, 1.0f), 100 * second + 10 * ms);
	CHECK(info.result == Timeline::AddResult::New && !info.newest);
	CHECK(u.SampleCount() == 2);
}

static void TestBounded()
{
	Timeline t;
	for (int i = 0; i < 2000; ++i)
	{
		t.Add(Pose(1000 * ms + i * 10 * ms, 1.0f), 10 * second + i * 10 * ms);
	}
	CHECK(t.SampleCount() == Timeline::kMaxSamples);

	// something older than everything kept when the timeline is full (the oldest kept is at ~15.9 s, the newest at ~21 s)
	const Timeline::AddInfo info = t.Add(Pose(15000 * ms, 1.0f), 40 * second);
	CHECK(info.result == Timeline::AddResult::Stale);
	CHECK(t.SampleCount() == Timeline::kMaxSamples);
}

// ---------------------------------------------------------------------------------------------------------------------
// the live playback: Fixed, Auto and Newest (Playback), replayed on a stream like the one of a phone

using OSCCamera::Playback;

struct Packet
{
	int64_t arrival;
	OSCCamera::PoseMessage pose;
};

static double AngleDegrees(const OSCCamera::Quat& a, const OSCCamera::Quat& b)
{
	const double dot = std::fabs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
	return 2.0 * std::acos(std::min(dot, 1.0)) * 180.0 / 3.14159265358979323846;
}

/// what a receiver that had seen the whole stream would draw at the phone time pTime
static OSCCamera::Quat Ideal(const std::vector<Packet>& pSorted, const int64_t pTime, const OSCCamera::ConvertSettings& pSettings)
{
	const auto lAfter = std::upper_bound(pSorted.begin(), pSorted.end(), pTime,
		[](const int64_t pValue, const Packet& pPacket) { return pValue < pPacket.pose.captureNanos; });
	if (lAfter == pSorted.begin())
	{
		return OSCCamera::ToBlended(pSorted.front().pose).rotation;
	}
	if (lAfter == pSorted.end())
	{
		return OSCCamera::ToBlended(pSorted.back().pose).rotation;
	}
	const Packet& lFrom = *(lAfter - 1);
	const Packet& lTo = *lAfter;
	return OSCCamera::Blend(OSCCamera::ToBlended(lFrom.pose), OSCCamera::ToBlended(lTo.pose),
		static_cast<double>(pTime - lFrom.pose.captureNanos) / static_cast<double>(lTo.pose.captureNanos - lFrom.pose.captureNanos)).rotation;
}

struct Replay
{
	double rms{ 0.0 };			//!< degrees, the angle between the frame and the ideal one
	double p99{ 0.0 };
	double lastRms{ 0.0 };		//!< the last 15 s
	double lastP99{ 0.0 };
	double interpolated{ 0.0 };	//!< percent of the frames
	double meanDelayMs{ 0.0 };
	double minDelayMs{ 1e9 };
	double maxDelayMs{ 0.0 };
	size_t frames{ 0 };
};

/// the packets in the order they arrive, the device renders at pRenderHz from a second after the first one
static Replay Run(const std::vector<Packet>& pPackets, const Playback::Settings& pSettings, const double pRenderHz = 60.0)
{
	const OSCCamera::ConvertSettings lConvert;

	std::vector<Packet> lSorted = pPackets;		// by capture time, without the repeats
	std::stable_sort(lSorted.begin(), lSorted.end(), [](const Packet& a, const Packet& b) { return a.pose.captureNanos < b.pose.captureNanos; });
	lSorted.erase(std::unique(lSorted.begin(), lSorted.end(), [](const Packet& a, const Packet& b) { return a.pose.captureNanos == b.pose.captureNanos; }), lSorted.end());

	Timeline lTimeline;
	Playback lPlayback;
	Replay lReplay;
	std::vector<double> lErrors, lTail;
	size_t lNext = 0;
	size_t lInterpolated = 0;
	double lDelaySum = 0.0;

	const int64_t lStart = pPackets.front().arrival + second;
	const int64_t lEnd = pPackets.back().arrival;
	const int64_t lStep = static_cast<int64_t>(1e9 / pRenderHz);
	for (int64_t lNow = lStart; lNow < lEnd; lNow += lStep)
	{
		while (lNext < pPackets.size() && pPackets[lNext].arrival <= lNow)
		{
			lTimeline.Add(pPackets[lNext].pose, pPackets[lNext].arrival);
			++lNext;
		}

		const Playback::Frame lFrame = lPlayback.Render(lTimeline, lNow, pSettings, lConvert);
		const int64_t lTime = lNow - lTimeline.ToLocalTime(0) - lFrame.delay;		// the phone time the frame is for
		if (lTime < lSorted.front().pose.captureNanos || lTime > lSorted.back().pose.captureNanos)
		{
			continue;
		}
		const double lError = AngleDegrees(lFrame.render.pose.rotation, Ideal(lSorted, lTime, lConvert));
		lErrors.push_back(lError);
		if (lNow > lEnd - 15 * second)
		{
			lTail.push_back(lError);
		}
		lInterpolated += (lFrame.render.state == Timeline::RenderState::Interpolated) ? 1 : 0;
		const double lDelayMs = static_cast<double>(lFrame.delay) * 1e-6;
		lDelaySum += lDelayMs;
		lReplay.minDelayMs = std::min(lReplay.minDelayMs, lDelayMs);
		lReplay.maxDelayMs = std::max(lReplay.maxDelayMs, lDelayMs);
	}

	auto lRms = [](const std::vector<double>& v)
	{
		double lSum = 0.0;
		for (const double e : v) { lSum += e * e; }
		return v.empty() ? 0.0 : std::sqrt(lSum / static_cast<double>(v.size()));
	};
	auto lPercentile = [](std::vector<double> v, const double q)
	{
		if (v.empty()) { return 0.0; }
		std::sort(v.begin(), v.end());
		return v[static_cast<size_t>(q * static_cast<double>(v.size() - 1))];
	};

	lReplay.frames = lErrors.size();
	lReplay.rms = lRms(lErrors);
	lReplay.p99 = lPercentile(lErrors, 0.99);
	lReplay.lastRms = lRms(lTail);
	lReplay.lastP99 = lPercentile(lTail, 0.99);
	lReplay.interpolated = lErrors.empty() ? 0.0 : 100.0 * static_cast<double>(lInterpolated) / static_cast<double>(lErrors.size());
	lReplay.meanDelayMs = lErrors.empty() ? 0.0 : lDelaySum / static_cast<double>(lErrors.size());
	return lReplay;
}

static Playback::Settings Mode(const Playback::Mode pMode, const int64_t pDelay = 0, const int64_t pMax = 60 * ms,
	const Playback::Responsiveness pResponsiveness = Playback::Responsiveness::Balanced)
{
	Playback::Settings lSettings;
	lSettings.mode = pMode;
	lSettings.delay = pDelay;
	lSettings.maxDelay = pMax;
	lSettings.responsiveness = pResponsiveness;
	return lSettings;
}

/// a pan at pDegreesPerSecond, a sample every pSpacing, arriving pBase + a random 0..pJitter later, noisy by +-pNoise degrees
static std::vector<Packet> Stream(const double pSeconds, const double pDegreesPerSecond, const int64_t pSpacing, const int64_t pBase, const int64_t pJitter,
	const double pNoise, const uint32_t pSeed = 1)
{
	uint32_t lState = pSeed;
	auto lRandom = [&lState]() { lState = lState * 1664525u + 1013904223u; return static_cast<double>(lState >> 8) / static_cast<double>(1 << 24); };

	std::vector<Packet> lPackets;
	const int64_t lPhoneStart = 3000 * second;
	for (int64_t t = 0; t < static_cast<int64_t>(pSeconds * 1e9); t += pSpacing)
	{
		Packet lPacket;
		lPacket.pose = Pose(lPhoneStart + t, 0.0f, static_cast<float>(pDegreesPerSecond * static_cast<double>(t) * 1e-9 + (lRandom() - 0.5) * 2.0 * pNoise));
		lPacket.arrival = lPhoneStart + t + pBase + static_cast<int64_t>(lRandom() * static_cast<double>(pJitter)) - 2900 * second;
		lPackets.push_back(lPacket);
	}
	std::stable_sort(lPackets.begin(), lPackets.end(), [](const Packet& a, const Packet& b) { return a.arrival < b.arrival; });
	return lPackets;
}

static void TestFixedExtrapolatesExactly()
{
	// a constant pan on a constant delay: the extrapolation of the last two samples is exact
	const std::vector<Packet> lPackets = Stream(20.0, 60.0, 16666667, 20 * ms, 0, 0.0);
	const Replay lFixed = Run(lPackets, Mode(Playback::Mode::Fixed, 0));
	CHECK(lFixed.frames > 1000);
	CHECK(lFixed.rms < 0.01);

	// Fixed with a delay is Render with that delay, nothing more
	Timeline t;
	for (const Packet& p : lPackets)
	{
		t.Add(p.pose, p.arrival);
	}
	const OSCCamera::ConvertSettings lConvert;
	Playback lPlayback;
	const int64_t lNow = lPackets.back().arrival + 7 * ms;
	const Playback::Frame lFrame = lPlayback.Render(t, lNow, Mode(Playback::Mode::Fixed, 30 * ms), lConvert);
	const Timeline::RenderInfo lDirect = t.Render(lNow, 30 * ms, lConvert);
	CHECK(lFrame.delay == 30 * ms && lFrame.render.state == lDirect.state);
	CHECK(AngleDegrees(lFrame.render.pose.rotation, lDirect.pose.rotation) < 1e-9);
}

static void TestAutoBeatsDelayZeroOnAJitteryNetwork()
{
	// 60 deg/s, a sample every 16.7 ms, arriving 20..50 ms later (uniform), 0.05 deg of noise on each sample
	const std::vector<Packet> lPackets = Stream(60.0, 60.0, 16666667, 20 * ms, 30 * ms, 0.05);

	const Replay lZero = Run(lPackets, Mode(Playback::Mode::Fixed, 0));
	const Replay lAuto = Run(lPackets, Mode(Playback::Mode::Auto));
	std::printf("  jittery stream: delay 0 rms %.3f p99 %.3f | auto rms %.3f p99 %.3f, %.1f %% interpolated, mean delay %.1f ms\n",
		lZero.rms, lZero.p99, lAuto.rms, lAuto.p99, lAuto.interpolated, lAuto.meanDelayMs);

	CHECK(lAuto.interpolated >= 95.0);
	CHECK(lAuto.rms < lZero.rms * 0.35);
	CHECK(lAuto.meanDelayMs < 45.0);

	// the more of the jitter it covers, the more delay and the fewer frames are guessed
	const Replay lResponsive = Run(lPackets, Mode(Playback::Mode::Auto, 0, 60 * ms, Playback::Responsiveness::Responsive));
	const Replay lSmooth = Run(lPackets, Mode(Playback::Mode::Auto, 0, 60 * ms, Playback::Responsiveness::Smooth));
	CHECK(lResponsive.meanDelayMs < lAuto.meanDelayMs && lAuto.meanDelayMs < lSmooth.meanDelayMs);
	CHECK(lSmooth.interpolated >= lAuto.interpolated && lAuto.interpolated >= lResponsive.interpolated);
}

static void TestAutoDelayMoves()
{
	const OSCCamera::ConvertSettings lConvert;
	const Playback::Settings lSettings = Mode(Playback::Mode::Auto, 10 * ms, 40 * ms);

	Timeline t;
	Playback p;
	int64_t lPhone = 3000 * second;
	uint32_t lState = 7;
	auto lRandom = [&lState]() { lState = lState * 1664525u + 1013904223u; return static_cast<double>(lState >> 8) / static_cast<double>(1 << 24); };

	// a sample every 16.7 ms arriving 20 ms + 0..pJitter later; the delay after pSeconds
	auto lFeed = [&](const double pSeconds, const int64_t pJitter)
	{
		int64_t lDelay = 0;
		for (int i = 0; i < static_cast<int>(pSeconds * 60.0); ++i)
		{
			lPhone += 16666667;
			t.Add(Pose(lPhone, 0.0f), lPhone - 2900 * second + 20 * ms + static_cast<int64_t>(lRandom() * static_cast<double>(pJitter)));
			const Playback::Frame lFrame = p.Render(t, lPhone - 2900 * second + 40 * ms, lSettings, lConvert);
			CHECK(lFrame.delay >= lSettings.delay && lFrame.delay <= lSettings.maxDelay);
			lDelay = lFrame.delay;
		}
		return lDelay;
	};

	const int64_t lQuiet = lFeed(8.0, 1 * ms);
	CHECK(lQuiet <= 12 * ms);								// near the floor

	// a burst of late packets: it rises at most 20 ms per second, but it does rise
	const int64_t lHalfSecond = lFeed(0.5, 30 * ms);
	CHECK(lHalfSecond > lQuiet);
	CHECK(lHalfSecond - lQuiet <= 11 * ms);
	const int64_t lBurst = lFeed(3.0, 30 * ms);
	CHECK(lBurst >= 30 * ms);

	// and it falls slowly, at most 2 ms per second, however quiet the network is
	const int64_t lSecondQuiet = lFeed(1.0, 1 * ms);
	CHECK(lSecondQuiet < lBurst);
	CHECK(lBurst - lSecondQuiet <= 2 * ms + ms / 2);
	const int64_t lFourQuiet = lFeed(3.0, 1 * ms);
	CHECK(lSecondQuiet - lFourQuiet <= 6 * ms + ms / 2);
	CHECK(lFeed(40.0, 1 * ms) <= 14 * ms);					// and gets back to the floor

	// the ceiling and the floor hold also when they are moved; a floor above the ceiling wins
	Playback::Settings lMoved = lSettings;
	lMoved.maxDelay = 12 * ms;
	CHECK(p.Render(t, lPhone - 2900 * second + 40 * ms, lMoved, lConvert).delay <= 12 * ms);
	lMoved.delay = 20 * ms;
	CHECK(p.Render(t, lPhone - 2900 * second + 40 * ms, lMoved, lConvert).delay == 20 * ms);
}

static void TestNewestShowsTheNewestSample()
{
	const OSCCamera::ConvertSettings lConvert;
	Timeline t;
	Playback p;

	CHECK(p.Render(t, 10 * second, Mode(Playback::Mode::Newest), lConvert).render.state == Timeline::RenderState::None);

	t.Add(Pose(1000 * ms, 1.0f), 50 * second);
	t.Add(Pose(1016 * ms, 2.0f), 50 * second + 16 * ms);
	t.Add(Pose(1033 * ms, 3.0f), 50 * second + 40 * ms);		// this one arrived late
	t.Add(Pose(1025 * ms, 9.0f), 50 * second + 45 * ms);		// and an older one after it: not the newest

	for (const int64_t lNow : { 50 * second + 41 * ms, 50 * second + 60 * ms, 60 * second })
	{
		const Playback::Frame lFrame = p.Render(t, lNow, Mode(Playback::Mode::Newest, 40 * ms), lConvert);
		CHECK(lFrame.render.state == Timeline::RenderState::Held);
		CHECK(Near(lFrame.render.pose.wire[0], 3.0));			// exactly the sample: no extrapolation, no delay, no blending
		CHECK(lFrame.delay == 0);
	}
}

static void TestRenderShares()
{
	const OSCCamera::ConvertSettings lConvert;
	Timeline t;
	Playback p;
	for (int i = 0; i < 10; ++i)
	{
		t.Add(Pose(1000 * ms + i * 30 * ms, static_cast<float>(i)), 50 * second + i * 30 * ms + 20 * ms);
	}
	const int64_t lBase = 50 * second + 20 * ms;

	// three frames between the samples
	for (int i = 0; i < 3; ++i)
	{
		p.Render(t, lBase + 100 * ms + i * ms + 15 * ms, Mode(Playback::Mode::Fixed, 15 * ms), lConvert);
	}
	Playback::Shares s = p.GetShares(lBase + 100 * ms);
	CHECK(Near(s.interpolated, 100.0, 1e-9) && Near(s.extrapolated, 0.0, 1e-9) && Near(s.held, 0.0, 1e-9));

	// one far past the newest sample: held, and the old ones are out of the last seconds
	p.Render(t, lBase + 10 * second, Mode(Playback::Mode::Fixed, 0), lConvert);
	s = p.GetShares(lBase + 10 * second);
	CHECK(Near(s.held, 100.0, 1e-9));

	// a mix
	for (int i = 0; i < 3; ++i)
	{
		p.Render(t, lBase + 10 * second + (i + 1) * ms, Mode(Playback::Mode::Fixed, 0), lConvert);
	}
	p.Render(t, lBase + 10 * second + 5 * ms, Mode(Playback::Mode::Fixed, 10 * second + 5 * ms - 100 * ms - 5 * ms), lConvert);	// back in the samples
	s = p.GetShares(lBase + 10 * second + 5 * ms);
	CHECK(Near(s.interpolated + s.extrapolated + s.held, 100.0, 1e-9));
	CHECK(s.held > 70.0 && s.interpolated > 10.0);
}

static void TestJitterIgnoresRepeatsAndOneLatePacket()
{
	Timeline t;
	for (int i = 0; i < 200; ++i)
	{
		const int64_t lCapture = 1000 * ms + i * 16 * ms;
		const int64_t lArrival = 50 * second + i * 16 * ms + 20 * ms;
		t.Add(Pose(lCapture, 0.0f), lArrival + (i == 100 ? 40 * ms : 0));			// one packet 40 ms late
		t.Add(Pose(lCapture, 0.0f), lArrival + 8 * ms);								// and the same sample again, 8 ms later
	}
	CHECK(t.JitterMilliseconds() > 39.0 && t.JitterMilliseconds() < 41.0);			// max - min: the late packet, not the repeats
	CHECK(t.JitterSpreadMilliseconds() < 1.0);										// p99 - p1: the late packet is one in 200
	CHECK(t.ExcessQuantile(0.99) < 1 * ms);
	CHECK(t.ExcessQuantile(1.0) > 39 * ms);

	// the spacing is the one of the sensor, not of the packets
	CHECK(t.SampleSpacing() == 16 * ms);
	Timeline e;
	CHECK(e.SampleSpacing() == Timeline::kDefaultSpacing && e.ExcessQuantile(0.99) == 0 && e.JitterSpreadMilliseconds() == 0.0);
}

// the capture of the phone of 2026-10-27 as recorded by tools/osc_relay.py: records of a big endian double (arrival, seconds), a big endian
// uint16 length and the datagram
static std::vector<Packet> ReadCapture(const std::string& pPath)
{
	std::vector<Packet> lPackets;
	std::FILE* lFile = std::fopen(pPath.c_str(), "rb");
	if (lFile == nullptr)
	{
		return lPackets;
	}
	std::vector<unsigned char> lRaw;
	unsigned char lChunk[65536];
	size_t lRead;
	while ((lRead = std::fread(lChunk, 1, sizeof(lChunk), lFile)) > 0)
	{
		lRaw.insert(lRaw.end(), lChunk, lChunk + lRead);
	}
	std::fclose(lFile);

	double lFirst = -1.0;
	for (size_t i = 0; i + 10 <= lRaw.size();)
	{
		uint64_t lBits = 0;
		for (size_t b = 0; b < 8; ++b)
		{
			lBits = (lBits << 8) | lRaw[i + b];
		}
		double lArrival;
		std::memcpy(&lArrival, &lBits, sizeof(lArrival));
		const size_t lLength = (static_cast<size_t>(lRaw[i + 8]) << 8) | lRaw[i + 9];
		if (i + 10 + lLength > lRaw.size())
		{
			break;
		}
		OSCCamera::DecodedMessage lMessage;
		OSCCamera::Decode(&lRaw[i + 10], lLength, lMessage);
		if (lMessage.type == OSCCamera::EMessageType::Pose)
		{
			if (lFirst < 0.0)
			{
				lFirst = lArrival;
			}
			Packet lPacket;
			lPacket.arrival = 100 * second + static_cast<int64_t>((lArrival - lFirst) * 1e9);
			lPacket.pose = lMessage.pose;
			lPackets.push_back(lPacket);
		}
		i += 10 + lLength;
	}
	return lPackets;
}

static void TestRecordedCapture()
{
	std::vector<Packet> lPackets;
	std::vector<std::string> lDirs;
#ifdef OSC_FIXTURE_DIR
	lDirs.push_back(OSC_FIXTURE_DIR);
#endif
	for (const char* lDir : { "tests/fixtures", "fixtures", "../tests/fixtures", "../../tests/fixtures" })
	{
		lDirs.push_back(lDir);
	}
	for (const std::string& lDir : lDirs)
	{
		lPackets = ReadCapture(lDir + "/mobu_session_estimator_wire.oscrec");
		if (!lPackets.empty())
		{
			break;
		}
	}
	CHECK(lPackets.size() > 6000);
	if (lPackets.empty())
	{
		return;
	}

	const Replay lZero = Run(lPackets, Mode(Playback::Mode::Fixed, 0));
	const Replay l40 = Run(lPackets, Mode(Playback::Mode::Fixed, 40 * ms));
	const Replay lAuto = Run(lPackets, Mode(Playback::Mode::Auto));
	const Replay lResponsive = Run(lPackets, Mode(Playback::Mode::Auto, 0, 60 * ms, Playback::Responsiveness::Responsive));
	const Replay lNewest = Run(lPackets, Mode(Playback::Mode::Newest));
	auto Print = [](const char* pName, const Replay& r)
	{
		std::printf("  %-16s rms %.2f p99 %.2f | last 15 s rms %.2f p99 %.2f | interpolated %3.0f %% | delay mean %.1f min %.1f max %.1f ms\n",
			pName, r.rms, r.p99, r.lastRms, r.lastP99, r.interpolated, r.meanDelayMs, r.minDelayMs, r.maxDelayMs);
	};
	Print("delay 0", lZero);
	Print("fixed 40 ms", l40);
	Print("auto balanced", lAuto);
	Print("auto responsive", lResponsive);
	Print("newest", lNewest);

	// today's numbers (Fixed, delay 0)
	CHECK(lZero.rms > 0.5 && lZero.rms < 0.65);
	CHECK(lZero.lastP99 > 3.5 && lZero.lastP99 < 4.5);
	CHECK(lZero.interpolated == 0.0);

	// Auto, p99 + half a spacing
	CHECK(lAuto.interpolated >= 98.0);
	CHECK(lAuto.rms <= 0.05);
	CHECK(lAuto.lastP99 <= 0.1);
	CHECK(lAuto.meanDelayMs <= 30.0);

	// the cheaper end of the dial still beats delay 0 by a wide margin
	CHECK(lResponsive.meanDelayMs < lAuto.meanDelayMs);
	CHECK(lResponsive.rms < lZero.rms * 0.3);
	CHECK(lResponsive.lastP99 < lZero.lastP99 * 0.25);

	// the newest sample is a real sample, but it is drawn when it arrives
	CHECK(lNewest.interpolated == 0.0 && lNewest.meanDelayMs == 0.0);
}

int main()
{
	TestBlendMath();
	TestOrderAndRepeats();
	TestUnstamped();
	TestOffsetAndRender();
	TestOffsetFollowsMinimum();
	TestReboot();
	TestBounded();
	TestFixedExtrapolatesExactly();
	TestAutoBeatsDelayZeroOnAJitteryNetwork();
	TestAutoDelayMoves();
	TestNewestShowsTheNewestSample();
	TestRenderShares();
	TestJitterIgnoresRepeatsAndOneLatePacket();
	TestRecordedCapture();

	std::printf(g_failures == 0 ? "all timeline tests passed\n" : "%d check(s) failed\n", g_failures);
	return g_failures;
}
