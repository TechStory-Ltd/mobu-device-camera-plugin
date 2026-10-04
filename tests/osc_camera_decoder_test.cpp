/**	\file	osc_camera_decoder_test.cpp
*	Standalone test for osc_camera_decoder.h (no MotionBuilder SDK needed).
*	Build target: device_oscCamera_decoder_test (excluded from the default build), the exit code is the number of failed checks.
*/

#include "osc_camera_decoder.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static int g_failures = 0;

#define CHECK(cond) \
	do { if (!(cond)) { ++g_failures; std::printf("FAILED %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

using Bytes = std::vector<uint8_t>;

// --- tiny OSC encoder, used to craft test packets independently from the decoder

static void PutString(Bytes& b, const std::string& s)
{
	b.insert(b.end(), s.begin(), s.end());
	b.push_back(0);
	while (b.size() % 4 != 0) b.push_back(0);
}

static void PutI32(Bytes& b, int32_t v)
{
	const uint32_t u = static_cast<uint32_t>(v);
	for (int shift = 24; shift >= 0; shift -= 8) b.push_back(static_cast<uint8_t>(u >> shift));
}

static void PutI64(Bytes& b, int64_t v)
{
	const uint64_t u = static_cast<uint64_t>(v);
	for (int shift = 56; shift >= 0; shift -= 8) b.push_back(static_cast<uint8_t>(u >> shift));
}

static void PutF32(Bytes& b, float v)
{
	uint32_t u;
	std::memcpy(&u, &v, 4);
	PutI32(b, static_cast<int32_t>(u));
}

static Bytes MakePose(int32_t cam, float pan, float tilt, float roll, float x, float y, float z, int32_t zoom, int32_t focus, int32_t iris,
	const char* address = "/livetracker/pose", const char* tags = ",iffffffiii")
{
	Bytes b;
	PutString(b, address);
	PutString(b, tags);
	PutI32(b, cam);
	PutF32(b, pan); PutF32(b, tilt); PutF32(b, roll);
	PutF32(b, x); PutF32(b, y); PutF32(b, z);
	PutI32(b, zoom); PutI32(b, focus); PutI32(b, iris);
	return b;
}

static Bytes MakeLens(int zc, int fc, int ic, float zmin, float zmax, float fmin, float fmax, float imin, float imax)
{
	Bytes b;
	PutString(b, "/livetracker/lens");
	PutString(b, ",iiiffffff");
	PutI32(b, zc); PutI32(b, fc); PutI32(b, ic);
	PutF32(b, zmin); PutF32(b, zmax); PutF32(b, fmin); PutF32(b, fmax); PutF32(b, imin); PutF32(b, imax);
	return b;
}

static Bytes MakeDiag(int32_t seq, int64_t millis)
{
	Bytes b;
	PutString(b, "/livetracker/diag");
	PutString(b, ",ih");
	PutI32(b, seq);
	PutI64(b, millis);
	return b;
}

// decode from an exactly sized heap copy, so an out of bounds read hits the heap guard / sanitizer
static OSCCamera::EMessageType DecodeExact(const Bytes& src, size_t length, OSCCamera::DecodedMessage& out)
{
	std::vector<uint8_t> exact(src.begin(), src.begin() + length);
	return OSCCamera::Decode(exact.data(), exact.size(), out);
}

// --- tests

static void TestWorkedExample()
{
	// a worked example: the 72 byte /livetracker/pose packet
	const Bytes packet = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x70, 0x6f, 0x73,
		0x65, 0x00, 0x00, 0x00, 0x2c, 0x69, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x69, 0x69, 0x69, 0x00,
		0x00, 0x00, 0x00, 0x01, 0x42, 0x36, 0x00, 0x00, 0xc1, 0x24, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
		0x3f, 0x99, 0x99, 0x9a, 0xc0, 0x90, 0x00, 0x00, 0x3e, 0x99, 0x99, 0x9a, 0x00, 0x01, 0x4c, 0x08,
		0x00, 0x00, 0x0d, 0xac, 0x00, 0x00, 0x0a, 0xf0 };

	CHECK(packet.size() == 72);

	OSCCamera::DecodedMessage m;
	CHECK(DecodeExact(packet, packet.size(), m) == OSCCamera::EMessageType::Pose);
	CHECK(m.pose.cameraId == 1);
	CHECK(m.pose.pan == 45.5f);
	CHECK(m.pose.tilt == -10.25f);
	CHECK(m.pose.roll == 2.0f);
	CHECK(m.pose.x == 1.2f);
	CHECK(m.pose.y == -4.5f);
	CHECK(m.pose.z == 0.3f);
	CHECK(m.pose.zoom == 85000);
	CHECK(m.pose.focus == 3500);
	CHECK(m.pose.iris == 2800);

	// the encoder above must produce the very same bytes
	CHECK(MakePose(1, 45.5f, -10.25f, 2.0f, 1.2f, -4.5f, 0.3f, 85000, 3500, 2800) == packet);
}

static void TestLensAndDiag()
{
	OSCCamera::DecodedMessage m;

	const Bytes lens = MakeLens(1, 0, 1, 24.0f, 200.0f, 0.0f, 0.0f, 1.4f, 22.0f);
	CHECK(lens.size() == 68);
	CHECK(DecodeExact(lens, lens.size(), m) == OSCCamera::EMessageType::Lens);
	CHECK(m.lens.zoomCalibrated && !m.lens.focusCalibrated && m.lens.irisCalibrated);
	CHECK(m.lens.zoomMin == 24.0f && m.lens.zoomMax == 200.0f);
	CHECK(m.lens.focusMin == 0.0f && m.lens.focusMax == 0.0f);
	CHECK(m.lens.irisMin == 1.4f && m.lens.irisMax == 22.0f);

	const Bytes diag = MakeDiag(12345, 1790000000123LL);
	CHECK(diag.size() == 36);
	CHECK(DecodeExact(diag, diag.size(), m) == OSCCamera::EMessageType::Diag);
	CHECK(m.diag.sequence == 12345);
	CHECK(m.diag.sendUnixMillis == 1790000000123LL);
}

static void TestRejected()
{
	OSCCamera::DecodedMessage m;

	// unknown address (e.g. the raw IMU stream, or an address added by a later app version)
	Bytes unknown;
	PutString(unknown, "/livetracker/imu");
	PutString(unknown, ",h");
	PutI64(unknown, 1);
	CHECK(DecodeExact(unknown, unknown.size(), m) == OSCCamera::EMessageType::Unknown);

	// known address, wrong type tags
	const Bytes wrongTags = MakePose(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, "/livetracker/pose", ",iffffffiif");
	CHECK(DecodeExact(wrongTags, wrongTags.size(), m) == OSCCamera::EMessageType::Unknown);

	// non finite floats
	const Bytes nan = MakePose(1, std::nanf(""), 0, 0, 0, 0, 0, 0, 0, 0);
	CHECK(DecodeExact(nan, nan.size(), m) == OSCCamera::EMessageType::Unknown);

	// a bundle is not sent by the app and is not supported
	Bytes bundle;
	PutString(bundle, "#bundle");
	PutI64(bundle, 1);
	PutI32(bundle, 4);
	PutI32(bundle, 0);
	CHECK(DecodeExact(bundle, bundle.size(), m) == OSCCamera::EMessageType::Unknown);

	// empty, null
	CHECK(OSCCamera::Decode(nullptr, 100, m) == OSCCamera::EMessageType::Unknown);
	const uint8_t one = 0;
	CHECK(OSCCamera::Decode(&one, 0, m) == OSCCamera::EMessageType::Unknown);
}

static void TestTruncation()
{
	OSCCamera::DecodedMessage m;

	const Bytes packets[3] = {
		MakePose(1, 45.5f, -10.25f, 2.0f, 1.2f, -4.5f, 0.3f, 85000, 3500, 2800),
		MakeLens(1, 1, 1, 24.0f, 200.0f, 0.3f, 10.0f, 1.4f, 22.0f),
		MakeDiag(1, 2) };

	for (const Bytes& packet : packets)
	{
		// every strictly shorter prefix must be rejected without reading past its end
		for (size_t len = 0; len < packet.size(); ++len)
		{
			CHECK(DecodeExact(packet, len, m) == OSCCamera::EMessageType::Unknown);
		}
		// trailing bytes after the arguments are tolerated
		Bytes longer = packet;
		longer.push_back(0);
		longer.push_back(0);
		CHECK(DecodeExact(longer, longer.size(), m) != OSCCamera::EMessageType::Unknown);
	}
}

static void TestGarbage()
{
	// random bytes and mutated valid packets: must never crash, must never read out of bounds
	OSCCamera::DecodedMessage m;
	std::srand(12345);

	const Bytes valid = MakePose(1, 45.5f, -10.25f, 2.0f, 1.2f, -4.5f, 0.3f, 85000, 3500, 2800);

	for (int i = 0; i < 200000; ++i)
	{
		Bytes b;
		if (i % 2 == 0)
		{
			b.resize(std::rand() % 130);
			for (uint8_t& v : b) v = static_cast<uint8_t>(std::rand());
		}
		else
		{
			b = valid;
			for (int k = 0; k < 4; ++k) b[std::rand() % b.size()] = static_cast<uint8_t>(std::rand());
			b.resize(std::rand() % (b.size() + 1));
		}
		DecodeExact(b, b.size(), m);
	}
	CHECK(true);
}

int main()
{
	TestWorkedExample();
	TestLensAndDiag();
	TestRejected();
	TestTruncation();
	TestGarbage();

	std::printf(g_failures == 0 ? "all decoder tests passed\n" : "%d check(s) failed\n", g_failures);
	return g_failures;
}
