/**	\file	osc_camera_decoder_test.cpp
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
*	Standalone test for osc_camera_decoder.h (no MotionBuilder SDK needed).
*	Build target: device_oscCamera_decoder_test (excluded from the default build), the exit code is the number of failed checks.
*/

#include "osc_camera_decoder.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
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
	int64_t captureNanos = 0, const char* address = "/livetracker/pose", const char* tags = ",iffffffiiih")
{
	Bytes b;
	PutString(b, address);
	PutString(b, tags);
	PutI32(b, cam);
	PutF32(b, pan); PutF32(b, tilt); PutF32(b, roll);
	PutF32(b, x); PutF32(b, y); PutF32(b, z);
	PutI32(b, zoom); PutI32(b, focus); PutI32(b, iris);
	if (std::strcmp(tags, ",iffffffiiih") == 0)
	{
		PutI64(b, captureNanos);
	}
	return b;
}

static Bytes MakeLens(int32_t cam, int zc, int fc, int ic, float zmin, float zmax, float fmin, float fmax, float imin, float imax,
	float sensorW = 24.89f, float sensorH = 14.0f)
{
	Bytes b;
	PutString(b, "/livetracker/lens");
	PutString(b, ",iiiiffffffff");
	PutI32(b, cam);
	PutI32(b, zc); PutI32(b, fc); PutI32(b, ic);
	PutF32(b, zmin); PutF32(b, zmax); PutF32(b, fmin); PutF32(b, fmax); PutF32(b, imin); PutF32(b, imax);
	PutF32(b, sensorW); PutF32(b, sensorH);
	return b;
}

static Bytes MakeLink(int32_t state, int32_t markers, int32_t lastSeq, const char* tags = ",iii")
{
	Bytes b;
	PutString(b, "/livetracker/link");
	PutString(b, tags);
	PutI32(b, state); PutI32(b, markers); PutI32(b, lastSeq);
	return b;
}

static Bytes MakeUnits(const char* unit, float spaceScale, const char* tags = ",sf")
{
	Bytes b;
	PutString(b, "/livetracker/units");
	PutString(b, tags);
	PutString(b, unit);
	PutF32(b, spaceScale);
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

static Bytes MakeCmd(int32_t cam, int32_t seq, const std::string& command, const char* tags = ",iis")
{
	Bytes b;
	PutString(b, "/livetracker/cmd");
	PutString(b, tags);
	PutI32(b, cam);
	PutI32(b, seq);
	PutString(b, command);
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
	// the worked example of the protocol (section 6): camera 1, captured at 5123456789012 ns - 84 bytes
	const Bytes packet = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x70, 0x6f, 0x73,
		0x65, 0x00, 0x00, 0x00, 0x2c, 0x69, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x69, 0x69, 0x69, 0x68,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x42, 0x36, 0x00, 0x00, 0xc1, 0x24, 0x00, 0x00,
		0x40, 0x00, 0x00, 0x00, 0x3f, 0x99, 0x99, 0x9a, 0xc0, 0x90, 0x00, 0x00, 0x3e, 0x99, 0x99, 0x9a,
		0x00, 0x01, 0x4c, 0x08, 0x00, 0x00, 0x0d, 0xac, 0x00, 0x00, 0x0a, 0xf0, 0x00, 0x00, 0x04, 0xa8,
		0xe5, 0xd2, 0x6a, 0x14 };

	CHECK(packet.size() == 84);

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
	CHECK(m.pose.captureNanos == 5123456789012LL);

	// the encoder above must produce the very same bytes
	CHECK(MakePose(1, 45.5f, -10.25f, 2.0f, 1.2f, -4.5f, 0.3f, 85000, 3500, 2800, 5123456789012LL) == packet);

	// no sample yet and a negative stamp (never sent, but a signed int64 is read as such)
	const Bytes none = MakePose(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	CHECK(DecodeExact(none, none.size(), m) == OSCCamera::EMessageType::Pose && m.pose.captureNanos == 0);
	const Bytes negative = MakePose(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -5);
	CHECK(DecodeExact(negative, negative.size(), m) == OSCCamera::EMessageType::Pose && m.pose.captureNanos == -5);
}

static void TestOutdatedLayouts()
{
	OSCCamera::DecodedMessage m;

	// the 72 byte pose and the 68 byte lens of the app before 2026-10-08 are recognized, not read
	const Bytes pose = MakePose(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "/livetracker/pose", ",iffffffiii");
	CHECK(pose.size() == 72);
	CHECK(DecodeExact(pose, pose.size(), m) == OSCCamera::EMessageType::OutdatedLayout);

	Bytes lens;
	PutString(lens, "/livetracker/lens");
	PutString(lens, ",iiiffffff");
	for (int i = 0; i < 9; ++i) PutI32(lens, 0);
	CHECK(lens.size() == 68);
	CHECK(DecodeExact(lens, lens.size(), m) == OSCCamera::EMessageType::OutdatedLayout);
}

static void TestLensAndDiag()
{
	OSCCamera::DecodedMessage m;

	const Bytes lens = MakeLens(3, 1, 0, 1, 24.0f, 200.0f, 0.0f, 0.0f, 1.4f, 22.0f, 36.0f, 24.0f);
	CHECK(lens.size() == 84);
	CHECK(DecodeExact(lens, lens.size(), m) == OSCCamera::EMessageType::Lens);
	CHECK(m.lens.cameraId == 3);
	CHECK(m.lens.zoomCalibrated && !m.lens.focusCalibrated && m.lens.irisCalibrated);
	CHECK(m.lens.zoomMin == 24.0f && m.lens.zoomMax == 200.0f);
	CHECK(m.lens.focusMin == 0.0f && m.lens.focusMax == 0.0f);
	CHECK(m.lens.irisMin == 1.4f && m.lens.irisMax == 22.0f);
	CHECK(m.lens.sensorWidthMm == 36.0f && m.lens.sensorHeightMm == 24.0f);

	const Bytes diag = MakeDiag(12345, 1790000000123LL);
	CHECK(diag.size() == 36);
	CHECK(DecodeExact(diag, diag.size(), m) == OSCCamera::EMessageType::Diag);
	CHECK(m.diag.sequence == 12345);
	CHECK(m.diag.sendUnixMillis == 1790000000123LL);
}

static void TestLink()
{
	OSCCamera::DecodedMessage m;

	// the worked example of the protocol (section 10.7): linked, 2 markers, newest seq 7 - 40 bytes
	const Bytes packet = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6c, 0x69, 0x6e,
		0x6b, 0x00, 0x00, 0x00, 0x2c, 0x69, 0x69, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
		0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x07 };
	CHECK(packet.size() == 40);
	CHECK(DecodeExact(packet, packet.size(), m) == OSCCamera::EMessageType::Link);
	CHECK(m.link.state == 2 && m.link.markers == 2 && m.link.lastSeq == 7);
	CHECK(MakeLink(2, 2, 7) == packet);

	const Bytes wrongTags = MakeLink(2, 2, 7, ",iif");
	CHECK(DecodeExact(wrongTags, wrongTags.size(), m) == OSCCamera::EMessageType::Unknown);
}

static void TestUnits()
{
	OSCCamera::DecodedMessage m;

	// the worked example of the protocol (section 2.9): metres, the phone's Space scale 10 - 32 bytes
	const Bytes packet = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x75, 0x6e, 0x69,
		0x74, 0x73, 0x00, 0x00, 0x2c, 0x73, 0x66, 0x00, 0x6d, 0x00, 0x00, 0x00, 0x41, 0x20, 0x00, 0x00 };
	CHECK(packet.size() == 32);
	CHECK(DecodeExact(packet, packet.size(), m) == OSCCamera::EMessageType::Units);
	CHECK(std::strcmp(m.units.unit, "m") == 0 && m.units.spaceScale == 10.0f);
	CHECK(MakeUnits("m", 10.0f) == packet);

	// another unit the plugin knows
	const Bytes millimetres = MakeUnits("mm", 1.0f);
	CHECK(DecodeExact(millimetres, millimetres.size(), m) == OSCCamera::EMessageType::Units);
	CHECK(std::strcmp(m.units.unit, "mm") == 0 && m.units.spaceScale == 1.0f);

	// what the plugin does with the unit: centimetres in one of it, 0 for a unit it does not know
	CHECK(OSCCamera::CentimetresPerUnit("m") == 100.0);
	CHECK(OSCCamera::CentimetresPerUnit("cm") == 1.0);
	CHECK(std::fabs(OSCCamera::CentimetresPerUnit("mm") - 0.1) < 1e-12);
	CHECK(OSCCamera::CentimetresPerUnit("in") == 0.0);
	CHECK(OSCCamera::CentimetresPerUnit("") == 0.0);
	CHECK(OSCCamera::kDefaultStreamUnitCm == 100.0);

	// wrong tags, no unit, a unit too long to keep: not a units message
	const Bytes wrongTags = MakeUnits("m", 10.0f, ",si");
	CHECK(DecodeExact(wrongTags, wrongTags.size(), m) == OSCCamera::EMessageType::Unknown);
	const Bytes empty = MakeUnits("", 10.0f);
	CHECK(DecodeExact(empty, empty.size(), m) == OSCCamera::EMessageType::Unknown);
	const Bytes tooLong = MakeUnits("metres-and-more", 10.0f);
	CHECK(DecodeExact(tooLong, tooLong.size(), m) == OSCCamera::EMessageType::Unknown);
}

static void TestLinkRoles()
{
	OSCCamera::DecodedMessage m;

	// the worked example of the protocol (section 10.7): rev 3, Focus Actor, no Aim, Ref Cart with heading, look-at 75 %,
	// the operator picked the Ref on the phone - 72 bytes
	const Bytes packet = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6c, 0x69, 0x6e,
		0x6b, 0x2f, 0x72, 0x6f, 0x6c, 0x65, 0x73, 0x00, 0x2c, 0x69, 0x73, 0x73, 0x73, 0x69, 0x69, 0x69,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x41, 0x63, 0x74, 0x6f, 0x72, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x43, 0x61, 0x72, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x4b, 0x00, 0x00, 0x00, 0x04 };
	CHECK(packet.size() == 72);
	CHECK(DecodeExact(packet, packet.size(), m) == OSCCamera::EMessageType::LinkRoles);
	CHECK(m.linkRoles.rev == 3);
	CHECK(std::strcmp(m.linkRoles.focus, "Actor") == 0 && m.linkRoles.aim[0] == 0 && std::strcmp(m.linkRoles.ref, "Cart") == 0);
	CHECK(m.linkRoles.refFollow == 1 && m.linkRoles.lookAt == 75 && m.linkRoles.overridden == 4);

	// cut anywhere: not a report
	for (size_t cut = 0; cut < packet.size(); cut += 4)
	{
		CHECK(DecodeExact(packet, cut, m) != OSCCamera::EMessageType::LinkRoles);
	}

	// wrong tags, a key longer than 32 bytes
	Bytes wrongTags = packet;
	wrongTags[31] = 0x66;	// ,isssiif
	CHECK(DecodeExact(wrongTags, wrongTags.size(), m) == OSCCamera::EMessageType::Unknown);

	Bytes longKey(packet.begin(), packet.begin() + 40);
	const std::string key(33, 'x');
	longKey.insert(longKey.end(), key.begin(), key.end());
	while (longKey.size() % 4 != 0 || longKey.back() != 0) longKey.push_back(0);
	longKey.insert(longKey.end(), packet.begin() + 48, packet.end());
	CHECK(DecodeExact(longKey, longKey.size(), m) == OSCCamera::EMessageType::Unknown);
}

static void TestCommand()
{
	OSCCamera::DecodedMessage m;

	// the worked example of the protocol: camera 1, press 7, "play" - 44 bytes
	const Bytes packet = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x63, 0x6d, 0x64,
		0x00, 0x00, 0x00, 0x00, 0x2c, 0x69, 0x69, 0x73, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x07, 0x70, 0x6c, 0x61, 0x79, 0x00, 0x00, 0x00, 0x00 };
	CHECK(packet.size() == 44);
	CHECK(DecodeExact(packet, packet.size(), m) == OSCCamera::EMessageType::Command);
	CHECK(m.cmd.cameraId == 1);
	CHECK(m.cmd.seq == 7);
	CHECK(std::strcmp(m.cmd.command, "play") == 0);
	CHECK(MakeCmd(1, 7, "play") == packet);

	// the longest string: 64 bytes, the largest message is 104 bytes
	const std::string longest(64, 'a');
	const Bytes maxPacket = MakeCmd(255, -1, longest);
	CHECK(maxPacket.size() == 104);
	CHECK(DecodeExact(maxPacket, maxPacket.size(), m) == OSCCamera::EMessageType::Command);
	CHECK(m.cmd.cameraId == 255 && m.cmd.seq == -1);
	CHECK(longest == m.cmd.command);

	// UTF-8 passes through untouched
	const Bytes utf8 = MakeCmd(0, 3, "scène_1");
	CHECK(DecodeExact(utf8, utf8.size(), m) == OSCCamera::EMessageType::Command);
	CHECK(std::strcmp(m.cmd.command, "scène_1") == 0);

	// 65 bytes, empty and wrong tags are rejected
	const Bytes tooLong = MakeCmd(1, 1, std::string(65, 'a'));
	CHECK(DecodeExact(tooLong, tooLong.size(), m) == OSCCamera::EMessageType::Unknown);
	const Bytes empty = MakeCmd(1, 1, "");
	CHECK(DecodeExact(empty, empty.size(), m) == OSCCamera::EMessageType::Unknown);
	const Bytes wrongTags = MakeCmd(1, 1, "play", ",iii");
	CHECK(DecodeExact(wrongTags, wrongTags.size(), m) == OSCCamera::EMessageType::Unknown);

	// a string that is not terminated inside the datagram
	Bytes unterminated = MakeCmd(1, 1, "play");
	unterminated.resize(unterminated.size() - 4);
	unterminated.insert(unterminated.end(), { 'p', 'l', 'a', 'y' });
	CHECK(DecodeExact(unterminated, unterminated.size(), m) == OSCCamera::EMessageType::Unknown);
}

static void TestParseCommand()
{
	using OSCCamera::ECommand;

	CHECK(OSCCamera::ParseCommand("play") == ECommand::Play);
	CHECK(OSCCamera::ParseCommand("stop") == ECommand::Stop);
	CHECK(OSCCamera::ParseCommand("record") == ECommand::Record);
	CHECK(OSCCamera::ParseCommand("goto_start") == ECommand::GotoStart);
	CHECK(OSCCamera::ParseCommand("goto_end") == ECommand::GotoEnd);
	CHECK(OSCCamera::ParseCommand("step_forward") == ECommand::StepForward);
	CHECK(OSCCamera::ParseCommand("step_back") == ECommand::StepBack);

	// custom buttons and anything not exact are not an error, just unknown
	CHECK(OSCCamera::ParseCommand("") == ECommand::Unknown);
	CHECK(OSCCamera::ParseCommand("Play") == ECommand::Unknown);
	CHECK(OSCCamera::ParseCommand("play ") == ECommand::Unknown);
	CHECK(OSCCamera::ParseCommand("rewind") == ECommand::Unknown);
	CHECK(OSCCamera::ParseCommand(nullptr) == ECommand::Unknown);
}

static void TestDeduplicator()
{
	const int64_t ms = 1000000LL;
	const int64_t second = 1000 * ms;
	OSCCamera::CommandDeduplicator d;

	// three copies of one press, about 30 ms apart (or all in the same millisecond)
	CHECK(d.Accept(10, 1, 7, 100 * second));
	CHECK(!d.Accept(10, 1, 7, 100 * second + 30 * ms));
	CHECK(!d.Accept(10, 1, 7, 100 * second + 60 * ms));

	// the next press, another phone, another camera are different keys
	CHECK(d.Accept(10, 1, 8, 100 * second + 500 * ms));
	CHECK(d.Accept(11, 1, 7, 100 * second + 60 * ms));
	CHECK(d.Accept(10, 2, 7, 100 * second + 60 * ms));

	// the app restarted and counts from 0 again: the same key a few seconds later is a new press
	CHECK(!d.Accept(10, 1, 7, 104 * second));
	CHECK(d.Accept(10, 1, 7, 106 * second));

	// more keys than the memory holds: the oldest is overwritten, nothing breaks
	OSCCamera::CommandDeduplicator full;
	for (int i = 0; i < 200; ++i)
	{
		CHECK(full.Accept(1, 1, i, 1000 * second + i * ms));
		CHECK(!full.Accept(1, 1, i, 1000 * second + i * ms + 30 * ms));
	}

	// reset forgets everything
	d.Reset();
	CHECK(d.Accept(10, 1, 7, 106 * second + ms));
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
	const Bytes wrongTags = MakePose(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "/livetracker/pose", ",iffffffiif");
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

	const Bytes packets[6] = {
		MakePose(1, 45.5f, -10.25f, 2.0f, 1.2f, -4.5f, 0.3f, 85000, 3500, 2800, 5123456789012LL),
		MakeLens(1, 1, 1, 1, 24.0f, 200.0f, 0.3f, 10.0f, 1.4f, 22.0f),
		MakeUnits("m", 10.0f),
		MakeLink(1, 0, 0),
		MakeDiag(1, 2),
		MakeCmd(1, 7, "goto_start") };

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
	const Bytes validCmd = MakeCmd(1, 7, "step_forward");

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
			b = (i % 4 == 1) ? valid : validCmd;
			for (int k = 0; k < 4; ++k) b[std::rand() % b.size()] = static_cast<uint8_t>(std::rand());
			b.resize(std::rand() % (b.size() + 1));
		}
		DecodeExact(b, b.size(), m);
	}
	CHECK(true);
}

// --- /livetracker/pose/ref (OSC_PROTOCOL.md section 2.8)

static Bytes FromHex(const char* hex)
{
	Bytes b;
	for (size_t i = 0; hex[i] != 0 && hex[i + 1] != 0; i += 2)
	{
		const char pair[3] = { hex[i], hex[i + 1], 0 };
		b.push_back(static_cast<uint8_t>(std::strtoul(pair, nullptr, 16)));
	}
	return b;
}

struct RefPoseFields
{
	int32_t version{ 1 };
	int32_t cameraId{ 1 };
	int32_t session{ 7 };
	int32_t seq{ 42 };
	int64_t capture{ 123456789012LL };
	int32_t mode{ 1 };
	int32_t generation{ 3 };
	int32_t follow{ 1 };
	std::string key{ "RefCart" };
	float local[6]{ 1.5f, -2.0f, 0.8f, 10.0f, -5.0f, 3.0f };
	float world[6]{ 4.0f, 5.0f, 6.0f, 20.0f, -5.0f, 3.0f };
	int32_t zoom{ 100 }, focus{ 200 }, iris{ 300 };
};

static Bytes MakePoseRef(const RefPoseFields& f, const char* tags = ",iiiihiiisffffffffffffiii")
{
	Bytes b;
	PutString(b, "/livetracker/pose/ref");
	PutString(b, tags);
	PutI32(b, f.version); PutI32(b, f.cameraId); PutI32(b, f.session); PutI32(b, f.seq);
	PutI64(b, f.capture);
	PutI32(b, f.mode); PutI32(b, f.generation); PutI32(b, f.follow);
	PutString(b, f.key);
	for (float v : f.local) PutF32(b, v);
	for (float v : f.world) PutF32(b, v);
	PutI32(b, f.zoom); PutI32(b, f.focus); PutI32(b, f.iris);
	return b;
}

static void TestPoseRef()
{
	// the bytes of the app (the shared fixture wire_ref_pose "the lab cart"), the encoder here must make the same ones
	const Bytes lab = FromHex("2f6c697665747261636b65722f706f73652f7265660000002c6969696968696969736666666666666666666666666969690000000000000100000001000000070000002a0000001cbe991a1400000001000000030000000152656643617274003fc00000c00000003f4ccccd41200000c0a00000404000004080000040a0000040c0000041a00000c0a000004040000000000064000000c80000012c");
	CHECK(MakePoseRef(RefPoseFields()) == lab);

	OSCCamera::DecodedMessage m;
	CHECK(DecodeExact(lab, lab.size(), m) == OSCCamera::EMessageType::Pose);
	CHECK(m.pose.hasReference);
	CHECK(m.pose.cameraId == 1 && m.pose.sessionId == 7 && m.pose.sequence == 42);
	CHECK(m.pose.captureNanos == 123456789012LL);
	CHECK(m.pose.mode == OSCCamera::kModeRefLocal && m.pose.generation == 3 && m.pose.follow == OSCCamera::kFollowHeading);
	CHECK(std::strcmp(m.pose.key, "RefCart") == 0);
	CHECK(m.pose.local[0] == 1.5f && m.pose.local[1] == -2.0f && m.pose.local[2] == 0.8f);
	CHECK(m.pose.local[3] == 10.0f && m.pose.local[4] == -5.0f && m.pose.local[5] == 3.0f);
	// the world pose stays in the fields of the pose
	CHECK(m.pose.x == 4.0f && m.pose.y == 5.0f && m.pose.z == 6.0f);
	CHECK(m.pose.pan == 20.0f && m.pose.tilt == -5.0f && m.pose.roll == 3.0f);
	CHECK(m.pose.zoom == 100 && m.pose.focus == 200 && m.pose.iris == 300);
	CHECK(OSCCamera::IsReferenceLocal(m.pose));

	// WORLD, no reference: the local values are placeholders
	const Bytes world = FromHex("2f6c697665747261636b65722f706f73652f7265660000002c6969696968696969736666666666666666666666666969690000000000000100000002ffffffff00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000bfa000003f00000040000000c32a000042a00000c1f00000000000000000000000000000");
	CHECK(DecodeExact(world, world.size(), m) == OSCCamera::EMessageType::Pose);
	CHECK(m.pose.hasReference && m.pose.mode == OSCCamera::kModeWorld && m.pose.sessionId == -1 && m.pose.key[0] == 0);
	CHECK(m.pose.x == -1.25f && m.pose.y == 0.5f && m.pose.z == 2.0f && m.pose.pan == -170.0f && m.pose.tilt == 80.0f && m.pose.roll == -30.0f);
	CHECK(!OSCCamera::IsReferenceLocal(m.pose));

	// negative numbers and a key of 32 bytes
	const Bytes extremes = FromHex("2f6c697665747261636b65722f706f73652f7265660000002c69696969686969697366666666666666666666666669696900000000000001000000017ffffffffffffffbfffffffde78ee60000000001fffffffe000000004b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b4b00000000c2c90000437a4000be0000004333800000000000c23400003f80000040000000404000004080000040a0000040c00000ffffffff0000ffff00000001");
	CHECK(DecodeExact(extremes, extremes.size(), m) == OSCCamera::EMessageType::Pose);
	CHECK(m.pose.sessionId == 2147483647 && m.pose.sequence == -5 && m.pose.captureNanos == -9000000000LL && m.pose.generation == -2);
	CHECK(std::strlen(m.pose.key) == 32 && m.pose.local[0] == -100.5f && m.pose.zoom == -1 && m.pose.focus == 65535 && m.pose.iris == 1);

	// the longest key: 32 UTF-16 units, 96 bytes of UTF-8
	const Bytes wide = FromHex("2f6c697665747261636b65722f706f73652f7265660000002c6969696968696969736666666666666666666666666969690000000000000100000001000000070000002b0000001cbe991a15000000010000000300000001e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83e58f83000000003f80000040000000404000000000000000000000000000004130000041b000004204000000000000000000000000000000000064000000c80000012c");
	CHECK(DecodeExact(wide, wide.size(), m) == OSCCamera::EMessageType::Pose);
	CHECK(std::strlen(m.pose.key) == 96);

	// the former message is still a world pose, with no reference
	const Bytes legacy = MakePose(2, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7, 8, 9, 10);
	CHECK(DecodeExact(legacy, legacy.size(), m) == OSCCamera::EMessageType::Pose);
	CHECK(!m.pose.hasReference && !OSCCamera::IsReferenceLocal(m.pose) && m.pose.key[0] == 0);
}

static void TestPoseRefRejected()
{
	OSCCamera::DecodedMessage m;
	auto Rejected = [&m](const Bytes& b) { return DecodeExact(b, b.size(), m) == OSCCamera::EMessageType::Unknown; };

	CHECK(!Rejected(MakePoseRef(RefPoseFields())));

	RefPoseFields f;
	f.version = 2;			CHECK(Rejected(MakePoseRef(f)));
	f = RefPoseFields();
	f.version = 0;			CHECK(Rejected(MakePoseRef(f)));
	f = RefPoseFields();
	f.mode = 2;				CHECK(Rejected(MakePoseRef(f)));
	f.mode = -1;			CHECK(Rejected(MakePoseRef(f)));
	f = RefPoseFields();
	f.follow = 2;			CHECK(Rejected(MakePoseRef(f)));
	f = RefPoseFields();
	f.local[2] = std::numeric_limits<float>::quiet_NaN();		CHECK(Rejected(MakePoseRef(f)));
	f = RefPoseFields();
	f.world[0] = std::numeric_limits<float>::infinity();		CHECK(Rejected(MakePoseRef(f)));

	// wrong type tags: one float short, the old pose tags
	CHECK(Rejected(MakePoseRef(RefPoseFields(), ",iiiihiiisfffffffffffiii")));
	CHECK(Rejected(MakePoseRef(RefPoseFields(), ",iiiihiiissffffffffffffiii")));

	// a key past the limit: 33 units, 97 bytes; the key is an identity, never cut
	f = RefPoseFields();
	f.key = std::string(33, 'K');
	CHECK(Rejected(MakePoseRef(f)));
	f.key = std::string(32, 'K');
	CHECK(!Rejected(MakePoseRef(f)));
	f.key.clear();
	CHECK(!Rejected(MakePoseRef(f)));
	f.key = "";
	for (int i = 0; i < 17; ++i) f.key += "\xF0\x9F\x98\x80";		// 17 characters of two UTF-16 units each = 34 units, 68 bytes
	CHECK(Rejected(MakePoseRef(f)));
	f.key = "";
	for (int i = 0; i < 16; ++i) f.key += "\xF0\x9F\x98\x80";		// 32 units
	CHECK(!Rejected(MakePoseRef(f)));

	// not UTF-8: a lone continuation byte, a truncated sequence, an overlong form, a surrogate
	f = RefPoseFields();
	f.key = "\x80";					CHECK(Rejected(MakePoseRef(f)));
	f.key = "ab\xE5\x8F";			CHECK(Rejected(MakePoseRef(f)));
	f.key = "\xC0\xAF";				CHECK(Rejected(MakePoseRef(f)));
	f.key = "\xED\xA0\x80";			CHECK(Rejected(MakePoseRef(f)));

	// every truncation is rejected, none reads past the end
	const Bytes full = MakePoseRef(RefPoseFields());
	for (size_t length = 0; length < full.size(); ++length)
	{
		CHECK(DecodeExact(full, length, m) == OSCCamera::EMessageType::Unknown);
	}
	CHECK(DecodeExact(full, full.size(), m) == OSCCamera::EMessageType::Pose);
}

int main()
{
	TestPoseRef();
	TestPoseRefRejected();
	TestWorkedExample();
	TestOutdatedLayouts();
	TestLensAndDiag();
	TestLink();
	TestUnits();
	TestLinkRoles();
	TestCommand();
	TestParseCommand();
	TestDeduplicator();
	TestRejected();
	TestTruncation();
	TestGarbage();

	std::printf(g_failures == 0 ? "all decoder tests passed\n" : "%d check(s) failed\n", g_failures);
	return g_failures;
}
