/**	\file	osc_marker_link_test.cpp
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
*	Standalone test for osc_marker_link.h (no MotionBuilder SDK needed).
*	Build target: device_oscCamera_marker_test (excluded from the default build), the exit code is the number of failed checks.
*/

#include "osc_marker_link.h"

#include <cmath>
#include <cstdio>
#include <limits>

static int g_failures = 0;

#define CHECK(cond) \
	do { if (!(cond)) { ++g_failures; std::printf("FAILED %s:%d  %s\n", __FILE__, __LINE__, #cond); } } while (0)

namespace M = OSCCamera::Marker;
using Bytes = std::vector<uint8_t>;

static const int64_t ms = 1000000LL;
static const int64_t second = 1000 * ms;

static bool Near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) <= eps; }

static void TestGoldenBytes()
{
	// the golden bytes of the protocol (section 10.7), the ones the app and the Blender add-on check too
	const Bytes def = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6d, 0x61, 0x72,
		0x6b, 0x65, 0x72, 0x2f, 0x64, 0x65, 0x66, 0x00, 0x2c, 0x73, 0x73, 0x69, 0x69, 0x66, 0x66, 0x66,
		0x69, 0x69, 0x00, 0x00, 0x41, 0x63, 0x74, 0x6f, 0x72, 0x00, 0x00, 0x00, 0x54, 0x68, 0x65, 0x20,
		0x41, 0x63, 0x74, 0x6f, 0x72, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02,
		0x3f, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x3f, 0xe6, 0x66, 0x66, 0x00, 0x00, 0x00, 0x02,
		0x00, 0x00, 0x00, 0x01 };
	CHECK(def.size() == 84);
	CHECK(M::EncodeDef("Actor", "The Actor", M::kShapeCapsule, 2, 0.5, 0.5, 1.8, M::kAnchorTop, true) == def);

	const Bytes list = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6d, 0x61, 0x72,
		0x6b, 0x65, 0x72, 0x2f, 0x6c, 0x69, 0x73, 0x74, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x69, 0x73, 0x73,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x41, 0x63, 0x74, 0x6f, 0x72, 0x00, 0x00, 0x00,
		0x43, 0x61, 0x72, 0x74, 0x00, 0x00, 0x00, 0x00 };
	CHECK(list.size() == 56);
	CHECK(M::EncodeList({ "Actor", "Cart" }) == list);

	const Bytes markers = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6d, 0x61, 0x72,
		0x6b, 0x65, 0x72, 0x73, 0x00, 0x00, 0x00, 0x00, 0x2c, 0x68, 0x69, 0x69, 0x69, 0x66, 0x66, 0x66,
		0x66, 0x69, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xe3, 0x60,
		0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00,
		0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0xb4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
		0x40, 0xa0, 0x00, 0x00, 0x40, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	CHECK(markers.size() == 96);
	CHECK(M::EncodePositions(1500000, 7, { { 1.0, 2.0, 0.0, 90.0 }, { 5.0, 6.0, 0.0, 0.0 } }) == markers);

	const Bytes bye = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6d, 0x61, 0x72,
		0x6b, 0x65, 0x72, 0x2f, 0x62, 0x79, 0x65, 0x00, 0x2c, 0x00, 0x00, 0x00 };
	CHECK(bye.size() == 28);
	CHECK(M::EncodeBye() == bye);
}

static void TestLimits()
{
	// a full scene: 16 markers are 448 bytes of positions
	std::vector<M::Position> sixteen(16);
	CHECK(M::EncodePositions(0, 0, sixteen).size() == 448);

	// sizes are clamped to what the phone takes (0.05 to 20 m), numbers are made finite
	const Bytes def = M::EncodeDef("k", "n", 99, 99, 0.0, 1000.0, std::nan(""), 99, true);
	const Bytes ref = M::EncodeDef("k", "n", M::kShapeCapsule, 7, 0.05, 20.0, 0.05, M::kAnchorTop, true);
	CHECK(def == ref);

	const Bytes pos = M::EncodePositions(0, 0, { { 1e12, std::nan(""), -1e12, std::numeric_limits<double>::infinity() } });
	const Bytes posRef = M::EncodePositions(0, 0, { { 1e6, 0.0, -1e6, 0.0 } });
	CHECK(pos == posRef);
}

static void TestKeys()
{
	CHECK(M::MakeKeys({ "Actor", "Cart" }) == std::vector<std::string>({ "Actor", "Cart" }));
	CHECK(M::MakeKeys({ "Cube", "Cube", "Cube" }) == std::vector<std::string>({ "Cube", "Cube 2", "Cube 3" }));
	CHECK(M::MakeKeys({ "" })[0] == "Marker");

	// cut to 32 bytes and still unique
	const std::string longName(40, 'a');
	const std::vector<std::string> keys = M::MakeKeys({ longName, longName });
	CHECK(keys[0].size() == 32 && keys[1].size() == 32 && keys[0] != keys[1]);

	// never cut in the middle of a UTF-8 character
	std::string utf8;
	for (int i = 0; i < 20; ++i) utf8 += "\xC3\xA9";	// 20 x e-acute = 40 bytes
	const std::string cut = M::CutUtf8(utf8, 32);
	CHECK(cut.size() == 32 && cut.substr(30) == "\xC3\xA9");
	const std::string cutOdd = M::CutUtf8("a" + utf8, 32);
	CHECK(cutOdd.size() == 31);
}

static void TestSettingsText()
{
	std::vector<M::Settings> rows;
	M::Settings a = M::DefaultSettings("Actor", 0);
	a.name = "The | Actor %7C; line\nbreak";
	a.shape = M::kShapeCapsule;
	a.color = 2;
	a.width = 0.45;
	a.depth = 0.3;
	a.height = 1.8;
	a.pivot = M::kPivotCentre;
	a.anchor = M::kAnchorTop;
	a.visible = false;
	rows.push_back(a);
	rows.push_back(M::DefaultSettings("Cart", 1));

	const std::string text = M::SerializeSettings(rows);
	const std::vector<M::Settings> back = M::ParseSettings(text);
	CHECK(back.size() == 2);
	CHECK(back[0].model == "Actor" && back[0].name == a.name);
	CHECK(back[0].shape == M::kShapeCapsule && back[0].color == 2 && !back[0].visible);
	CHECK(Near(back[0].width, 0.45) && Near(back[0].depth, 0.3) && Near(back[0].height, 1.8));
	CHECK(back[0].pivot == M::kPivotCentre && back[0].anchor == M::kAnchorTop);
	CHECK(back[1].model == "Cart" && back[1].color == 1);
	CHECK(M::SerializeSettings(back) == text);
	CHECK(text.find('\n') == std::string::npos && text.find('\r') == std::string::npos);

	// not ours, empty, damaged: no rows, never a crash
	CHECK(M::ParseSettings("").empty());
	CHECK(M::ParseSettings("hello").empty());
	CHECK(M::ParseSettings("OSCMARKERS1;only|three|fields").empty());
	const std::vector<M::Settings> damaged = M::ParseSettings("OSCMARKERS1;A|A|1|99|99|x|y|z|9|9");
	CHECK(damaged.size() == 1 && damaged[0].shape == M::kShapeCount - 1 && damaged[0].color == M::kColorCount - 1);
	CHECK(Near(damaged[0].width, 0.5) && damaged[0].pivot == M::kPivotCount - 1);
	CHECK(M::ParseSettings("OSCMARKERS1;;;A|A|1|1|0|1|1|1|0|0;").size() == 1);
}

static void TestReconcile()
{
	std::vector<M::Settings> saved = { M::DefaultSettings("A", 0), M::DefaultSettings("B", 1), M::DefaultSettings("C", 2) };
	saved[0].width = 1.0; saved[1].width = 2.0; saved[2].width = 3.0;

	// same list: every model gets its own row
	std::vector<M::Settings> r = M::Reconcile(saved, { "A", "B", "C" });
	CHECK(r.size() == 3 && Near(r[0].width, 1.0) && Near(r[1].width, 2.0) && Near(r[2].width, 3.0));

	// reordered by hand or a script: rows follow the models
	r = M::Reconcile(saved, { "C", "A", "B" });
	CHECK(Near(r[0].width, 3.0) && Near(r[1].width, 1.0) && Near(r[2].width, 2.0));

	// a model removed from the middle, a new one at the end gets the defaults and never the row of another
	r = M::Reconcile(saved, { "A", "C", "New" });
	CHECK(Near(r[0].width, 1.0) && Near(r[1].width, 3.0));
	CHECK(r[2].model == "New" && r[2].name == "New" && Near(r[2].width, 0.5));

	// two models with the same name use two rows, not one twice
	std::vector<M::Settings> twins = { M::DefaultSettings("Cube", 0), M::DefaultSettings("Cube", 1) };
	twins[0].width = 1.0; twins[1].width = 2.0;
	r = M::Reconcile(twins, { "Cube", "Cube" });
	CHECK(Near(r[0].width, 1.0) && Near(r[1].width, 2.0));
	r = M::Reconcile(twins, { "Cube", "Cube", "Cube" });
	CHECK(Near(r[0].width, 1.0) && Near(r[1].width, 2.0) && Near(r[2].width, 0.5));

	// no saved rows: defaults, with the colors going round
	r = M::Reconcile({}, { "A", "B" });
	CHECK(r.size() == 2 && r[0].color == 0 && r[1].color == 1);
	CHECK(M::Reconcile({ M::DefaultSettings("A", 0) }, {}).empty());
}

static void TestPlace()
{
	OSCCamera::ConvertSettings convert;	// the stream is in metres, the scene in cm: the defaults

	M::Settings settings = M::DefaultSettings("A", 0);
	settings.height = 1.8;

	// the wire point (1 right, 2 forward, 0 up) is (right 100, up 0, back -200) cm in MotionBuilder; facing forward (-Z) is 90 degrees
	const double position[3] = { 100.0, 0.0, -200.0 };
	const double forwardAxis[3] = { 0.0, 0.0, -1.0 };
	M::Position p = M::Place(position, forwardAxis, settings, convert);
	CHECK(Near(p.x, 1.0) && Near(p.y, 2.0) && Near(p.z, 0.0) && Near(p.yaw, 90.0));

	// a model facing right (MotionBuilder +X) is heading 0, facing back (+Z) 270 / -90, facing left 180
	const double rightAxis[3] = { 1.0, 0.0, 0.0 };
	CHECK(Near(M::Place(position, rightAxis, settings, convert).yaw, 0.0));
	const double backAxis[3] = { 0.0, 0.0, 1.0 };
	CHECK(Near(M::Place(position, backAxis, settings, convert).yaw, -90.0));
	const double leftAxis[3] = { -1.0, 0.0, 0.0 };
	CHECK(Near(std::fabs(M::Place(position, leftAxis, settings, convert).yaw), 180.0));

	// a tilted model has only a heading: the tilt of its axis does not change it
	const double tiltedAxis[3] = { 0.0, 0.7071067811865476, -0.7071067811865476 };
	CHECK(Near(M::Place(position, tiltedAxis, settings, convert).yaw, 90.0));

	// the origin in the centre of the marker: the base is half the height lower
	settings.pivot = M::kPivotCentre;
	const double centre[3] = { 100.0, 90.0, -200.0 };	// 0.9 m up = the middle of a 1.8 m marker standing on the floor
	p = M::Place(centre, forwardAxis, settings, convert);
	CHECK(Near(p.z, 0.0));
	// a point is the point itself
	settings.shape = M::kShapePoint;
	p = M::Place(centre, forwardAxis, settings, convert);
	CHECK(Near(p.z, 0.9));

	// a stream that names centimetres: the wire is cm already
	convert.streamUnitCm = OSCCamera::CentimetresPerUnit("cm");
	p = M::Place(position, forwardAxis, settings, convert);
	CHECK(Near(p.x, 100.0) && Near(p.y, 200.0) && Near(p.z, 0.0));

	// the Space Scale property is an extra multiplier of the camera: the marker goes through the same factor the other way
	convert.streamUnitCm = 100.0;
	convert.spaceScale = 2.0;
	p = M::Place(position, forwardAxis, settings, convert);
	CHECK(Near(p.x, 0.5) && Near(p.y, 1.0) && Near(p.z, 0.0));

	// the inverse of the camera conversion: placing the camera at this pose and the marker at the same spot gives the same wire numbers
	OSCCamera::ConvertSettings scene;	// the defaults
	OSCCamera::PoseMessage pose;
	pose.x = 3.5f; pose.y = -1.25f; pose.z = 0.75f;
	const OSCCamera::CameraState camera = OSCCamera::Convert(pose, nullptr, scene);
	M::Settings pointSettings = M::DefaultSettings("P", 0);
	pointSettings.shape = M::kShapePoint;
	p = M::Place(camera.position, camera.forward, pointSettings, scene);
	CHECK(Near(p.x, pose.x, 1e-5) && Near(p.y, pose.y, 1e-5) && Near(p.z, pose.z, 1e-5));
}

static M::State State(const char* key, double x, double y, double z)
{
	M::State s;
	s.key = key;
	s.name = key;
	s.position.x = x; s.position.y = y; s.position.z = z;
	return s;
}

static int CountAddress(const std::vector<M::Datagram>& out, const char* address)
{
	int count = 0;
	for (const M::Datagram& d : out)
	{
		if (std::strcmp(reinterpret_cast<const char*>(d.data()), address) == 0) ++count;
	}
	return count;
}

static void TestSender()
{
	M::Sender sender;
	std::vector<M::Datagram> out;
	std::vector<M::State> markers = { State("Actor", 0, 0, 0), State("Cart", 5, 6, 0) };
	int64_t now = 100 * second;

	// the first step sends everything: two definitions, the list, the positions
	CHECK(sender.Due(now));
	sender.Step(now, markers, 0, out);
	CHECK(CountAddress(out, M::kAddressDef) == 2 && CountAddress(out, M::kAddressList) == 1 && CountAddress(out, M::kAddressMarkers) == 1);
	CHECK(sender.Seq() == 1);

	// nothing moved, nothing changed: nothing is sent until the heartbeat
	out.clear();
	now += 100 * ms;
	sender.Step(now, markers, 0, out);
	CHECK(out.empty());
	now += 450 * ms;
	sender.Step(now, markers, 0, out);
	CHECK(CountAddress(out, M::kAddressMarkers) == 1 && CountAddress(out, M::kAddressDef) == 0);

	// a move is sent at once, with the next sequence number
	out.clear();
	now += 20 * ms;
	markers[1].position.x = 5.5;
	sender.Step(now, markers, 0, out);
	CHECK(CountAddress(out, M::kAddressMarkers) == 1 && sender.Seq() == 3);

	// the scene time alone (an animation playing with nothing moved) is a reason too
	out.clear();
	now += 20 * ms;
	sender.Step(now, markers, 40000, out);
	CHECK(CountAddress(out, M::kAddressMarkers) == 1);

	// the definitions come again after a second, and at once on a change (but not more than 10 times a second)
	out.clear();
	now += 1 * second;
	sender.Step(now, markers, 40000, out);
	CHECK(CountAddress(out, M::kAddressDef) == 2 && CountAddress(out, M::kAddressList) == 1);

	out.clear();
	now += 30 * ms;
	markers[0].color = 4;
	sender.Step(now, markers, 40000, out);
	CHECK(CountAddress(out, M::kAddressDef) == 0);		// too soon after the last definitions
	now += 100 * ms;
	sender.Step(now, markers, 40000, out);
	CHECK(CountAddress(out, M::kAddressDef) == 2);

	// a size change under a centimetre is not worth a message, a bigger one is
	out.clear();
	now += 150 * ms;
	markers[0].height = 1.005;
	sender.Step(now, markers, 40000, out);
	CHECK(CountAddress(out, M::kAddressDef) == 0);
	now += 150 * ms;
	markers[0].height = 1.5;
	sender.Step(now, markers, 40000, out);
	CHECK(CountAddress(out, M::kAddressDef) == 2);

	// at most 16 markers are sent
	M::Sender big;
	std::vector<M::State> many;
	for (int i = 0; i < 20; ++i) many.push_back(State(("M" + std::to_string(i)).c_str(), i, 0, 0));
	out.clear();
	big.Step(now, many, 0, out);
	CHECK(CountAddress(out, M::kAddressDef) == 16);

	// no more than 60 steps a second
	M::Sender fast;
	CHECK(fast.Due(now));
	fast.Step(now, markers, 0, out);
	CHECK(!fast.Due(now + 10 * ms) && fast.Due(now + 17 * ms));

	// reset: starts over, the sequence too
	fast.Reset();
	CHECK(fast.Seq() == 0 && fast.Due(now));
}

static void TestStatusText()
{
	CHECK(M::LinkStatusText(false, 0, 0, 1.0) == "Waiting for the phone to answer");
	CHECK(M::LinkStatusText(false, 0, 0, 10.0).find("does not answer") != std::string::npos);
	CHECK(M::LinkStatusText(true, 0, 0, 10.0).find("off") != std::string::npos);
	CHECK(M::LinkStatusText(true, 1, 0, 10.0).find("allow") != std::string::npos);
	CHECK(M::LinkStatusText(true, 2, 1, 10.0) == "Phone linked, 1 marker");
	CHECK(M::LinkStatusText(true, 2, 3, 10.0) == "Phone linked, 3 markers");
}

// --- the roles (section 10.10), the same cases as the Blender add-on's test_rolesync.py

static const int kFocusBit = 1, kAimBit = 2, kRefBit = 4, kFollowBit = 8, kLookBit = 16;

static M::Roles MakeRoles(const char* focus, const char* aim, const char* ref, int follow = M::kRefFollowPosition, int lookAt = M::kLookAtPhone)
{
	M::Roles r;
	r.focus = focus; r.aim = aim; r.ref = ref; r.refFollow = follow; r.lookAt = lookAt;
	return r;
}

static M::RolesReport MakeReport(int32_t rev, int overridden, const M::Roles& roles)
{
	M::RolesReport rep;
	rep.rev = rev; rep.roles = roles; rep.overridden = overridden;
	return rep;
}

static void TestRolesGoldenBytes()
{
	// rev 3: Focus Actor, no Aim, Ref Cart passing on position + heading, look-at 75 % - 68 bytes, the empty Aim key is one word of NULs
	const Bytes roles = {
		0x2f, 0x6c, 0x69, 0x76, 0x65, 0x74, 0x72, 0x61, 0x63, 0x6b, 0x65, 0x72, 0x2f, 0x6d, 0x61, 0x72,
		0x6b, 0x65, 0x72, 0x2f, 0x72, 0x6f, 0x6c, 0x65, 0x73, 0x00, 0x00, 0x00, 0x2c, 0x69, 0x73, 0x73,
		0x73, 0x69, 0x69, 0x00, 0x00, 0x00, 0x00, 0x03, 0x41, 0x63, 0x74, 0x6f, 0x72, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x43, 0x61, 0x72, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
		0x00, 0x00, 0x00, 0x4b };
	CHECK(roles.size() == 68);
	CHECK(M::EncodeRoles(3, "Actor", "", "Cart", M::kRefFollowHeading, 75) == roles);

	// out of the limits: the phone would drop it, so it is clamped
	const M::Datagram clamped = M::EncodeRoles(1, std::string(40, 'k'), "", "", 7, 300);
	CHECK(clamped == M::EncodeRoles(1, std::string(32, 'k'), "", "", M::kRefFollowPosition, 100));
	CHECK(M::EncodeRoles(1, "", "", "", 0, -5) == M::EncodeRoles(1, "", "", "", 0, M::kLookAtPhone));
}

static void TestRolesSettings()
{
	// the roles are saved with the row, a combination too; the one role of the plugins of before is written beside them (Ref wins)
	std::vector<M::Settings> rows = { M::DefaultSettings("Actor", 0), M::DefaultSettings("Cart", 1) };
	rows[0].roles = M::kRoleFocus | M::kRoleAim;
	rows[1].roles = M::kRoleRef | M::kRoleFocus;
	const std::string text = M::SerializeSettings(rows);
	CHECK(text.find("|0|1|1|3;") != std::string::npos);
	CHECK(text.size() > 4 && text.compare(text.size() - 4, 4, "|3|5") == 0);
	const std::vector<M::Settings> back = M::ParseSettings(text);
	// the second row keeps Focus as it is read after the first: one marker per role
	CHECK(back.size() == 2 && back[0].roles == (M::kRoleFocus | M::kRoleAim) && back[1].roles == M::kRoleRef);
	rows[0].roles = M::kRoleAim;
	CHECK(M::ParseSettings(M::SerializeSettings(rows))[1].roles == (M::kRoleRef | M::kRoleFocus));
	CHECK(M::SerializeSettings({ M::DefaultSettings("A", 0) }).find("|0|0") != std::string::npos);

	// a scene saved before the roles: ten fields, no role
	const std::vector<M::Settings> old = M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0");
	CHECK(old.size() == 1 && old[0].roles == M::kRoleNone);
	// a scene saved with one role per marker: eleven fields, 1 Focus, 2 Aim, 3 Ref
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|1")[0].roles == M::kRoleFocus);
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|2")[0].roles == M::kRoleAim);
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|3")[0].roles == M::kRoleRef);
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|9")[0].roles == M::kRoleRef);
	// the set is read over the one role, and kept to the three roles
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|3|7")[0].roles == M::kRoleAll);
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|3|99")[0].roles == M::kRoleAll);
	CHECK(M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|3|x")[0].roles == M::kRoleNone);

	// one marker per role: the row given the roles keeps them, the other rows keep the rest of theirs
	std::vector<M::Settings> three = { M::DefaultSettings("A", 0), M::DefaultSettings("B", 1), M::DefaultSettings("C", 2) };
	three[0].roles = M::kRoleAim | M::kRoleRef;
	three[2].roles = M::kRoleFocus;
	three[1].roles = M::kRoleAim | M::kRoleFocus;
	M::KeepOneRole(three, 1);
	CHECK(three[0].roles == M::kRoleRef && three[1].roles == (M::kRoleAim | M::kRoleFocus) && three[2].roles == M::kRoleNone);
	const std::vector<M::Settings> twice = M::ParseSettings("OSCMARKERS1;A|A|1|1|0|1|1|1|0|0|3;B|B|1|1|0|1|1|1|0|0|3");
	CHECK(twice.size() == 2 && twice[0].roles == M::kRoleRef && twice[1].roles == M::kRoleNone);

	// the roles of the markers that are sent, by their keys; a marker with two roles is named in both slots
	std::vector<M::State> states = { State("Actor", 0, 0, 0), State("Cart", 0, 0, 0), State("Lamp", 0, 0, 0) };
	states[0].roles = M::kRoleFocus;
	states[1].roles = M::kRoleRef;
	CHECK(M::RolesOf(states, M::kRefFollowHeading, 75) == MakeRoles("Actor", "", "Cart", M::kRefFollowHeading, 75));
	CHECK(M::RolesOf({}, 5, 500) == MakeRoles("", "", "", M::kRefFollowPosition, 100));
	states[0].roles = M::kRoleNone;
	states[1].roles = M::kRoleAll;
	CHECK(M::RolesOf(states, 0, -1) == MakeRoles("Cart", "Cart", "Cart"));
	states[1].roles = M::kRoleFocus | M::kRoleRef;
	states[2].roles = M::kRoleAim;
	CHECK(M::RolesOf(states, 0, -1) == MakeRoles("Cart", "Lamp", "Cart"));
}

static void TestMirrorRoles()
{
	const std::vector<std::string> keys = { "Actor", "Cart", "", "Lamp" };
	auto makeRows = [](int a, int b, int c, int d)
	{
		std::vector<M::Settings> rows = { M::DefaultSettings("Actor", 0), M::DefaultSettings("Cart", 1), M::DefaultSettings("Off", 2),
			M::DefaultSettings("Lamp", 3) };
		rows[0].roles = a; rows[1].roles = b; rows[2].roles = c; rows[3].roles = d;
		return rows;
	};
	auto rolesOf = [](const std::vector<M::Settings>& rows)
	{
		return std::vector<int>{ rows[0].roles, rows[1].roles, rows[2].roles, rows[3].roles };
	};

	{	// a second role from the phone goes beside the first
		std::vector<M::Settings> rows = makeRows(M::kRoleNone, M::kRoleRef, 0, 0);
		M::MirrorRoles(rows, keys, MakeRoles("Cart", "", "Cart"), kFocusBit);
		CHECK(rolesOf(rows) == (std::vector<int>{ 0, M::kRoleRef | M::kRoleFocus, 0, 0 }));
	}
	{	// the role moves: it is taken off the marker that had it, which keeps its others
		std::vector<M::Settings> rows = makeRows(M::kRoleFocus | M::kRoleAim, M::kRoleRef, 0, 0);
		M::MirrorRoles(rows, keys, MakeRoles("Cart", "Actor", "Cart"), kFocusBit);
		CHECK(rolesOf(rows) == (std::vector<int>{ M::kRoleAim, M::kRoleRef | M::kRoleFocus, 0, 0 }));
	}
	{	// all three on one marker, in one report
		std::vector<M::Settings> rows = makeRows(M::kRoleFocus, M::kRoleAim, 0, M::kRoleRef);
		M::MirrorRoles(rows, keys, MakeRoles("Lamp", "Lamp", "Lamp"), kFocusBit | kAimBit | kRefBit);
		CHECK(rolesOf(rows) == (std::vector<int>{ 0, 0, 0, M::kRoleAll }));
	}
	{	// a role the phone drops leaves the marker's others
		std::vector<M::Settings> rows = makeRows(M::kRoleAll, 0, 0, 0);
		M::MirrorRoles(rows, keys, MakeRoles("Actor", "", "Actor"), kAimBit);
		CHECK(rolesOf(rows) == (std::vector<int>{ M::kRoleFocus | M::kRoleRef, 0, 0, 0 }));
	}
	{	// only the slots of the mask, and a key that is no row's changes nothing
		std::vector<M::Settings> rows = makeRows(M::kRoleFocus, M::kRoleAim, 0, 0);
		M::MirrorRoles(rows, keys, MakeRoles("Lamp", "Ghost", "Lamp"), kAimBit);
		CHECK(rolesOf(rows) == (std::vector<int>{ M::kRoleFocus, M::kRoleAim, 0, 0 }));
	}
	{	// the follow and the look-at are not the rows'
		std::vector<M::Settings> rows = makeRows(M::kRoleFocus, 0, 0, 0);
		M::MirrorRoles(rows, keys, MakeRoles("", "", "", 1, 40), kFollowBit | kLookBit);
		CHECK(rolesOf(rows) == (std::vector<int>{ M::kRoleFocus, 0, 0, 0 }));
	}
}

static void TestRoleSync()
{
	int64_t now = 100 * second;

	{	// the message is the roles with the rev
		M::RoleSync sync;
		sync.Local(MakeRoles("Actor", "", "Car", M::kRefFollowHeading, 75), now);
		CHECK(sync.Rev() == 1 && sync.Message() == M::EncodeRoles(1, "Actor", "", "Car", 1, 75));
	}
	{	// a choice made on the phone is mirrored, and that is not the artist's change
		M::RoleSync sync;
		sync.Local(MakeRoles("", "Actor", "Car"), now);
		const M::RolesReport rep = MakeReport(1, kRefBit, MakeRoles("", "Actor", "Truck"));
		const int mask = sync.Report(rep, true, now);
		CHECK(mask == kRefBit);
		sync.Mirrored(rep.roles, mask);
		CHECK(sync.Current().ref == "Truck");
		sync.Local(MakeRoles("", "Actor", "Truck"), now);
		CHECK(!sync.Pending(M::kSlotRef, now));
	}
	{	// mirroring off: nothing is written, the difference is shown
		M::RoleSync sync;
		sync.Local(MakeRoles("", "", "Car"), now);
		const M::RolesReport rep = MakeReport(1, kRefBit, MakeRoles("", "", "Truck"));
		CHECK(sync.Report(rep, false, now) == 0);
		CHECK(sync.Differs(rep) == kRefBit);
		CHECK(M::RolesStatusText(rep, sync.Differs(rep)) == "Phone: Ref = Truck (set on phone)");
		CHECK(M::RolesStatusText(rep, 0).empty());
	}
	{	// a role just changed on the device is not undone by a stale report
		M::RoleSync sync;
		sync.Local(MakeRoles("", "", "Car"), now);
		sync.Local(MakeRoles("", "", "Boat"), now);
		CHECK(sync.Report(MakeReport(1, kRefBit, MakeRoles("", "", "Truck")), true, now) == 0);
		// the phone takes the change (no longer the operator's): reports are mirrored again
		CHECK(sync.Report(MakeReport(1, 0, MakeRoles("", "", "Boat")), true, now) == 0);
		CHECK(!sync.Pending(M::kSlotRef, now));
		CHECK(sync.Report(MakeReport(1, kRefBit, MakeRoles("", "", "Truck")), true, now) == kRefBit);
	}
	{	// a pending role gives up waiting after three seconds
		M::RoleSync sync;
		sync.Local(MakeRoles("", "", "Car"), now);
		sync.Local(MakeRoles("", "", "Boat"), now);
		CHECK(sync.Report(MakeReport(1, kRefBit, MakeRoles("", "", "Truck")), true, now + 3100 * ms) == kRefBit);
	}
	{	// "Send roles to phone" raises the rev, a report of the old one is not mirrored
		M::RoleSync sync;
		sync.Local(MakeRoles("", "", "Car"), now);
		sync.SendAgain(now);
		CHECK(sync.Rev() == 2 && sync.Message() == M::EncodeRoles(2, "", "", "Car", 0, -1));
		CHECK(sync.Report(MakeReport(1, kRefBit, MakeRoles("", "", "Truck")), true, now + 5 * second) == 0);
		CHECK(sync.Report(MakeReport(2, 0, MakeRoles("", "", "Car")), true, now + 5 * second) == 0);
	}
	{	// the look-at is mirrored only when the device sets it
		M::RoleSync sync;
		sync.Local(MakeRoles("", "", "", 0, M::kLookAtPhone), now);
		CHECK(sync.Report(MakeReport(1, kLookBit, MakeRoles("", "", "", 0, 25)), true, now) == 0);
		M::RoleSync other;
		other.Local(MakeRoles("", "", "", 0, 50), now);
		CHECK(other.Report(MakeReport(1, kLookBit, MakeRoles("", "", "", 0, 25)), true, now) == kLookBit);
	}
	{	// only the slots the operator set are mirrored
		M::RoleSync sync;
		sync.Local(MakeRoles("Actor", "Actor", "", 0), now);
		CHECK(sync.Report(MakeReport(1, kFollowBit, MakeRoles("Lamp", "Actor", "", 1)), true, now) == kFollowBit);
	}
	{	// the status lists every role the operator set, in the order of the slots
		const M::RolesReport rep = MakeReport(1, kFocusBit | kAimBit | kFollowBit | kLookBit, MakeRoles("", "Actor", "", 1, 40));
		CHECK(M::RolesStatusText(rep, kFocusBit | kAimBit | kFollowBit | kLookBit)
			== "Phone: Focus = none, Aim = Actor, Ref follow = Position + heading, Look-at = 40 % (set on phone)");
	}
}

static void TestSenderRoles()
{
	M::Sender sender;
	std::vector<M::Datagram> out;
	const std::vector<M::State> markers = { State("Actor", 0, 4, 0) };
	int64_t now = 100 * second;

	// with the definitions, after the key list
	const M::Datagram a = M::EncodeRoles(1, "", "Actor", "", 0, -1);
	sender.Step(now, markers, 0, out, &a);
	int listAt = -1, rolesAt = -1;
	for (size_t i = 0; i < out.size(); ++i)
	{
		const char* address = reinterpret_cast<const char*>(out[i].data());
		if (std::strcmp(address, M::kAddressList) == 0) listAt = static_cast<int>(i);
		if (std::strcmp(address, M::kAddressRoles) == 0) rolesAt = static_cast<int>(i);
	}
	CHECK(listAt >= 0 && rolesAt > listAt);

	// nothing new: not again; a change: at once; and once a second with the definitions
	out.clear();
	now += 100 * ms;
	sender.Step(now, markers, 0, out, &a);
	CHECK(CountAddress(out, M::kAddressRoles) == 0);
	out.clear();
	now += 100 * ms;
	const M::Datagram b = M::EncodeRoles(1, "", "Actor", "Actor", 0, -1);
	sender.Step(now, markers, 0, out, &b);
	CHECK(CountAddress(out, M::kAddressRoles) == 1);
	CHECK(std::find(out.begin(), out.end(), b) != out.end());
	out.clear();
	now += 1 * second;
	sender.Step(now, markers, 0, out, &b);
	CHECK(CountAddress(out, M::kAddressRoles) == 1);

	// no roles: no roles message
	M::Sender quiet;
	out.clear();
	quiet.Step(now, markers, 0, out);
	CHECK(CountAddress(out, M::kAddressRoles) == 0);
}

int main()
{
	TestGoldenBytes();
	TestLimits();
	TestKeys();
	TestSettingsText();
	TestReconcile();
	TestPlace();
	TestSender();
	TestStatusText();
	TestRolesGoldenBytes();
	TestRolesSettings();
	TestMirrorRoles();
	TestRoleSync();
	TestSenderRoles();

	std::printf(g_failures == 0 ? "all marker link tests passed\n" : "%d check(s) failed\n", g_failures);
	return g_failures;
}
