#pragma once

/**	\file	osc_camera_decoder.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
* Strict decoder for the 6DoF Camera Tracker OSC stream.
* Header-only, no SDK and no platform dependencies, so it can be unit tested standalone.
*
* Every function is bounds checked: a truncated or malformed datagram is rejected, never read past its end.
*/

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>

namespace OSCCamera
{
	constexpr const char* kAddressPose = "/livetracker/pose";
	constexpr const char* kAddressPoseRef = "/livetracker/pose/ref";
	constexpr const char* kAddressLens = "/livetracker/lens";
	constexpr const char* kAddressDiag = "/livetracker/diag";
	constexpr const char* kAddressCmd = "/livetracker/cmd";
	constexpr const char* kAddressLink = "/livetracker/link";
	constexpr const char* kAddressLinkRoles = "/livetracker/link/roles";
	constexpr const char* kAddressUnits = "/livetracker/units";

	constexpr const char* kTagsPose = ",iffffffiiih";
	constexpr const char* kTagsPoseRef = ",iiiihiiisffffffffffffiii";	//!< OSC_PROTOCOL.md section 2.8
	constexpr const char* kTagsLens = ",iiiiffffffff";
	constexpr const char* kTagsDiag = ",ih";
	constexpr const char* kTagsCmd = ",iis";
	constexpr const char* kTagsLink = ",iii";
	constexpr const char* kTagsLinkRoles = ",isssiii";
	constexpr const char* kTagsUnits = ",sf";

	// the layouts of the app before 2026-10-08, they are not decoded - only recognized, to tell the user to update the app
	constexpr const char* kTagsPoseOutdated = ",iffffffiii";
	constexpr const char* kTagsLensOutdated = ",iiiffffff";

	constexpr size_t kMaxDatagramSize = 2048;
	constexpr size_t kMaxCommandLength = 64;	//!< bytes of the command string, without the terminator
	constexpr size_t kMaxMarkerKeyLength = 32;	//!< bytes of a marker key, without the terminator
	constexpr size_t kMaxReferenceKeyLength = 96;	//!< bytes of the Ref key of a pose: 32 UTF-16 code units, up to three bytes each in UTF-8
	constexpr size_t kMaxReferenceKeyUnits = 32;	//!< UTF-16 code units of a marker key
	constexpr int32_t kPoseRefVersion = 1;

	//! the mode of a /livetracker/pose/ref: which of its two poses is the camera
	enum EReferenceMode : int32_t
	{
		kModeWorld = 0,		//!< the world pose
		kModeRefLocal = 1	//!< the local pose, relative to the Ref marker named by the key
	};

	enum EReferenceFollow : int32_t
	{
		kFollowPosition = 0,	//!< the Ref passes on its position
		kFollowHeading = 1		//!< and its heading (counter-clockwise yaw about the vertical)
	};

	enum class EMessageType : uint8_t
	{
		Unknown,	//!< unknown address, wrong type tags or malformed - to be ignored
		Pose,
		Lens,
		Diag,
		Command,
		Link,
		LinkRoles,
		Units,
		OutdatedLayout	//!< a pose or lens message of an older app version: known address, but a layout this decoder does not read
	};

	//! /livetracker/pose/ref (the camera message of the app) or the /livetracker/pose it replaced.
	//! pan, tilt, roll, x, y, z are always the world pose - the camera of a receiver that does not know the Ref, and the recorded one.
	struct PoseMessage
	{
		int32_t cameraId{ 0 };
		float pan{ 0.0f };		//!< degrees
		float tilt{ 0.0f };		//!< degrees
		float roll{ 0.0f };		//!< degrees
		float x{ 0.0f };		//!< metres x Space scale
		float y{ 0.0f };
		float z{ 0.0f };
		int32_t zoom{ 0 };		//!< see the lens section of the protocol, depends on LensMessage calibration flags
		int32_t focus{ 0 };
		int32_t iris{ 0 };
		int64_t captureNanos{ 0 };	//!< when the sensor sample behind the pose was taken, on the phone's own clock (ns since boot), 0 - no sample yet

		// the reference-local envelope, only with /livetracker/pose/ref
		bool hasReference{ false };	//!< the message is a /livetracker/pose/ref (the fields below are read)
		int32_t sessionId{ 0 };		//!< the stream session of the phone, a new one for every stream
		int32_t sequence{ 0 };		//!< output sequence of the datagrams of a session (int32 wrap-aware)
		int32_t mode{ kModeWorld };	//!< EReferenceMode
		int32_t generation{ 0 };	//!< changes whenever the identity of the reference does
		int32_t follow{ kFollowPosition };	//!< EReferenceFollow
		char key[kMaxReferenceKeyLength + 1]{};	//!< the marker key of the Ref, UTF-8, "" for none
		float local[6]{};			//!< x, y, z (as above), pan, tilt, roll relative to the Ref; the pose only in kModeRefLocal
	};

	//! the sample is a camera relative to its Ref marker, not a world pose
	inline bool IsReferenceLocal(const PoseMessage& pose)
	{
		return pose.hasReference && pose.mode == kModeRefLocal;
	}

	//! Two samples may be interpolated: both are world poses (always valid), or both local ones relative to the same Ref of one stream
	//! session - the same key, generation and follow. A pose that is local for one sample and world for the next is not blended.
	inline bool CanBlend(const PoseMessage& a, const PoseMessage& b)
	{
		const bool lLocalA = IsReferenceLocal(a);
		if (lLocalA != IsReferenceLocal(b))
		{
			return false;
		}
		return !lLocalA
			|| (a.sessionId == b.sessionId && a.generation == b.generation && a.follow == b.follow && std::strcmp(a.key, b.key) == 0);
	}

	//! /livetracker/lens
	struct LensMessage
	{
		int32_t cameraId{ 0 };		//!< the camera this describes, filter on it like the pose
		bool zoomCalibrated{ false };
		bool focusCalibrated{ false };
		bool irisCalibrated{ false };
		float zoomMin{ 0.0f };		//!< mm
		float zoomMax{ 0.0f };
		float focusMin{ 0.0f };		//!< metres
		float focusMax{ 0.0f };
		float irisMin{ 0.0f };		//!< f-number
		float irisMax{ 0.0f };
		float sensorWidthMm{ 0.0f };	//!< the sensor (film back) the app uses
		float sensorHeightMm{ 0.0f };
	};

	//! /livetracker/diag
	struct DiagMessage
	{
		int32_t sequence{ 0 };
		int64_t sendUnixMillis{ 0 };
	};

	//! /livetracker/cmd - an event sent when the operator presses a button of the app, three copies per press
	struct CommandMessage
	{
		int32_t cameraId{ 0 };
		int32_t seq{ 0 };		//!< counter of presses, the copies of one press share it
		char command[kMaxCommandLength + 1]{};	//!< UTF-8, NUL terminated, never empty
	};

	//! /livetracker/link - what the phone says about the marker link, about once a second while it streams
	struct LinkMessage
	{
		int32_t state{ 0 };		//!< 0 the link is off, 1 listening (no sender allowed yet), 2 linked
		int32_t markers{ 0 };	//!< markers the phone holds from the sender
		int32_t lastSeq{ 0 };	//!< seq of the newest positions message the phone accepted
	};

	//! /livetracker/link/roles - the marker roles in force on the phone's Live scene, about once a second and at once on a change
	struct LinkRolesMessage
	{
		int32_t rev{ 0 };		//!< the rev of the newest roles message the phone took from the sender
		char focus[kMaxMarkerKeyLength + 1]{};	//!< marker keys, "" for none
		char aim[kMaxMarkerKeyLength + 1]{};
		char ref[kMaxMarkerKeyLength + 1]{};
		int32_t refFollow{ 0 };	//!< 0 position, 1 position + heading
		int32_t lookAt{ 0 };	//!< 0 to 100
		int32_t overridden{ 0 };	//!< bit 0 focus, 1 aim, 2 ref, 3 follow, 4 look-at: the slots the operator set on the phone
	};

	//! /livetracker/units - the unit of every position of the stream and the phone's Space scale, a gain; about once a second while it streams
	struct UnitsMessage
	{
		char unit[8]{};				//!< "m" (metres), the only unit the app sends; "cm" and "mm" are understood too
		float spaceScale{ 1.0f };	//!< how many times as far the camera travels as the phone: already in the positions, for display
	};

	//! the unit of the stream's positions before the stream names one: /livetracker/pose/ref version 1 is defined as metres
	constexpr double kDefaultStreamUnitCm = 100.0;

	/// \return centimetres in one \p pUnit of the stream ("m" 100, "cm" 1, "mm" 0.1), 0 for a unit that is not known
	inline double CentimetresPerUnit(const char* pUnit)
	{
		if (std::strcmp(pUnit, "m") == 0)
		{
			return 100.0;
		}
		if (std::strcmp(pUnit, "cm") == 0)
		{
			return 1.0;
		}
		if (std::strcmp(pUnit, "mm") == 0)
		{
			return 0.1;
		}
		return 0.0;
	}

	//! the commands the app sends by default, anything else is a custom button
	enum class ECommand : uint8_t
	{
		Unknown,
		Play,
		Stop,
		Record,
		GotoStart,
		GotoEnd,
		StepForward,
		StepBack
	};

	struct DecodedMessage
	{
		EMessageType type{ EMessageType::Unknown };
		PoseMessage pose;
		LensMessage lens;
		DiagMessage diag;
		CommandMessage cmd;
		LinkMessage link;
		LinkRolesMessage linkRoles;
		UnitsMessage units;
	};

	/// \return the built-in command of a command string (exact, lower case), Unknown for anything else
	inline ECommand ParseCommand(const char* command)
	{
		struct Entry { const char* name; ECommand command; };
		static const Entry kEntries[] = {
			{ "play", ECommand::Play },
			{ "stop", ECommand::Stop },
			{ "record", ECommand::Record },
			{ "goto_start", ECommand::GotoStart },
			{ "goto_end", ECommand::GotoEnd },
			{ "step_forward", ECommand::StepForward },
			{ "step_back", ECommand::StepBack },
		};

		if (command != nullptr)
		{
			for (const Entry& entry : kEntries)
			{
				if (std::strcmp(command, entry.name) == 0)
				{
					return entry.command;
				}
			}
		}
		return ECommand::Unknown;
	}

	/// Drops the repeats of a button press: the app sends every press three times with the same sequence number.
	/// A key is remembered for a few seconds only - the sequence restarts at 0 when the app does, so there is no high-water mark.
	class CommandDeduplicator
	{
	public:
		static constexpr int64_t kMemoryNanoseconds = 5000000000LL;

		/// \param pSender	address of the sending phone (two phones may share a camera id and a port)
		/// \param pNow		monotonic time, nanoseconds
		/// \return true for the first copy of a press, false for a repeat
		bool Accept(const uint32_t pSender, const int32_t pCameraId, const int32_t pSeq, const int64_t pNow)
		{
			for (const Entry& entry : mEntries)
			{
				if (entry.used && entry.sender == pSender && entry.cameraId == pCameraId && entry.seq == pSeq
					&& pNow - entry.time < kMemoryNanoseconds)
				{
					return false;
				}
			}

			// reuse a free (or expired) slot, else overwrite the oldest one
			Entry* slot = &mEntries[0];
			for (Entry& entry : mEntries)
			{
				if (!entry.used || pNow - entry.time >= kMemoryNanoseconds)
				{
					slot = &entry;
					break;
				}
				if (entry.time < slot->time)
				{
					slot = &entry;
				}
			}

			*slot = Entry{ true, pSender, pCameraId, pSeq, pNow };
			return true;
		}

		void Reset()
		{
			for (Entry& entry : mEntries)
			{
				entry.used = false;
			}
		}

	private:
		struct Entry
		{
			bool used{ false };
			uint32_t sender{ 0 };
			int32_t cameraId{ 0 };
			int32_t seq{ 0 };
			int64_t time{ 0 };
		};

		Entry mEntries[32];
	};

	namespace detail
	{
		inline uint32_t ReadU32(const uint8_t* p)
		{
			return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16)
				| (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
		}

		inline int32_t ReadI32(const uint8_t* p)
		{
			return static_cast<int32_t>(ReadU32(p));
		}

		inline int64_t ReadI64(const uint8_t* p)
		{
			const uint64_t hi = ReadU32(p);
			const uint64_t lo = ReadU32(p + 4);
			return static_cast<int64_t>((hi << 32) | lo);
		}

		inline float ReadF32(const uint8_t* p)
		{
			const uint32_t u = ReadU32(p);
			float f;
			std::memcpy(&f, &u, sizeof(f));
			return f;
		}

		/// read a NUL terminated OSC string padded to a multiple of 4 bytes
		/// \param offset in: start of the string (multiple of 4), out: start of the next element
		/// \return false if the string is not terminated or its padding runs past the datagram end
		inline bool ReadString(const uint8_t* buffer, const size_t length, size_t& offset, const char*& str)
		{
			size_t end = offset;
			while (end < length && buffer[end] != 0)
			{
				++end;
			}
			if (end >= length)
			{
				return false; // no terminator
			}

			// terminator plus padding up to the next 4 byte boundary
			const size_t next = (end + 4) & ~static_cast<size_t>(3);
			if (next > length)
			{
				return false;
			}

			str = reinterpret_cast<const char*>(buffer + offset);
			offset = next;
			return true;
		}

		/// \return true if \p text is well formed UTF-8 (no overlong forms, surrogates or values past U+10FFFF) of at most
		///  \p maxBytes bytes and \p maxUnits UTF-16 code units - the limits of a marker key
		inline bool IsValidKey(const char* text, const size_t maxBytes, const size_t maxUnits)
		{
			const uint8_t* p = reinterpret_cast<const uint8_t*>(text);
			const size_t length = std::strlen(text);
			if (length > maxBytes)
			{
				return false;
			}

			size_t units = 0;
			for (size_t i = 0; i < length;)
			{
				const uint8_t lead = p[i];
				size_t extra;
				uint32_t code;
				uint32_t minimum;
				if (lead < 0x80) { extra = 0; code = lead; minimum = 0; }
				else if ((lead & 0xE0) == 0xC0) { extra = 1; code = lead & 0x1F; minimum = 0x80; }
				else if ((lead & 0xF0) == 0xE0) { extra = 2; code = lead & 0x0F; minimum = 0x800; }
				else if ((lead & 0xF8) == 0xF0) { extra = 3; code = lead & 0x07; minimum = 0x10000; }
				else { return false; }

				for (size_t k = 1; k <= extra; ++k)
				{
					if (i + k >= length || (p[i + k] & 0xC0) != 0x80)
					{
						return false;
					}
					code = (code << 6) | (p[i + k] & 0x3F);
				}
				if (code < minimum || code > 0x10FFFF || (code >= 0xD800 && code <= 0xDFFF))
				{
					return false;
				}

				units += (code >= 0x10000) ? 2 : 1;
				i += extra + 1;
			}
			return units <= maxUnits;
		}

		inline bool ReadFloats(const uint8_t* p, float* values, const size_t count)
		{
			for (size_t i = 0; i < count; ++i)
			{
				values[i] = ReadF32(p + 4 * i);
				if (!std::isfinite(values[i]))
				{
					return false;
				}
			}
			return true;
		}
	}

	/// decode one OSC datagram
	/// \return the message type, or Unknown if the datagram is not one of the messages we understand.
	///  \p out is filled only for the returned type.
	inline EMessageType Decode(const void* data, const size_t length, DecodedMessage& out)
	{
		using namespace detail;

		out.type = EMessageType::Unknown;

		if (data == nullptr || length < 8)
		{
			return EMessageType::Unknown;
		}

		const uint8_t* buffer = static_cast<const uint8_t*>(data);
		size_t offset = 0;
		const char* address = nullptr;
		const char* tags = nullptr;

		if (!ReadString(buffer, length, offset, address) || !ReadString(buffer, length, offset, tags))
		{
			return EMessageType::Unknown;
		}

		const uint8_t* args = buffer + offset;
		const size_t argsSize = length - offset;

		if (std::strcmp(address, kAddressPose) == 0)
		{
			if (std::strcmp(tags, kTagsPoseOutdated) == 0)
			{
				out.type = EMessageType::OutdatedLayout;
				return out.type;
			}
			if (std::strcmp(tags, kTagsPose) != 0 || argsSize < 48)
			{
				return EMessageType::Unknown;
			}

			float f[6];
			if (!ReadFloats(args + 4, f, 6))
			{
				return EMessageType::Unknown;
			}

			out.pose = PoseMessage();
			out.pose.cameraId = ReadI32(args);
			out.pose.pan = f[0];
			out.pose.tilt = f[1];
			out.pose.roll = f[2];
			out.pose.x = f[3];
			out.pose.y = f[4];
			out.pose.z = f[5];
			out.pose.zoom = ReadI32(args + 28);
			out.pose.focus = ReadI32(args + 32);
			out.pose.iris = ReadI32(args + 36);
			out.pose.captureNanos = ReadI64(args + 40);

			out.type = EMessageType::Pose;
		}
		else if (std::strcmp(address, kAddressPoseRef) == 0)
		{
			// version, camera, session, sequence, capture time, mode, generation, follow, then the key, 12 floats and 3 ints
			constexpr size_t kHead = 36;
			constexpr size_t kTail = 12 * 4 + 3 * 4;
			if (std::strcmp(tags, kTagsPoseRef) != 0 || argsSize < kHead)
			{
				return EMessageType::Unknown;
			}

			size_t tailOffset = offset + kHead;
			const char* key = nullptr;
			if (!ReadString(buffer, length, tailOffset, key) || length - tailOffset < kTail
				|| !IsValidKey(key, kMaxReferenceKeyLength, kMaxReferenceKeyUnits))
			{
				return EMessageType::Unknown;
			}

			const int32_t version = ReadI32(args);
			const int32_t mode = ReadI32(args + 24);
			const int32_t follow = ReadI32(args + 32);
			float f[12];
			if (version != kPoseRefVersion || (mode != kModeWorld && mode != kModeRefLocal)
				|| (follow != kFollowPosition && follow != kFollowHeading) || !ReadFloats(buffer + tailOffset, f, 12))
			{
				return EMessageType::Unknown;
			}

			out.pose = PoseMessage();
			out.pose.hasReference = true;
			out.pose.cameraId = ReadI32(args + 4);
			out.pose.sessionId = ReadI32(args + 8);
			out.pose.sequence = ReadI32(args + 12);
			out.pose.captureNanos = ReadI64(args + 16);
			out.pose.mode = mode;
			out.pose.generation = ReadI32(args + 28);
			out.pose.follow = follow;
			std::memcpy(out.pose.key, key, std::strlen(key) + 1);
			for (int i = 0; i < 6; ++i)
			{
				out.pose.local[i] = f[i];
			}
			out.pose.x = f[6];
			out.pose.y = f[7];
			out.pose.z = f[8];
			out.pose.pan = f[9];
			out.pose.tilt = f[10];
			out.pose.roll = f[11];
			out.pose.zoom = ReadI32(buffer + tailOffset + 48);
			out.pose.focus = ReadI32(buffer + tailOffset + 52);
			out.pose.iris = ReadI32(buffer + tailOffset + 56);

			out.type = EMessageType::Pose;
		}
		else if (std::strcmp(address, kAddressLens) == 0)
		{
			if (std::strcmp(tags, kTagsLensOutdated) == 0)
			{
				out.type = EMessageType::OutdatedLayout;
				return out.type;
			}
			if (std::strcmp(tags, kTagsLens) != 0 || argsSize < 48)
			{
				return EMessageType::Unknown;
			}

			float f[8];
			if (!ReadFloats(args + 16, f, 8))
			{
				return EMessageType::Unknown;
			}

			out.lens.cameraId = ReadI32(args);
			out.lens.zoomCalibrated = (ReadI32(args + 4) != 0);
			out.lens.focusCalibrated = (ReadI32(args + 8) != 0);
			out.lens.irisCalibrated = (ReadI32(args + 12) != 0);
			out.lens.zoomMin = f[0];
			out.lens.zoomMax = f[1];
			out.lens.focusMin = f[2];
			out.lens.focusMax = f[3];
			out.lens.irisMin = f[4];
			out.lens.irisMax = f[5];
			out.lens.sensorWidthMm = f[6];
			out.lens.sensorHeightMm = f[7];

			out.type = EMessageType::Lens;
		}
		else if (std::strcmp(address, kAddressDiag) == 0)
		{
			if (std::strcmp(tags, kTagsDiag) != 0 || argsSize < 12)
			{
				return EMessageType::Unknown;
			}

			out.diag.sequence = ReadI32(args);
			out.diag.sendUnixMillis = ReadI64(args + 4);

			out.type = EMessageType::Diag;
		}
		else if (std::strcmp(address, kAddressCmd) == 0)
		{
			if (std::strcmp(tags, kTagsCmd) != 0 || argsSize < 12)
			{
				return EMessageType::Unknown;
			}

			// the string follows the two ints, it is bounds checked against the datagram
			size_t stringOffset = offset + 8;
			const char* command = nullptr;
			if (!ReadString(buffer, length, stringOffset, command))
			{
				return EMessageType::Unknown;
			}

			const size_t commandLength = std::strlen(command);
			if (commandLength == 0 || commandLength > kMaxCommandLength)
			{
				return EMessageType::Unknown;
			}

			out.cmd.cameraId = ReadI32(args);
			out.cmd.seq = ReadI32(args + 4);
			std::memcpy(out.cmd.command, command, commandLength + 1);

			out.type = EMessageType::Command;
		}
		else if (std::strcmp(address, kAddressLink) == 0)
		{
			if (std::strcmp(tags, kTagsLink) != 0 || argsSize < 12)
			{
				return EMessageType::Unknown;
			}

			out.link.state = ReadI32(args);
			out.link.markers = ReadI32(args + 4);
			out.link.lastSeq = ReadI32(args + 8);

			out.type = EMessageType::Link;
		}
		else if (std::strcmp(address, kAddressLinkRoles) == 0)
		{
			if (std::strcmp(tags, kTagsLinkRoles) != 0 || argsSize < 4)
			{
				return EMessageType::Unknown;
			}

			// rev, the three keys (bounds checked against the datagram), then three ints
			size_t argOffset = offset + 4;
			char* keys[3] = { out.linkRoles.focus, out.linkRoles.aim, out.linkRoles.ref };
			for (char* key : keys)
			{
				const char* text = nullptr;
				if (!ReadString(buffer, length, argOffset, text))
				{
					return EMessageType::Unknown;
				}
				const size_t keyLength = std::strlen(text);
				if (keyLength > kMaxMarkerKeyLength)
				{
					return EMessageType::Unknown;
				}
				std::memcpy(key, text, keyLength + 1);
			}
			if (length - argOffset < 12)
			{
				return EMessageType::Unknown;
			}

			out.linkRoles.rev = ReadI32(args);
			out.linkRoles.refFollow = ReadI32(buffer + argOffset);
			out.linkRoles.lookAt = ReadI32(buffer + argOffset + 4);
			out.linkRoles.overridden = ReadI32(buffer + argOffset + 8);

			out.type = EMessageType::LinkRoles;
		}
		else if (std::strcmp(address, kAddressUnits) == 0)
		{
			if (std::strcmp(tags, kTagsUnits) != 0)
			{
				return EMessageType::Unknown;
			}

			// the unit as a string (bounds checked against the datagram), then the gain
			size_t argOffset = offset;
			const char* unit = nullptr;
			if (!ReadString(buffer, length, argOffset, unit) || length - argOffset < 4)
			{
				return EMessageType::Unknown;
			}
			const size_t unitLength = std::strlen(unit);
			if (unitLength == 0 || unitLength >= sizeof(out.units.unit))
			{
				return EMessageType::Unknown;
			}

			std::memcpy(out.units.unit, unit, unitLength + 1);
			out.units.spaceScale = ReadF32(buffer + argOffset);

			out.type = EMessageType::Units;
		}

		return out.type;
	}
}
