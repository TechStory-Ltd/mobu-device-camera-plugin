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
	constexpr const char* kAddressLens = "/livetracker/lens";
	constexpr const char* kAddressDiag = "/livetracker/diag";

	constexpr const char* kTagsPose = ",iffffffiii";
	constexpr const char* kTagsLens = ",iiiffffff";
	constexpr const char* kTagsDiag = ",ih";

	constexpr size_t kMaxDatagramSize = 2048;

	enum class EMessageType : uint8_t
	{
		Unknown,	//!< unknown address, wrong type tags or malformed - to be ignored
		Pose,
		Lens,
		Diag
	};

	//! /livetracker/pose
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
	};

	//! /livetracker/lens
	struct LensMessage
	{
		bool zoomCalibrated{ false };
		bool focusCalibrated{ false };
		bool irisCalibrated{ false };
		float zoomMin{ 0.0f };		//!< mm
		float zoomMax{ 0.0f };
		float focusMin{ 0.0f };		//!< metres
		float focusMax{ 0.0f };
		float irisMin{ 0.0f };		//!< f-number
		float irisMax{ 0.0f };
	};

	//! /livetracker/diag
	struct DiagMessage
	{
		int32_t sequence{ 0 };
		int64_t sendUnixMillis{ 0 };
	};

	struct DecodedMessage
	{
		EMessageType type{ EMessageType::Unknown };
		PoseMessage pose;
		LensMessage lens;
		DiagMessage diag;
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
			if (std::strcmp(tags, kTagsPose) != 0 || argsSize < 40)
			{
				return EMessageType::Unknown;
			}

			float f[6];
			if (!ReadFloats(args + 4, f, 6))
			{
				return EMessageType::Unknown;
			}

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

			out.type = EMessageType::Pose;
		}
		else if (std::strcmp(address, kAddressLens) == 0)
		{
			if (std::strcmp(tags, kTagsLens) != 0 || argsSize < 36)
			{
				return EMessageType::Unknown;
			}

			float f[6];
			if (!ReadFloats(args + 12, f, 6))
			{
				return EMessageType::Unknown;
			}

			out.lens.zoomCalibrated = (ReadI32(args) != 0);
			out.lens.focusCalibrated = (ReadI32(args + 4) != 0);
			out.lens.irisCalibrated = (ReadI32(args + 8) != 0);
			out.lens.zoomMin = f[0];
			out.lens.zoomMax = f[1];
			out.lens.focusMin = f[2];
			out.lens.focusMax = f[3];
			out.lens.irisMin = f[4];
			out.lens.irisMax = f[5];

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

		return out.type;
	}
}
