#pragma once

/**	\file	osc_marker_link.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
* The marker link (OSC_PROTOCOL.md section 10): the scene objects of MotionBuilder sent to the phone as markers it can aim at and
* hold focus on. Header-only, no SDK dependency, so it can be unit tested standalone.
*
*  - the encoders of the five messages (/livetracker/marker/def, .../list, /livetracker/markers, .../roles, .../bye);
*  - MarkerSender: decides what to send now (definitions once a second or on a change, positions on a change and at least
*    twice a second, a sequence number, the roles with the definitions and on a change, the goodbye);
*  - the settings of a marker as the operator edits them in the device table, and their text form that is saved in the scene;
*  - the placement: a model of the MotionBuilder scene to the position and heading in the stream frame the phone works in;
*  - RoleSync: the Focus, Aim and Ref roles between the device and the phone (section 10.10), the same rules as the Blender
*    add-on's rolesync.py: a marker holds any combination of the roles, the phone follows the device, the operator overrides one role
*    on the phone until the device changes it, and with "Mirror the phone" on the device takes the operator's choices in (MirrorRoles).
*/

#include "osc_camera_math.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace OSCCamera
{
	namespace Marker
	{
		constexpr const char* kAddressDef = "/livetracker/marker/def";
		constexpr const char* kAddressList = "/livetracker/marker/list";
		constexpr const char* kAddressMarkers = "/livetracker/markers";
		constexpr const char* kAddressBye = "/livetracker/marker/bye";
		constexpr const char* kAddressRoles = "/livetracker/marker/roles";

		constexpr int kMaxMarkers = 16;
		constexpr size_t kMaxKeyLength = 32;
		constexpr double kMinSizeMetres = 0.05;
		constexpr double kMaxSizeMetres = 20.0;
		constexpr double kMaxValue = 1.0e6;

		enum EShape { kShapePoint = 0, kShapeBox = 1, kShapeSphere = 2, kShapeCapsule = 3, kShapeCount = 4 };
		enum EAnchor { kAnchorBase = 0, kAnchorCentre = 1, kAnchorTop = 2, kAnchorCount = 3 };
		enum EPivot { kPivotBase = 0, kPivotCentre = 1, kPivotCount = 2 };	//!< where the origin of the model is, in the marker
		constexpr int kColorCount = 8;	//!< orange, blue, green, purple, yellow, red, white, teal

		//! what the phone does with a marker, as bits: Focus (the focus follows its distance), Aim (the soft look-at turns the camera toward
		//! it), Ref (the marker is the camera's parent: an offset, or a vehicle the camera rides on). A marker holds any combination of them
		//! (OSC_PROTOCOL.md 2026-11-01), one marker per role. The bit of a role is 1 << its ERoleSlot.
		enum ERole { kRoleNone = 0, kRoleFocus = 1 << 0, kRoleAim = 1 << 1, kRoleRef = 1 << 2, kRoleAll = kRoleFocus | kRoleAim | kRoleRef };
		//! the one role of a marker saved before the combinations, still written for the plugins of before
		enum ELegacyRole { kLegacyRoleNone = 0, kLegacyRoleFocus = 1, kLegacyRoleAim = 2, kLegacyRoleRef = 3, kLegacyRoleCount = 4 };
		//! what the Ref passes on to the camera
		enum ERefFollow { kRefFollowPosition = 0, kRefFollowHeading = 1 };
		constexpr int kLookAtPhone = -1;	//!< the look-at strength is left to the operator on the phone

		using Datagram = std::vector<uint8_t>;

		// --- the encoders

		namespace detail
		{
			inline void PutString(Datagram& pOut, const std::string& pString)
			{
				pOut.insert(pOut.end(), pString.begin(), pString.end());
				pOut.push_back(0);
				while (pOut.size() % 4 != 0)
				{
					pOut.push_back(0);
				}
			}

			inline void PutI32(Datagram& pOut, const int32_t pValue)
			{
				const uint32_t lValue = static_cast<uint32_t>(pValue);
				for (int lShift = 24; lShift >= 0; lShift -= 8)
				{
					pOut.push_back(static_cast<uint8_t>(lValue >> lShift));
				}
			}

			inline void PutI64(Datagram& pOut, const int64_t pValue)
			{
				const uint64_t lValue = static_cast<uint64_t>(pValue);
				for (int lShift = 56; lShift >= 0; lShift -= 8)
				{
					pOut.push_back(static_cast<uint8_t>(lValue >> lShift));
				}
			}

			inline void PutF32(Datagram& pOut, const float pValue)
			{
				uint32_t lValue;
				std::memcpy(&lValue, &pValue, sizeof(lValue));
				PutI32(pOut, static_cast<int32_t>(lValue));
			}

			/// a finite number within the limits of the phone, anything else would make it drop the message
			inline float Sane(const double pValue)
			{
				if (!std::isfinite(pValue))
				{
					return 0.0f;
				}
				return static_cast<float>(std::min(std::max(pValue, -kMaxValue), kMaxValue));
			}

			inline float SaneSize(const double pMetres)
			{
				if (!std::isfinite(pMetres))
				{
					return static_cast<float>(kMinSizeMetres);
				}
				return static_cast<float>(std::min(std::max(pMetres, kMinSizeMetres), kMaxSizeMetres));
			}
		}

		/// /livetracker/marker/def ,ssiifffii: key name shape color width depth height anchor flags (bit 0 visible). Sizes in metres.
		inline Datagram EncodeDef(const std::string& pKey, const std::string& pName, const int pShape, const int pColor,
			const double pWidth, const double pDepth, const double pHeight, const int pAnchor, const bool pVisible)
		{
			using namespace detail;
			Datagram lOut;
			PutString(lOut, kAddressDef);
			PutString(lOut, ",ssiifffii");
			PutString(lOut, pKey);
			PutString(lOut, pName);
			PutI32(lOut, std::min(std::max(pShape, 0), static_cast<int>(kShapeCount) - 1));
			PutI32(lOut, std::min(std::max(pColor, 0), kColorCount - 1));
			PutF32(lOut, SaneSize(pWidth));
			PutF32(lOut, SaneSize(pDepth));
			PutF32(lOut, SaneSize(pHeight));
			PutI32(lOut, std::min(std::max(pAnchor, 0), static_cast<int>(kAnchorCount) - 1));
			PutI32(lOut, pVisible ? 1 : 0);
			return lOut;
		}

		/// /livetracker/marker/list ,i then s for each key: the keys in the order the positions count them
		inline Datagram EncodeList(const std::vector<std::string>& pKeys)
		{
			using namespace detail;
			Datagram lOut;
			PutString(lOut, kAddressList);
			std::string lTags = ",i";
			lTags.append(pKeys.size(), 's');
			PutString(lOut, lTags);
			PutI32(lOut, static_cast<int32_t>(pKeys.size()));
			for (const std::string& lKey : pKeys)
			{
				PutString(lOut, lKey);
			}
			return lOut;
		}

		struct Position
		{
			double x{ 0.0 };	//!< stream frame, the unit of the stream's positions (metres, the Space scale gain of the phone already in them)
			double y{ 0.0 };
			double z{ 0.0 };
			double yaw{ 0.0 };	//!< degrees, counter-clockwise from the stream +X
		};

		/// /livetracker/markers ,hii then iffff for each marker: sceneTimeUs seq count, then index x y z yawDeg
		inline Datagram EncodePositions(const int64_t pSceneTimeUs, const int32_t pSeq, const std::vector<Position>& pPositions)
		{
			using namespace detail;
			Datagram lOut;
			PutString(lOut, kAddressMarkers);
			std::string lTags = ",hii";
			for (size_t i = 0; i < pPositions.size(); ++i)
			{
				lTags += "iffff";
			}
			PutString(lOut, lTags);
			PutI64(lOut, pSceneTimeUs);
			PutI32(lOut, pSeq);
			PutI32(lOut, static_cast<int32_t>(pPositions.size()));
			for (size_t i = 0; i < pPositions.size(); ++i)
			{
				PutI32(lOut, static_cast<int32_t>(i));
				PutF32(lOut, Sane(pPositions[i].x));
				PutF32(lOut, Sane(pPositions[i].y));
				PutF32(lOut, Sane(pPositions[i].z));
				PutF32(lOut, Sane(pPositions[i].yaw));
			}
			return lOut;
		}

		inline std::string CutUtf8(const std::string& pText, const size_t pMax);

		/// /livetracker/marker/roles ,isssii: rev focus aim ref refFollow lookAt. The keys of the Focus, Aim and Ref markers ("" for none), what the
		/// Ref passes on (0 position, 1 position + heading) and the look-at strength (0 to 100, -1 to leave it to the phone). A key is cut to
		/// 32 bytes and the numbers are clamped, the phone drops a message outside the limits whole.
		inline Datagram EncodeRoles(const int32_t pRev, const std::string& pFocus, const std::string& pAim, const std::string& pRef,
			const int pRefFollow, const int pLookAt)
		{
			using namespace detail;
			Datagram lOut;
			PutString(lOut, kAddressRoles);
			PutString(lOut, ",isssii");
			PutI32(lOut, pRev);
			for (const std::string* lKey : { &pFocus, &pAim, &pRef })
			{
				PutString(lOut, CutUtf8(*lKey, kMaxKeyLength));
			}
			PutI32(lOut, (pRefFollow == kRefFollowHeading) ? kRefFollowHeading : kRefFollowPosition);
			PutI32(lOut, std::min(std::max(pLookAt, kLookAtPhone), 100));
			return lOut;
		}

		/// /livetracker/marker/bye, no arguments
		inline Datagram EncodeBye()
		{
			using namespace detail;
			Datagram lOut;
			PutString(lOut, kAddressBye);
			PutString(lOut, ",");
			return lOut;
		}

		// --- keys

		/// cut a UTF-8 string to at most \p pMax bytes, on a character boundary
		inline std::string CutUtf8(const std::string& pText, const size_t pMax)
		{
			if (pText.size() <= pMax)
			{
				return pText;
			}
			size_t lEnd = pMax;
			while (lEnd > 0 && (static_cast<uint8_t>(pText[lEnd]) & 0xC0) == 0x80)
			{
				--lEnd;
			}
			return pText.substr(0, lEnd);
		}

		/// marker keys from model names: 1 to 32 bytes, never twice the same (a number is added)
		inline std::vector<std::string> MakeKeys(const std::vector<std::string>& pNames)
		{
			std::vector<std::string> lKeys;
			for (const std::string& lRaw : pNames)
			{
				const std::string lBase = CutUtf8(lRaw.empty() ? std::string("Marker") : lRaw, kMaxKeyLength);
				std::string lKey = lBase;
				for (int lNumber = 2; std::find(lKeys.begin(), lKeys.end(), lKey) != lKeys.end(); ++lNumber)
				{
					const std::string lSuffix = " " + std::to_string(lNumber);
					lKey = CutUtf8(lBase, kMaxKeyLength - lSuffix.size()) + lSuffix;
				}
				lKeys.push_back(lKey);
			}
			return lKeys;
		}

		// --- what the operator defines for a model

		struct Settings
		{
			std::string model;				//!< the name of the model when it was added: finds the row again when the list order changes
			std::string name;				//!< shown on the phone, the model name when empty
			bool visible{ true };
			int shape{ kShapeBox };
			int color{ 0 };
			double width{ 0.5 };			//!< metres. Along the marker's X; a sphere uses it as its diameter, a capsule as its diameter
			double depth{ 0.5 };			//!< metres. Along its Y (a box only)
			double height{ 1.0 };			//!< metres. Up from the base (a point uses none of the sizes, a sphere none but the width)
			int pivot{ kPivotBase };		//!< where the model origin is: at the base of the marker, or in its centre
			int anchor{ kAnchorCentre };	//!< the point focus and aim use
			int roles{ kRoleNone };			//!< Focus, Aim and Ref on the phone (bits of ERole, any combination), one marker per role
		};

		/// one marker per role: \p pRow keeps its roles, they are taken off any other row that has them
		inline void KeepOneRole(std::vector<Settings>& pRows, const size_t pRow)
		{
			if (pRow >= pRows.size() || pRows[pRow].roles == kRoleNone)
			{
				return;
			}
			for (size_t i = 0; i < pRows.size(); ++i)
			{
				if (i != pRow)
				{
					pRows[i].roles &= ~pRows[pRow].roles;
				}
			}
		}

		namespace detail
		{
			/// the one role a plugin of before reads: the one the phone kept before the combinations (Ref won)
			inline int LegacyRole(const int pRoles)
			{
				return (pRoles & kRoleRef) ? kLegacyRoleRef : (pRoles & kRoleFocus) ? kLegacyRoleFocus : (pRoles & kRoleAim) ? kLegacyRoleAim
					: kLegacyRoleNone;
			}

			inline int RolesOfLegacy(const int pRole)
			{
				return (pRole == kLegacyRoleFocus) ? kRoleFocus : (pRole == kLegacyRoleAim) ? kRoleAim : (pRole == kLegacyRoleRef) ? kRoleRef
					: kRoleNone;
			}
		}

		inline Settings DefaultSettings(const std::string& pModel, const int pIndex)
		{
			Settings lSettings;
			lSettings.model = pModel;
			lSettings.name = pModel;
			lSettings.color = ((pIndex % kColorCount) + kColorCount) % kColorCount;
			return lSettings;
		}

		namespace detail
		{
			inline std::string Escape(const std::string& pText)
			{
				std::string lOut;
				for (const char lChar : pText)
				{
					switch (lChar)
					{
					case '|':	lOut += "%7C";	break;
					case ';':	lOut += "%3B";	break;
					case '%':	lOut += "%25";	break;
					case '\n':	lOut += "%0A";	break;
					case '\r':	lOut += "%0D";	break;
					default:	lOut += lChar;	break;
					}
				}
				return lOut;
			}

			inline std::string Unescape(const std::string& pText)
			{
				std::string lOut;
				for (size_t i = 0; i < pText.size(); ++i)
				{
					if (pText[i] == '%' && i + 2 < pText.size())
					{
						const std::string lCode = pText.substr(i + 1, 2);
						if (lCode == "7C") { lOut += '|'; i += 2; continue; }
						if (lCode == "3B") { lOut += ';'; i += 2; continue; }
						if (lCode == "25") { lOut += '%'; i += 2; continue; }
						if (lCode == "0A") { lOut += '\n'; i += 2; continue; }
						if (lCode == "0D") { lOut += '\r'; i += 2; continue; }
					}
					lOut += pText[i];
				}
				return lOut;
			}

			inline std::vector<std::string> Split(const std::string& pText, const char pSeparator)
			{
				std::vector<std::string> lParts;
				size_t lStart = 0;
				for (;;)
				{
					const size_t lEnd = pText.find(pSeparator, lStart);
					if (lEnd == std::string::npos)
					{
						lParts.push_back(pText.substr(lStart));
						return lParts;
					}
					lParts.push_back(pText.substr(lStart, lEnd - lStart));
					lStart = lEnd + 1;
				}
			}

			inline int ToInt(const std::string& pText, const int pDefault, const int pMin, const int pMax)
			{
				if (pText.empty())
				{
					return pDefault;
				}
				char* lEnd = nullptr;
				const long lValue = std::strtol(pText.c_str(), &lEnd, 10);
				if (lEnd == pText.c_str() || *lEnd != '\0')
				{
					return pDefault;
				}
				return static_cast<int>(std::min(std::max(lValue, static_cast<long>(pMin)), static_cast<long>(pMax)));
			}

			inline double ToDouble(const std::string& pText, const double pDefault)
			{
				if (pText.empty())
				{
					return pDefault;
				}
				char* lEnd = nullptr;
				const double lValue = std::strtod(pText.c_str(), &lEnd);
				if (lEnd == pText.c_str() || *lEnd != '\0' || !std::isfinite(lValue))
				{
					return pDefault;
				}
				return lValue;
			}
		}

		constexpr const char* kSettingsHeader = "OSCMARKERS1";

		/// the text form that is saved with the device in the scene: a header, then a row per marker, separated by semicolons (no line breaks
		/// in a property string): model|name|visible|shape|color|width|depth|height|pivot|anchor|role|roles. `role` is the one role of the
		/// plugins of before (0 none, 1 Focus, 2 Aim, 3 Ref; Ref when the marker has it), `roles` the bits of ERole. A row saved before the
		/// combinations has no `roles` field and reads its `role`, a row saved before the roles has neither and reads as none.
		inline std::string SerializeSettings(const std::vector<Settings>& pSettings)
		{
			using namespace detail;
			std::string lText = kSettingsHeader;
			char lNumber[48];
			for (const Settings& lRow : pSettings)
			{
				lText += ';';
				lText += Escape(lRow.model) + '|' + Escape(lRow.name) + '|' + (lRow.visible ? '1' : '0') + '|' + std::to_string(lRow.shape)
					+ '|' + std::to_string(lRow.color);
				for (const double lSize : { lRow.width, lRow.depth, lRow.height })
				{
					std::snprintf(lNumber, sizeof(lNumber), "|%.4f", lSize);
					lText += lNumber;
				}
				lText += '|' + std::to_string(lRow.pivot) + '|' + std::to_string(lRow.anchor) + '|' + std::to_string(LegacyRole(lRow.roles))
					+ '|' + std::to_string(lRow.roles & kRoleAll);
			}
			return lText;
		}

		/// the rows of a saved text, in the order they were saved. Rows that make no sense are skipped, never an error.
		inline std::vector<Settings> ParseSettings(const std::string& pText)
		{
			using namespace detail;
			std::vector<Settings> lRows;
			const std::vector<std::string> lLines = Split(pText, ';');
			if (lLines.empty() || lLines[0] != kSettingsHeader)
			{
				return lRows;
			}

			for (size_t i = 1; i < lLines.size(); ++i)
			{
				const std::vector<std::string> lFields = Split(lLines[i], '|');
				if (lFields.size() < 10)
				{
					continue;
				}

				Settings lRow;
				lRow.model = Unescape(lFields[0]);
				lRow.name = Unescape(lFields[1]);
				lRow.visible = (lFields[2] != "0");
				lRow.shape = ToInt(lFields[3], kShapeBox, 0, kShapeCount - 1);
				lRow.color = ToInt(lFields[4], 0, 0, kColorCount - 1);
				lRow.width = ToDouble(lFields[5], 0.5);
				lRow.depth = ToDouble(lFields[6], 0.5);
				lRow.height = ToDouble(lFields[7], 1.0);
				lRow.pivot = ToInt(lFields[8], kPivotBase, 0, kPivotCount - 1);
				lRow.anchor = ToInt(lFields[9], kAnchorCentre, 0, kAnchorCount - 1);
				if (lFields.size() > 11)
				{
					lRow.roles = ToInt(lFields[11], kRoleNone, kRoleNone, kRoleAll);
				}
				else if (lFields.size() > 10)
				{
					lRow.roles = RolesOfLegacy(ToInt(lFields[10], kLegacyRoleNone, 0, kLegacyRoleCount - 1));
				}
				lRows.push_back(lRow);
			}

			// a text edited by hand or a script may give a role twice: the first row keeps it
			for (size_t i = 0; i < lRows.size(); ++i)
			{
				KeepOneRole(lRows, i);
			}
			return lRows;
		}

		/// The settings of the models of the list, in the order of the list. A saved row is taken for the model at its place when it was
		/// saved for the same model, else it is looked for by the model name (the list was reordered by hand or a script), else the model
		/// gets the defaults. A model never gets the row of another model.
		inline std::vector<Settings> Reconcile(const std::vector<Settings>& pSaved, const std::vector<std::string>& pModelNames)
		{
			std::vector<Settings> lResult;
			std::vector<bool> lUsed(pSaved.size(), false);

			for (size_t i = 0; i < pModelNames.size(); ++i)
			{
				int lFound = -1;
				if (i < pSaved.size() && !lUsed[i] && pSaved[i].model == pModelNames[i])
				{
					lFound = static_cast<int>(i);
				}
				else
				{
					for (size_t k = 0; k < pSaved.size(); ++k)
					{
						if (!lUsed[k] && pSaved[k].model == pModelNames[i])
						{
							lFound = static_cast<int>(k);
							break;
						}
					}
				}

				if (lFound >= 0)
				{
					lUsed[static_cast<size_t>(lFound)] = true;
					lResult.push_back(pSaved[static_cast<size_t>(lFound)]);
				}
				else
				{
					lResult.push_back(DefaultSettings(pModelNames[i], static_cast<int>(i)));
				}
			}
			return lResult;
		}

		// --- placement

		/// A model of the scene as the phone wants it.
		/// \param pPositionCm	the origin of the model in the frame of the device Reference model (MotionBuilder axes: X right, Y up, Z back), centimetres
		/// \param pXAxis		the local X axis of the model in the same frame
		/// The camera of the device is placed in that frame by its stream pose, so the phone sees the marker where the camera it drives would.
		inline Position Place(const double pPositionCm[3], const double pXAxis[3], const Settings& pSettings, const ConvertSettings& pConvert)
		{
			// the vertical offset first: the position sent is the middle of the base of the marker
			double lPosition[3] = { pPositionCm[0], pPositionCm[1], pPositionCm[2] };
			if (pSettings.pivot == kPivotCentre && pSettings.shape != kShapePoint)
			{
				lPosition[1] -= 0.5 * pSettings.height * 100.0;
			}

			// the MotionBuilder scene (right, up, back) to the wire (right, forward, up), the inverse of the camera position
			double lWire[3], lDirection[3];
			OSCCamera::detail::SceneToWire(lPosition, lWire);
			OSCCamera::detail::SceneToWire(pXAxis, lDirection);

			Position lResult;
			double* lOut[3] = { &lResult.x, &lResult.y, &lResult.z };
			const double lToScene = pConvert.CentimetresPerWireUnit();	// the same factor as the camera, the other way round
			for (int i = 0; i < 3; ++i)
			{
				*lOut[i] = (lToScene != 0.0) ? lWire[i] / lToScene : 0.0;
			}

			// the heading is a direction: it is not scaled
			const double lDx = lDirection[0];
			const double lDy = lDirection[1];
			lResult.yaw = (std::fabs(lDx) < 1e-9 && std::fabs(lDy) < 1e-9) ? 0.0 : std::atan2(lDy, lDx) * 180.0 / kPi;
			return lResult;
		}

		// --- what to send

		/// One marker as the sender sees it in this step.
		struct State
		{
			std::string key;
			std::string name;
			int shape{ kShapeBox };
			int color{ 0 };
			double width{ 0.5 };
			double depth{ 0.5 };
			double height{ 1.0 };
			int anchor{ kAnchorCentre };
			bool visible{ true };
			int roles{ kRoleNone };		//!< bits of ERole
			Position position;
		};

		/// Decides what to send. Step is called often; it appends the datagrams to write now (often none):
		///  - the definitions and the key list as soon as they change (a size at most 10 times a second) and in any case once a second,
		///    so a phone that missed a packet or joined late catches up;
		///  - the positions when a marker moved or the scene time changed (at most 60 times a second), and at least twice a second even
		///    when nothing moves (the phone calls a link quiet after three seconds);
		///  - the roles (RoleSync::Message), after the key list so a role never names a marker the phone has not heard of yet: with the
		///    definitions and as soon as they change.
		class Sender
		{
		public:
			static constexpr int64_t kSecond = 1000000000LL;
			static constexpr int64_t kMinGap = kSecond / 60;
			static constexpr int64_t kDefinitionPeriod = kSecond;
			static constexpr int64_t kHeartbeatPeriod = kSecond / 2;
			static constexpr int64_t kDefinitionChangeGap = kSecond / 10;
			static constexpr double kSizeStep = 0.01;
			static constexpr double kPositionEpsilon = 1e-4;

			Sender()
			{
				Reset();
			}

			void Reset()
			{
				mSeq = 0;
				mSentDefinitions.clear();
				mDefinitionsAt = INT64_MIN / 2;
				mLastStep = INT64_MIN / 2;
				mPositionsAt = INT64_MIN / 2;
				mSentPositions.clear();
				mHasSentPositions = false;
				mSentTime = 0;
				mSentRoles.clear();
			}

			int32_t Seq() const { return mSeq; }

			/// false while it is too soon for another step, so the caller can skip the work of measuring
			bool Due(const int64_t pNow) const
			{
				return pNow - mLastStep >= kMinGap;
			}

			/// \param pRoles	the roles message to keep the phone on, nullptr - the sender gives no roles (the operator picks them on the phone)
			void Step(const int64_t pNow, const std::vector<State>& pMarkersIn, const int64_t pSceneTimeUs, std::vector<Datagram>& pOut,
				const Datagram* pRoles = nullptr)
			{
				mLastStep = pNow;

				std::vector<State> lMarkers = pMarkersIn;
				if (lMarkers.size() > static_cast<size_t>(kMaxMarkers))
				{
					lMarkers.resize(kMaxMarkers);
				}

				bool lSendDefinitions = false;
				if (pNow - mDefinitionsAt >= kDefinitionPeriod)
				{
					lSendDefinitions = true;
				}
				else if (DefinitionsChanged(lMarkers) && pNow - mDefinitionsAt >= kDefinitionChangeGap)
				{
					lSendDefinitions = true;
				}

				if (lSendDefinitions)
				{
					std::vector<std::string> lKeys;
					for (const State& lMarker : lMarkers)
					{
						pOut.push_back(EncodeDef(lMarker.key, lMarker.name, lMarker.shape, lMarker.color, lMarker.width, lMarker.depth,
							lMarker.height, lMarker.anchor, lMarker.visible));
						lKeys.push_back(lMarker.key);
					}
					pOut.push_back(EncodeList(lKeys));
					mSentDefinitions = lMarkers;
					mDefinitionsAt = pNow;
				}

				if (pRoles != nullptr && (lSendDefinitions || *pRoles != mSentRoles))
				{
					pOut.push_back(*pRoles);
					mSentRoles = *pRoles;
				}

				std::vector<Position> lPositions;
				for (const State& lMarker : lMarkers)
				{
					lPositions.push_back(lMarker.position);
				}

				bool lMoved = !mHasSentPositions || lPositions.size() != mSentPositions.size();
				for (size_t i = 0; !lMoved && i < lPositions.size(); ++i)
				{
					const Position& a = lPositions[i];
					const Position& b = mSentPositions[i];
					lMoved = std::fabs(a.x - b.x) > kPositionEpsilon || std::fabs(a.y - b.y) > kPositionEpsilon
						|| std::fabs(a.z - b.z) > kPositionEpsilon || std::fabs(a.yaw - b.yaw) > kPositionEpsilon;
				}

				if (lMoved || pSceneTimeUs != mSentTime || pNow - mPositionsAt >= kHeartbeatPeriod)
				{
					pOut.push_back(EncodePositions(pSceneTimeUs, mSeq, lPositions));
					++mSeq;	// a signed 32 bit wrap is fine
					mPositionsAt = pNow;
					mSentPositions = lPositions;
					mHasSentPositions = true;
					mSentTime = pSceneTimeUs;
				}
			}

		private:
			bool DefinitionsChanged(const std::vector<State>& pMarkers) const
			{
				if (pMarkers.size() != mSentDefinitions.size())
				{
					return true;
				}
				for (size_t i = 0; i < pMarkers.size(); ++i)
				{
					const State& a = pMarkers[i];
					const State& b = mSentDefinitions[i];
					if (a.key != b.key || a.name != b.name || a.shape != b.shape || a.color != b.color || a.anchor != b.anchor
						|| a.visible != b.visible || std::fabs(a.width - b.width) > kSizeStep || std::fabs(a.depth - b.depth) > kSizeStep
						|| std::fabs(a.height - b.height) > kSizeStep)
					{
						return true;
					}
				}
				return false;
			}

			int32_t					mSeq{ 0 };
			std::vector<State>		mSentDefinitions;
			int64_t					mDefinitionsAt{ 0 };
			int64_t					mLastStep{ 0 };
			int64_t					mPositionsAt{ 0 };
			std::vector<Position>	mSentPositions;
			bool					mHasSentPositions{ false };
			int64_t					mSentTime{ 0 };
			Datagram				mSentRoles;
		};

		// --- the roles (section 10.10)

		//! the slots of the roles, in the order of the bits of the phone's "overridden"
		enum ERoleSlot { kSlotFocus = 0, kSlotAim, kSlotRef, kSlotFollow, kSlotLookAt, kSlotCount };

		struct Roles
		{
			std::string	focus;		//!< the marker keys, "" for none
			std::string	aim;
			std::string	ref;
			int			refFollow{ kRefFollowPosition };
			int			lookAt{ kLookAtPhone };	//!< 0 to 100, -1 - the phone's

			bool operator==(const Roles& pOther) const
			{
				return focus == pOther.focus && aim == pOther.aim && ref == pOther.ref && refFollow == pOther.refFollow && lookAt == pOther.lookAt;
			}
			bool operator!=(const Roles& pOther) const { return !(*this == pOther); }

			/// the slots that differ, as bits of ERoleSlot
			int Differs(const Roles& pOther) const
			{
				return (focus != pOther.focus ? 1 << kSlotFocus : 0) | (aim != pOther.aim ? 1 << kSlotAim : 0) | (ref != pOther.ref ? 1 << kSlotRef : 0)
					| (refFollow != pOther.refFollow ? 1 << kSlotFollow : 0) | (lookAt != pOther.lookAt ? 1 << kSlotLookAt : 0);
			}

			/// take the slots of \p pMask from \p pFrom
			void Take(const Roles& pFrom, const int pMask)
			{
				if (pMask & (1 << kSlotFocus)) { focus = pFrom.focus; }
				if (pMask & (1 << kSlotAim)) { aim = pFrom.aim; }
				if (pMask & (1 << kSlotRef)) { ref = pFrom.ref; }
				if (pMask & (1 << kSlotFollow)) { refFollow = pFrom.refFollow; }
				if (pMask & (1 << kSlotLookAt)) { lookAt = pFrom.lookAt; }
			}
		};

		//! the phone's /livetracker/link/roles: the rev it follows, the roles in force on its Live scene, the slots the operator set there
		struct RolesReport
		{
			int32_t	rev{ 0 };
			Roles	roles;
			int		overridden{ 0 };	//!< bits of ERoleSlot

			bool IsOverridden(const int pSlot) const { return (overridden & (1 << pSlot)) != 0; }
		};

		/// the bit of ERole of a slot of the roles (Focus, Aim, Ref), kRoleNone for the others
		inline int RoleOfSlot(const int pSlot)
		{
			return (pSlot >= kSlotFocus && pSlot <= kSlotRef) ? (1 << pSlot) : kRoleNone;
		}

		/// the roles of the markers that are sent: the key of the first marker of each role. A marker with two or three roles is named in
		/// each of their slots.
		inline Roles RolesOf(const std::vector<State>& pMarkers, const int pRefFollow, const int pLookAt)
		{
			Roles lRoles;
			std::string* const lSlots[] = { &lRoles.focus, &lRoles.aim, &lRoles.ref };
			for (const State& lMarker : pMarkers)
			{
				for (int lSlot = kSlotFocus; lSlot <= kSlotRef; ++lSlot)
				{
					if ((lMarker.roles & RoleOfSlot(lSlot)) && lSlots[lSlot]->empty())
					{
						*lSlots[lSlot] = lMarker.key;
					}
				}
			}
			lRoles.refFollow = (pRefFollow == kRefFollowHeading) ? kRefFollowHeading : kRefFollowPosition;
			lRoles.lookAt = std::min(std::max(pLookAt, kLookAtPhone), 100);
			return lRoles;
		}

		/// Writes the Focus, Aim and Ref slots of \p pMask from the phone into the rows, every slot as it is: the row of the slot's key gets
		/// the role beside the ones it has, and the role is taken off the other rows. A slot of "" clears the role; a key that is no row's
		/// leaves the rows as they are.
		/// \param pKeys	the key each row is sent with, "" for a row that is not sent
		inline void MirrorRoles(std::vector<Settings>& pRows, const std::vector<std::string>& pKeys, const Roles& pRoles, const int pMask)
		{
			const std::string* const lKeys[] = { &pRoles.focus, &pRoles.aim, &pRoles.ref };
			for (int lSlot = kSlotFocus; lSlot <= kSlotRef; ++lSlot)
			{
				if (!(pMask & (1 << lSlot)))
				{
					continue;
				}
				const int lRole = RoleOfSlot(lSlot);
				const std::string& lKey = *lKeys[lSlot];

				size_t lFound = pRows.size();
				for (size_t i = 0; !lKey.empty() && i < pRows.size() && i < pKeys.size(); ++i)
				{
					if (pKeys[i] == lKey)
					{
						lFound = i;
						break;
					}
				}
				if (!lKey.empty() && lFound == pRows.size())
				{
					continue;
				}
				for (size_t i = 0; i < pRows.size(); ++i)
				{
					pRows[i].roles = (i == lFound) ? (pRows[i].roles | lRole) : (pRows[i].roles & ~lRole);
				}
			}
		}

		/**	Keeps the roles of the device and the phone's in step, the same rules as rolesync.RoleSync of the Blender add-on.
		*	Call Local with the device's roles every step, Message for the datagram to send, Report with each new report of the phone,
		*	and Mirrored once the changes it returned are written into the device.
		*
		*	Two guards keep a stale report from undoing a change made on the device a moment ago:
		*	 - per role: a role just changed on the device is not mirrored until the phone reports it as no longer the operator's (it has
		*	   taken the change), or for at most kPendingPeriod;
		*	 - rev: it goes up only on SendAgain ("Send roles to phone": the phone gives every role back), a report with another rev is not
		*	   mirrored.
		*/
		class RoleSync
		{
		public:
			static constexpr int64_t kPendingPeriod = 3000000000LL;	//!< ns

			int32_t Rev() const { return mRev; }
			const Roles& Current() const { return mLocal; }

			/// the device's roles now. A slot that changed since the last call (and not by Mirrored) is the artist's change.
			void Local(const Roles& pRoles, const int64_t pNow)
			{
				if (mHasLocal)
				{
					const int lChanged = pRoles.Differs(mLocal);
					for (int lSlot = 0; lSlot < kSlotCount; ++lSlot)
					{
						if (lChanged & (1 << lSlot))
						{
							mPendingSince[lSlot] = pNow;
							mPending[lSlot] = true;
						}
					}
				}
				mLocal = pRoles;
				mHasLocal = true;
			}

			/// "Send roles to phone": a new rev, the phone gives every role back to the device
			void SendAgain(const int64_t pNow)
			{
				++mRev;
				for (int lSlot = 0; lSlot < kSlotCount; ++lSlot)
				{
					mPendingSince[lSlot] = pNow;
					mPending[lSlot] = true;
				}
			}

			Datagram Message() const
			{
				return EncodeRoles(mRev, mLocal.focus, mLocal.aim, mLocal.ref, mLocal.refFollow, mLocal.lookAt);
			}

			bool Pending(const int pSlot, const int64_t pNow) const
			{
				return mPending[pSlot] && pNow - mPendingSince[pSlot] < kPendingPeriod;
			}

			/// Takes a report in. \return the slots (bits of ERoleSlot) to write into the device from pReport.roles: the ones the operator set
			/// on the phone that the device has not just changed and that differ from the device's; none unless \p pMirror.
			int Report(const RolesReport& pReport, const bool pMirror, const int64_t pNow)
			{
				if (pReport.rev != mRev)
				{
					return 0;	// the phone has not taken the latest request yet
				}
				for (int lSlot = 0; lSlot < kSlotCount; ++lSlot)
				{
					if (!pReport.IsOverridden(lSlot))
					{
						mPending[lSlot] = false;	// the phone follows the device's value here: nothing waits any more
					}
				}
				if (!pMirror)
				{
					return 0;
				}

				const int lDiffers = pReport.roles.Differs(mLocal);
				int lOut = 0;
				for (int lSlot = 0; lSlot < kSlotCount; ++lSlot)
				{
					if (!pReport.IsOverridden(lSlot) || Pending(lSlot, pNow) || !(lDiffers & (1 << lSlot)))
					{
						continue;
					}
					if (lSlot == kSlotLookAt && mLocal.lookAt < 0)
					{
						continue;	// the device leaves the strength to the phone: nothing to mirror
					}
					lOut |= 1 << lSlot;
				}
				return lOut;
			}

			/// the slots Report returned are now the device's: they are taken as the local roles, not counted as the artist's change
			void Mirrored(const Roles& pFrom, const int pMask)
			{
				mLocal.Take(pFrom, pMask);
			}

			/// for the operator: the slots where the phone has something other than the device (the operator's choices)
			int Differs(const RolesReport& pReport) const
			{
				return pReport.roles.Differs(mLocal) & pReport.overridden;
			}

		private:
			int32_t	mRev{ 1 };
			Roles	mLocal;
			bool	mHasLocal{ false };
			bool	mPending[kSlotCount]{};
			int64_t	mPendingSince[kSlotCount]{};
		};

		/// What to tell the operator about the roles on the phone: "" when the phone has the device's, else a line per role the operator set
		/// on the phone, as the Blender add-on shows them ("Phone: Ref = Cart (set on phone)").
		inline std::string RolesStatusText(const RolesReport& pReport, const int pDiffers)
		{
			static const char* kNames[kSlotCount] = { "Focus", "Aim", "Ref", "Ref follow", "Look-at" };
			std::string lText;
			for (int lSlot = 0; lSlot < kSlotCount; ++lSlot)
			{
				if (!(pDiffers & (1 << lSlot)))
				{
					continue;
				}
				std::string lValue;
				switch (lSlot)
				{
				case kSlotFocus:	lValue = pReport.roles.focus.empty() ? "none" : pReport.roles.focus;	break;
				case kSlotAim:		lValue = pReport.roles.aim.empty() ? "none" : pReport.roles.aim;		break;
				case kSlotRef:		lValue = pReport.roles.ref.empty() ? "none" : pReport.roles.ref;		break;
				case kSlotFollow:	lValue = (pReport.roles.refFollow == kRefFollowHeading) ? "Position + heading" : "Position";	break;
				default:			lValue = std::to_string(pReport.roles.lookAt) + " %";	break;
				}
				lText += (lText.empty() ? "" : ", ") + std::string(kNames[lSlot]) + " = " + lValue;
			}
			return lText.empty() ? lText : "Phone: " + lText + " (set on phone)";
		}

		// --- what the phone says back

		/// What to tell the operator about the phone's end of the link (the last /livetracker/link, nothing when none was heard in the last 5 s).
		/// \param pHeard			a /livetracker/link was heard lately
		/// \param pSecondsLinked	how long we have been sending
		inline std::string LinkStatusText(const bool pHeard, const int pState, const int pMarkers, const double pSecondsLinked)
		{
			if (!pHeard)
			{
				return (pSecondsLinked < 3.0) ? "Waiting for the phone to answer" : "The phone does not answer: is the stream running?";
			}
			if (pState == 0)
			{
				return "The phone's marker link is off: turn on Settings > Stream > Scene link";
			}
			if (pState == 1)
			{
				return "The phone is listening: allow this computer on the phone";
			}
			return "Phone linked, " + std::to_string(pMarkers) + (pMarkers == 1 ? " marker" : " markers");
		}
	}
}
