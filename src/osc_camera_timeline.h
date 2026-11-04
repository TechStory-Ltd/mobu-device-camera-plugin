#pragma once

/**	\file	osc_camera_timeline.h
*	Copyright (c) 2026 TechStory Ltd
*	MIT License, see the LICENSE file in the repository root
*
* The poses on the timeline of the phone instead of the order and the time they arrive in (OSC_PROTOCOL.md section 5.1).
* Header-only, no SDK dependency, so it can be unit tested standalone.
*
*  - the poses are kept sorted by their capture time, so a late packet is still used and a repeat of the same sensor sample
*    (the app sends up to 120 poses per second, a sensor makes 30-60) is dropped;
*  - the offset between the phone's clock and ours is the smallest (arrival - capture) of the last few seconds: the best case
*    network delay, the late packets are ignored. It follows a faster path down at once and a slower one up gradually, so a
*    change of the delay does not make the camera lurch;
*  - a pose is rendered by interpolating the samples around (now - offset - delay), slerp for the rotation. Past the newest
*    sample it is extrapolated for a moment, then held;
*  - Playback chooses the delay for every render: a fixed one, one that follows the jitter of the network (the smallest that
*    still lets the render land between two samples), or none at all (the newest sample as received).
*
* All times are nanoseconds. The arrival time must come from a high resolution monotonic clock.
*/

#include "osc_camera_math.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <vector>

namespace OSCCamera
{
	namespace detail
	{
		/// linear interpolation between the closest ranks of an ascending, not empty vector
		inline double Percentile(const std::vector<double>& pSorted, const double pQuantile)
		{
			const double lRank = std::min(std::max(pQuantile, 0.0), 1.0) * static_cast<double>(pSorted.size() - 1);
			const size_t lLow = static_cast<size_t>(lRank);
			const size_t lHigh = std::min(lLow + 1, pSorted.size() - 1);
			return pSorted[lLow] + (pSorted[lHigh] - pSorted[lLow]) * (lRank - static_cast<double>(lLow));
		}
	}

	class Timeline
	{
	public:
		enum class AddResult : uint8_t
		{
			New,		//!< a sample that was not on the timeline
			Repeat,		//!< the same sensor sample again (equal capture time)
			Stale,		//!< older than everything the timeline keeps
			Reset,		//!< the phone's clock restarted (a reboot): everything before was dropped, the sample is on the new timeline
			Unstamped	//!< captureNanos is 0 - no sensor sample behind the pose, it is held but not put on the timeline
		};

		enum class RenderState : uint8_t
		{
			None,			//!< nothing has arrived yet
			Interpolated,
			Extrapolated,
			Held
		};

		struct AddInfo
		{
			AddResult result{ AddResult::New };
			bool newest{ false };	//!< the sample is the newest on the timeline: the one to record
			int64_t captureNanos{ 0 };
		};

		struct RenderInfo
		{
			RenderState state{ RenderState::None };
			BlendedPose pose;
		};

		static constexpr size_t kMaxSamples = 512;
		static constexpr int64_t kOffsetWindow = 5000000000LL;		//!< the minimum of (arrival - capture) is taken over this time
		static constexpr int64_t kSlewUpPerSecond = 2000000LL;		//!< the offset may rise this much per second
		static constexpr int64_t kRebootJump = 10000000000LL;		//!< a capture time this much older than the newest is a new phone session
		static constexpr int64_t kExtrapolateLimit = 50000000LL;	//!< how long past the newest sample the pose is extrapolated
		static constexpr int64_t kMinSampleSpacing = 1000000LL;		//!< two samples closer than this say nothing about the velocity
		static constexpr size_t kSpacingSamples = 9;				//!< the sample spacing is the median over the last few samples
		static constexpr int64_t kDefaultSpacing = 16666667LL;		//!< the spacing before there are enough samples

		Timeline()
		{
			Reset();
		}

		void Reset()
		{
			mCaptures.clear();
			mPoses.clear();
			mDiffs.clear();
			mHasOffset = false;
			mOffset = 0.0;
			mOffsetAt = 0;
			mHasUnstamped = false;
		}

		size_t SampleCount() const { return mCaptures.size(); }
		bool HasPose() const { return !mCaptures.empty() || mHasUnstamped; }
		bool HasOffset() const { return mHasOffset; }

		/// the phone's capture time as a time on our clock (capture + offset), the earliest it could have arrived
		int64_t ToLocalTime(const int64_t pCapture) const
		{
			return pCapture + static_cast<int64_t>(mOffset);
		}

		/// \return the largest minus the smallest (arrival - capture) of the window, milliseconds: the jitter of the network as seen here
		double JitterMilliseconds() const
		{
			if (mDiffs.empty())
			{
				return 0.0;
			}
			int64_t lMin = mDiffs.front().diff;
			int64_t lMax = lMin;
			for (const Diff& lDiff : mDiffs)
			{
				lMin = std::min(lMin, lDiff.diff);
				lMax = std::max(lMax, lDiff.diff);
			}
			return static_cast<double>(lMax - lMin) * 1e-6;
		}

		/// \return the 99th minus the 1st percentile of (arrival - capture) of the window, milliseconds: the jitter without the one late packet
		double JitterSpreadMilliseconds() const
		{
			if (mDiffs.empty())
			{
				return 0.0;
			}
			std::vector<double> lSorted;
			SortedDiffs(lSorted, 0.0);
			return (detail::Percentile(lSorted, 0.99) - detail::Percentile(lSorted, 0.01)) * 1e-6;
		}

		/// \return the pQuantile (0..1) of the excess delay of the window, nanoseconds: how much later than the best case the packets arrive.
		/// 0 until a sample has arrived.
		int64_t ExcessQuantile(const double pQuantile) const
		{
			if (mDiffs.empty() || !mHasOffset)
			{
				return 0;
			}
			std::vector<double> lSorted;
			SortedDiffs(lSorted, mOffset);
			return static_cast<int64_t>(std::max(detail::Percentile(lSorted, pQuantile), 0.0));
		}

		/// \return the time between two samples, nanoseconds: the median of the last few spacings (the sensor rate, not the packet rate)
		int64_t SampleSpacing() const
		{
			const size_t lCount = std::min(mCaptures.size(), kSpacingSamples);
			if (lCount < 3)
			{
				return kDefaultSpacing;
			}
			std::array<int64_t, kSpacingSamples> lSpacings;
			for (size_t i = 0; i + 1 < lCount; ++i)
			{
				lSpacings[i] = mCaptures[mCaptures.size() - lCount + i + 1] - mCaptures[mCaptures.size() - lCount + i];
			}
			const size_t lN = lCount - 1;
			std::nth_element(lSpacings.begin(), lSpacings.begin() + static_cast<std::ptrdiff_t>(lN / 2), lSpacings.begin() + static_cast<std::ptrdiff_t>(lN));
			return std::max(lSpacings[lN / 2], kMinSampleSpacing);
		}

		AddInfo Add(const PoseMessage& pPose, const int64_t pArrival)
		{
			AddInfo lInfo;
			lInfo.captureNanos = pPose.captureNanos;

			const int64_t lCapture = pPose.captureNanos;
			if (lCapture == 0)
			{
				mUnstamped = pPose;
				mHasUnstamped = true;
				lInfo.result = AddResult::Unstamped;
				return lInfo;
			}

			if (!mCaptures.empty() && lCapture < mCaptures.back() - kRebootJump)
			{
				Reset();
				lInfo.result = AddResult::Reset;
			}

			// a repeat of a sample is sent later than the sample itself: it says nothing about the network
			const auto lAt = std::lower_bound(mCaptures.begin(), mCaptures.end(), lCapture);
			if (lAt != mCaptures.end() && *lAt == lCapture)
			{
				lInfo.result = AddResult::Repeat;
				return lInfo;
			}

			// the clock offset: the smallest (arrival - capture) over the window
			mDiffs.push_back(Diff{ pArrival, pArrival - lCapture });
			while (!mDiffs.empty() && mDiffs.front().arrival < pArrival - kOffsetWindow)
			{
				mDiffs.pop_front();
			}
			int64_t lTarget = mDiffs.front().diff;
			for (const Diff& lDiff : mDiffs)
			{
				lTarget = std::min(lTarget, lDiff.diff);
			}

			if (!mHasOffset || static_cast<double>(lTarget) <= mOffset)
			{
				mOffset = static_cast<double>(lTarget);
				mHasOffset = true;
			}
			else
			{
				const double lMaxUp = static_cast<double>(kSlewUpPerSecond) * static_cast<double>(pArrival - mOffsetAt) * 1e-9;
				mOffset = std::min(static_cast<double>(lTarget), mOffset + lMaxUp);
			}
			mOffsetAt = pArrival;

			const size_t lIndex = static_cast<size_t>(lAt - mCaptures.begin());
			if (lIndex == 0 && mCaptures.size() >= kMaxSamples)
			{
				lInfo.result = AddResult::Stale;
				return lInfo;
			}

			mCaptures.insert(lAt, lCapture);
			mPoses.insert(mPoses.begin() + static_cast<std::ptrdiff_t>(lIndex), pPose);
			if (mCaptures.size() > kMaxSamples)
			{
				mCaptures.erase(mCaptures.begin());
				mPoses.erase(mPoses.begin());
			}

			lInfo.newest = (lCapture == mCaptures.back());
			return lInfo;
		}

		/// the camera at (pNow - delay) on the phone's timeline
		RenderInfo Render(const int64_t pNow, const int64_t pDelay, const ConvertSettings& pSettings) const
		{
			RenderInfo lInfo;

			if (mCaptures.empty() || !mHasOffset)
			{
				if (mHasUnstamped)
				{
					lInfo.pose = ToBlended(mUnstamped);
					lInfo.state = RenderState::Held;
				}
				return lInfo;
			}

			const int64_t lTime = pNow - static_cast<int64_t>(mOffset) - pDelay;	// on the phone's capture clock
			const auto lAfter = std::upper_bound(mCaptures.begin(), mCaptures.end(), lTime);
			const size_t lIndex = static_cast<size_t>(lAfter - mCaptures.begin());

			if (lIndex == 0)
			{
				lInfo.pose = ToBlended(mPoses.front());
				lInfo.state = RenderState::Held;
				return lInfo;
			}

			if (lIndex == mCaptures.size())
			{
				const int64_t lOver = lTime - mCaptures.back();
				if (mCaptures.size() >= 2 && lOver > 0 && lOver <= kExtrapolateLimit && CanBlend(mPoses[lIndex - 2], mPoses[lIndex - 1]))
				{
					const int64_t lSpacing = mCaptures[lIndex - 1] - mCaptures[lIndex - 2];
					if (lSpacing >= kMinSampleSpacing)
					{
						lInfo.pose = Blend(ToBlended(mPoses[lIndex - 2]), ToBlended(mPoses[lIndex - 1]),
							1.0 + static_cast<double>(lOver) / static_cast<double>(lSpacing));
						lInfo.state = RenderState::Extrapolated;
						return lInfo;
					}
				}
				lInfo.pose = ToBlended(mPoses.back());
				lInfo.state = RenderState::Held;
				return lInfo;
			}

			// a local pose is blended only with one relative to the same Ref (section 2.8): across a change of the Ref, of its follow or of
			// the mode the camera stays at the earlier sample until the later one is due
			if (!CanBlend(mPoses[lIndex - 1], mPoses[lIndex]))
			{
				lInfo.pose = ToBlended(mPoses[lIndex - 1]);
				lInfo.state = RenderState::Held;
				return lInfo;
			}

			const int64_t lFrom = mCaptures[lIndex - 1];
			const int64_t lTo = mCaptures[lIndex];
			const double lFraction = static_cast<double>(lTime - lFrom) / static_cast<double>(lTo - lFrom);
			lInfo.pose = Blend(ToBlended(mPoses[lIndex - 1]), ToBlended(mPoses[lIndex]), lFraction);
			lInfo.state = RenderState::Interpolated;
			return lInfo;
		}

		/// the newest sample as received: no delay, no interpolation, no extrapolation
		RenderInfo RenderNewest(const ConvertSettings& pSettings) const
		{
			RenderInfo lInfo;
			if (!mPoses.empty())
			{
				lInfo.pose = ToBlended(mPoses.back());
				lInfo.state = RenderState::Held;
			}
			else if (mHasUnstamped)
			{
				lInfo.pose = ToBlended(mUnstamped);
				lInfo.state = RenderState::Held;
			}
			return lInfo;
		}

		/// the newest sample as received (no interpolation), e.g. to show the raw values
		bool GetNewest(PoseMessage& pPose) const
		{
			if (!mPoses.empty())
			{
				pPose = mPoses.back();
				return true;
			}
			if (mHasUnstamped)
			{
				pPose = mUnstamped;
				return true;
			}
			return false;
		}

	private:
		/// (arrival - capture) of the window minus \p pOffset, ascending
		void SortedDiffs(std::vector<double>& pOut, const double pOffset) const
		{
			pOut.reserve(mDiffs.size());
			for (const Diff& lDiff : mDiffs)
			{
				pOut.push_back(static_cast<double>(lDiff.diff) - pOffset);
			}
			std::sort(pOut.begin(), pOut.end());
		}

		struct Diff
		{
			int64_t arrival;
			int64_t diff;	//!< arrival - capture
		};

		std::vector<int64_t>	mCaptures;		//!< ascending
		std::vector<PoseMessage> mPoses;		//!< the matching poses
		std::deque<Diff>		mDiffs;
		bool					mHasOffset{ false };
		double					mOffset{ 0.0 };
		int64_t					mOffsetAt{ 0 };
		PoseMessage				mUnstamped;
		bool					mHasUnstamped{ false };
	};

	/// Which datagrams of /livetracker/pose/ref are used (OSC_PROTOCOL.md section 2.8): one stream session at a time, and in it only the
	/// datagrams newer than the last one by the output sequence (int32 wrap-aware). The phone repeats a sensor sample in several datagrams;
	/// those are told apart by the timeline, not here. A /livetracker/pose has no session and is always accepted.
	class StreamFilter
	{
	public:
		enum class Result : uint8_t
		{
			Accepted,
			NewSession,	//!< the first datagram of another session: what the timeline holds is of a stream that is over
			Stale,		//!< a duplicate or an older output sequence of the session
			Retired		//!< a late datagram of a session that was replaced a moment ago
		};

		static constexpr int64_t kRetiredMemory = 2000000000LL;	//!< a replaced session is told from a new one for this long
		static constexpr int64_t kSilence = 1000000000LL;		//!< after this long without a datagram the next one starts the order again, whatever its sequence
		static constexpr size_t kMaxRetired = 8;

		void Reset()
		{
			mHasSession = false;
			mRetired.clear();
		}

		Result Accept(const PoseMessage& pPose, const int64_t pNow)
		{
			if (!pPose.hasReference)
			{
				return Result::Accepted;
			}

			while (!mRetired.empty() && pNow - mRetired.front().time > kRetiredMemory)
			{
				mRetired.erase(mRetired.begin());
			}

			if (!mHasSession)
			{
				Adopt(pPose, pNow);
				return Result::Accepted;
			}

			if (pPose.sessionId != mSession)
			{
				// the stream restarted - or one of the last datagrams of the one before arrives late
				for (const Retired& lRetired : mRetired)
				{
					if (lRetired.session == pPose.sessionId)
					{
						return Result::Retired;
					}
				}
				if (mRetired.size() >= kMaxRetired)
				{
					mRetired.erase(mRetired.begin());
				}
				mRetired.push_back(Retired{ mSession, pNow });
				Adopt(pPose, pNow);
				return Result::NewSession;
			}

			const int32_t lStep = static_cast<int32_t>(static_cast<uint32_t>(pPose.sequence) - static_cast<uint32_t>(mSequence));
			if (lStep <= 0 && pNow - mLastAccepted <= kSilence)
			{
				return Result::Stale;
			}
			mSequence = pPose.sequence;
			mLastAccepted = pNow;
			return Result::Accepted;
		}

	private:
		struct Retired
		{
			int32_t session;
			int64_t time;
		};

		void Adopt(const PoseMessage& pPose, const int64_t pNow)
		{
			mHasSession = true;
			mSession = pPose.sessionId;
			mSequence = pPose.sequence;
			mLastAccepted = pNow;
		}

		bool					mHasSession{ false };
		int32_t					mSession{ 0 };
		int32_t					mSequence{ 0 };
		int64_t					mLastAccepted{ 0 };
		std::vector<Retired>	mRetired;
	};

	/// How the live camera is drawn from a Timeline, at every render of the device: which delay, and how the frames were made.
	class Playback
	{
	public:
		enum class Mode : uint8_t
		{
			Auto,		//!< the delay follows the jitter of the network, between the floor (Settings::delay) and Settings::maxDelay
			Fixed,		//!< Settings::delay, 0 - the newest sample extrapolated by its age
			Newest		//!< the newest sample as received, nothing is guessed or blended, the network jitter is seen as it is
		};

		/// how much of the jitter the Auto delay covers: less delay, or fewer frames that have to be guessed
		enum class Responsiveness : uint8_t
		{
			Responsive,	//!< the 99th percentile of the late packets
			Balanced,	//!< and half a sample spacing
			Smooth		//!< and a whole one
		};

		struct Settings
		{
			Mode mode{ Mode::Auto };
			Responsiveness responsiveness{ Responsiveness::Balanced };
			int64_t delay{ 0 };			//!< ns, the delay of Fixed, the floor of Auto
			int64_t maxDelay{ 60000000LL };	//!< ns, the ceiling of Auto
		};

		struct Frame
		{
			Timeline::RenderInfo render;
			int64_t delay{ 0 };			//!< ns, the delay this frame was drawn with
		};

		/// the share of the frames of the last seconds, percent
		struct Shares
		{
			double interpolated{ 0.0 };
			double extrapolated{ 0.0 };
			double held{ 0.0 };
		};

		static constexpr double kRiseSecondsPerSecond = 0.020;		//!< the delay grows at most this fast (a 2 % time warp, while the jitter grows)
		static constexpr double kFallSecondsPerSecond = 0.002;		//!< and shrinks this fast (0.2 %, not visible)
		static constexpr double kMaxStepSeconds = 0.1;				//!< a stall of the UI does not make a jump
		static constexpr int kStatSeconds = 5;

		void Reset()
		{
			mHasDelay = false;
			mDelay = 0.0;
			mLastNow = 0;
			mBuckets = {};
		}

		Frame Render(const Timeline& pTimeline, const int64_t pNow, const Settings& pSettings, const ConvertSettings& pConvert)
		{
			Frame lFrame;
			switch (pSettings.mode)
			{
				case Mode::Newest:
					lFrame.render = pTimeline.RenderNewest(pConvert);
					mHasDelay = false;
					break;

				case Mode::Fixed:
					lFrame.delay = std::max<int64_t>(pSettings.delay, 0);
					lFrame.render = pTimeline.Render(pNow, lFrame.delay, pConvert);
					mHasDelay = false;
					break;

				default:
					lFrame.delay = AutoDelay(pTimeline, pNow, pSettings);
					lFrame.render = pTimeline.Render(pNow, lFrame.delay, pConvert);
					break;
			}
			Count(lFrame.render.state, pNow);
			return lFrame;
		}

		Shares GetShares(const int64_t pNow) const
		{
			const int64_t lSecond = pNow / 1000000000LL;
			uint32_t lCounts[3]{};
			for (const Bucket& lBucket : mBuckets)
			{
				if (lBucket.second > lSecond - kStatSeconds && lBucket.second <= lSecond)
				{
					for (int i = 0; i < 3; ++i)
					{
						lCounts[i] += lBucket.counts[i];
					}
				}
			}
			const double lTotal = static_cast<double>(lCounts[0]) + lCounts[1] + lCounts[2];
			Shares lShares;
			if (lTotal > 0.0)
			{
				lShares.interpolated = 100.0 * lCounts[0] / lTotal;
				lShares.extrapolated = 100.0 * lCounts[1] / lTotal;
				lShares.held = 100.0 * lCounts[2] / lTotal;
			}
			return lShares;
		}

	private:
		struct Bucket
		{
			int64_t second{ -1000 };
			uint32_t counts[3]{};		//!< interpolated, extrapolated, held
		};

		static void Quantile(const Responsiveness pResponsiveness, double& pQuantile, double& pMargin)
		{
			pQuantile = 0.99;
			switch (pResponsiveness)
			{
				case Responsiveness::Responsive: pMargin = 0.0; break;
				case Responsiveness::Smooth: pMargin = 1.0; break;
				default: pMargin = 0.5; break;
			}
		}

		/// the late packets of the network (a quantile of the excess delay) and a margin of the sample spacing, moving at a limited speed
		int64_t AutoDelay(const Timeline& pTimeline, const int64_t pNow, const Settings& pSettings)
		{
			const double lFloor = static_cast<double>(std::max<int64_t>(pSettings.delay, 0));
			const double lCeiling = std::max(static_cast<double>(pSettings.maxDelay), lFloor);

			double lQuantile, lMargin;
			Quantile(pSettings.responsiveness, lQuantile, lMargin);
			const double lWanted = std::min(std::max(
				static_cast<double>(pTimeline.ExcessQuantile(lQuantile)) + lMargin * static_cast<double>(pTimeline.SampleSpacing()), lFloor), lCeiling);

			if (!mHasDelay)
			{
				mDelay = lWanted;
				mHasDelay = true;
			}
			else
			{
				const double lStep = std::min(std::max(static_cast<double>(pNow - mLastNow) * 1e-9, 0.0), kMaxStepSeconds);
				mDelay = (lWanted > mDelay) ? std::min(lWanted, mDelay + kRiseSecondsPerSecond * lStep * 1e9)
											: std::max(lWanted, mDelay - kFallSecondsPerSecond * lStep * 1e9);
			}
			mDelay = std::min(std::max(mDelay, lFloor), lCeiling);
			mLastNow = pNow;
			return static_cast<int64_t>(mDelay);
		}

		void Count(const Timeline::RenderState pState, const int64_t pNow)
		{
			int lIndex;
			switch (pState)
			{
				case Timeline::RenderState::Interpolated: lIndex = 0; break;
				case Timeline::RenderState::Extrapolated: lIndex = 1; break;
				case Timeline::RenderState::Held: lIndex = 2; break;
				default: return;
			}
			const int64_t lSecond = pNow / 1000000000LL;
			Bucket& lBucket = mBuckets[static_cast<size_t>(lSecond % kStatSeconds)];
			if (lBucket.second != lSecond)
			{
				lBucket = Bucket();
				lBucket.second = lSecond;
			}
			++lBucket.counts[lIndex];
		}

		bool	mHasDelay{ false };
		double	mDelay{ 0.0 };		//!< ns
		int64_t	mLastNow{ 0 };
		std::array<Bucket, kStatSeconds>	mBuckets{};
	};
}
