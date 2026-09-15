// Copyright 2026 Gole Robotics
#include "slam_3d_localization/low_compute_budget.hpp"
#include "slam_3d_localization/thread_cpu_counter.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace gole::slam_3d_localization
{
namespace
{

TEST(LowComputeBudget, KeepsSmallCloudsUnsubsampled)
{
  EXPECT_EQ(LowComputeBudget::pointStride(0U), 1U);
  EXPECT_EQ(LowComputeBudget::pointStride(1U), 1U);
  EXPECT_EQ(LowComputeBudget::pointStride(1024U), 1U);
  EXPECT_EQ(LowComputeBudget::pointStride(1025U), 2U);
}

TEST(ThreadCpuCounter, StartsAtZeroAndCountsCompletedScopes)
{
  ThreadCpuCounter counter;
  EXPECT_DOUBLE_EQ(counter.ms(), 0.0);
  EXPECT_EQ(counter.count(), 0U);
  {
    const ThreadCpuCounter::Scope scope(counter);
    EXPECT_EQ(counter.count(), 0U);
  }
  EXPECT_EQ(counter.count(), 1U);
  EXPECT_GE(counter.ms(), 0.0);
  EXPECT_EQ(counter.clockFailures(), 0U);
}

TEST(ThreadCpuCounter, AccumulatesMonotonicallyWithoutLongWorkloads)
{
  ThreadCpuCounter counter;
  double previous_ms = 0.0;
  for (std::uint64_t iteration = 0U; iteration < 3U; ++iteration) {
    {
      const ThreadCpuCounter::Scope scope(counter);
      volatile std::uint64_t sum = 0U;
      for (std::uint64_t index = 0U; index < 1000U; ++index) {
        sum += index;
      }
      (void)sum;
    }
    EXPECT_GE(counter.ms(), previous_ms);
    EXPECT_EQ(counter.count(), iteration + 1U);
    previous_ms = counter.ms();
  }
  EXPECT_EQ(counter.clockFailures(), 0U);
}

TEST(LowComputeBudget, BoundsSampleCountWithoutOverflow)
{
  const std::array<std::size_t, 9> counts{
    1U, 1023U, 1024U, 1025U, 2047U, 2048U, 2049U, 1000000U,
    std::numeric_limits<std::size_t>::max()};
  for (const auto count : counts) {
    const auto stride = LowComputeBudget::pointStride(count);
    ASSERT_GT(stride, 0U);
    const auto sampled = 1U + (count - 1U) / stride;
    EXPECT_LE(sampled, LowComputeBudget::kMaxDeskewPoints) << "count=" << count;
  }
}

TEST(LowComputeBudget, AdmitsFirstScanIncludingZeroStamp)
{
  std::int64_t previous = -1;
  EXPECT_TRUE(LowComputeBudget::admitScan(0, previous, 100000000LL));
  EXPECT_EQ(previous, 0);
}

TEST(LowComputeBudget, RejectsInvalidScanWithoutAdvancingWatermark)
{
  std::int64_t previous = 1000000000LL;
  EXPECT_FALSE(LowComputeBudget::admitScan(-1, previous, 100000000LL));
  EXPECT_EQ(previous, 1000000000LL);
}

TEST(LowComputeBudget, EnforcesHalfHertzUsingSensorTime)
{
  std::int64_t previous = 1000000000LL;
  EXPECT_FALSE(LowComputeBudget::admitScan(1000000000LL, previous, 100000000LL));
  EXPECT_FALSE(LowComputeBudget::admitScan(2999999999LL, previous, 100000000LL));
  EXPECT_EQ(previous, 1000000000LL);
  EXPECT_TRUE(LowComputeBudget::admitScan(3000000000LL, previous, 100000000LL));
  EXPECT_EQ(previous, 3000000000LL);
}

TEST(LowComputeBudget, IgnoresSmallReorderingButAdmitsNewEpoch)
{
  std::int64_t previous = 5000000000LL;
  constexpr std::int64_t reset_threshold = 1000000000LL;
  EXPECT_FALSE(LowComputeBudget::admitScan(4999999999LL, previous, reset_threshold));
  EXPECT_FALSE(LowComputeBudget::admitScan(4000000000LL, previous, reset_threshold));
  EXPECT_EQ(previous, 5000000000LL);
  EXPECT_TRUE(LowComputeBudget::admitScan(3999999999LL, previous, reset_threshold));
  EXPECT_EQ(previous, 3999999999LL);
  EXPECT_FALSE(LowComputeBudget::admitScan(4000000000LL, previous, reset_threshold));
}

TEST(LowComputeBudget, OdometryIsDueAtTwoHertzBoundary)
{
  EXPECT_TRUE(LowComputeBudget::odometryDue(0, -1));
  EXPECT_FALSE(LowComputeBudget::odometryDue(1000000000LL, 1000000000LL));
  EXPECT_FALSE(LowComputeBudget::odometryDue(1499999999LL, 1000000000LL));
  EXPECT_TRUE(LowComputeBudget::odometryDue(1500000000LL, 1000000000LL));
  EXPECT_TRUE(LowComputeBudget::odometryDue(1500000001LL, 1000000000LL));
}

TEST(LowComputeBudget, OdometryRewindDoesNotBlockNewEpoch)
{
  EXPECT_TRUE(LowComputeBudget::odometryDue(0, 5000000000LL));
}

TEST(LowComputeBudget, SelectsTwoPublicationsFromTwoHundredHertzTimestamps)
{
  std::int64_t previous = -1;
  std::size_t publications = 0U;
  std::size_t input_measurements = 0U;
  for (std::int64_t stamp = 0; stamp < 1000000000LL; stamp += 5000000LL) {
    ++input_measurements;
    if (LowComputeBudget::odometryDue(stamp, previous)) {
      previous = stamp;
      ++publications;
    }
  }
  EXPECT_EQ(input_measurements, 200U);
  EXPECT_EQ(publications, 2U);
}

}  // namespace
}  // namespace gole::slam_3d_localization
