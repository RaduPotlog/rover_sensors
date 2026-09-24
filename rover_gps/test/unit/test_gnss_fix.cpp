// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>

#include "rover_gps/domain/gnss_fix.hpp"

using rover_gps::domain::horizontalStdFromVariances;

TEST(GnssFixTest, HorizontalStdIsTheOneSigmaOfTheWorseAxis)
{
    EXPECT_DOUBLE_EQ(horizontalStdFromVariances(4.0, 9.0), 3.0);
    EXPECT_DOUBLE_EQ(horizontalStdFromVariances(9.0, 4.0), 3.0);
}

TEST(GnssFixTest, HorizontalStdWithEqualAxes)
{
    EXPECT_DOUBLE_EQ(horizontalStdFromVariances(0.25, 0.25), 0.5);
}

TEST(GnssFixTest, HorizontalStdOfZeroVarianceIsZero)
{
    EXPECT_DOUBLE_EQ(horizontalStdFromVariances(0.0, 0.0), 0.0);
}

TEST(GnssFixTest, NegativeVarianceIsNoEstimate)
{
    EXPECT_TRUE(std::isnan(horizontalStdFromVariances(-1.0, -1.0)));
    // A negative axis never masks a real one: the worse (larger) variance still wins.
    EXPECT_DOUBLE_EQ(horizontalStdFromVariances(-1.0, 16.0), 4.0);
}
