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
#include <memory>
#include <stdexcept>
#include <vector>

#include "rover_gps/application/monitor_gps_use_case.hpp"

using rover_gps::application::MonitorGpsUseCase;
using namespace rover_gps::domain;  // NOLINT

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;

class FakeHealthPublisher : public GpsHealthPublisherPort
{
public:
    void publish(const GpsHealthReport & report) override {reports.push_back(report);}

    std::vector<GpsHealthReport> reports;
};

GnssFix fixAtNorth(double north_m, double stamp_s)
{
    GnssFix fix;
    fix.latitude_deg = 45.0 + north_m / kEarthRadiusM * 180.0 / kPi;
    fix.longitude_deg = 25.0;
    fix.status = FixStatus::Fix;
    fix.horizontal_std_m = 1.0;
    fix.stamp_s = stamp_s;
    return fix;
}

}  // namespace

TEST(MonitorGpsUseCaseTest, PublishesEvaluationOnTick)
{
    auto publisher = std::make_shared<FakeHealthPublisher>();
    MonitorGpsUseCase use_case(publisher, GpsHealthThresholds{});

    use_case.onTick(1.0);
    use_case.onFix(fixAtNorth(0.0, 2.0));
    use_case.onTick(2.5);

    ASSERT_EQ(publisher->reports.size(), 2u);
    EXPECT_EQ(publisher->reports[0].level, HealthLevel::Stale);
    EXPECT_EQ(publisher->reports[1].level, HealthLevel::Ok);
    EXPECT_EQ(publisher->reports[1].fix_count, 1u);
}

TEST(MonitorGpsUseCaseTest, RequiresPublisher)
{
    EXPECT_THROW(MonitorGpsUseCase(nullptr, GpsHealthThresholds{}), std::invalid_argument);
}
