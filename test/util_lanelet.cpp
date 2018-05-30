/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "gtest/gtest.h"

#include "util_lanelet.hpp"

#include <boost/filesystem.hpp>


class util_geo_coordinates::CoordinateTransformRosTest : public util_geo_coordinates::CoordinateTransformRos {
public:
    inline CoordinateTransformRosTest() : util_geo_coordinates::CoordinateTransformRos() {
        double originLat = 49.02;
        double originLon = 8.43;
        coordinateTransformPtr_ = std::make_shared<util_geo_coordinates::CoordinateTransform>(originLat, originLon);
        initialized_ = true;
    }
};

TEST(UtilLanelet, Initialization) {
    const std::string mapFileName = std::string(RES_DIRECTORY + std::string("test/lanelet_figure8.osm"));

    boost::filesystem::path mapPath(mapFileName);
    ASSERT_TRUE(boost::filesystem::exists(mapPath)) << "mapPath=" << mapPath;
    ASSERT_TRUE(mapPath.extension() == ".osm") << "mapPath=" << mapPath;


    util_geo_coordinates::CoordinateTransformRosTest ctNoRos{};
    util_geo_coordinates::CoordinateTransformRos ct =
        static_cast<util_geo_coordinates::CoordinateTransformRos>(ctNoRos);
    std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> ctPtr =
        std::make_shared<util_geo_coordinates::CoordinateTransformRos>(ct);

    util_lanelet::lanelet_map_wrapper laneletMapWrapper{mapFileName, ctPtr};
    ASSERT_TRUE(laneletMapWrapper.containsLanelets());
}
