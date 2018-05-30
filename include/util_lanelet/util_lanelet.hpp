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

#pragma once

#include <stdexcept>
#include <stdlib.h>

#include <util_eigen_geometry/util_eigen_geometry.hpp>
#include <util_geo_coordinates_ros/util_geo_coordinates_ros.hpp>

#include <automated_driving_msgs/MotionState.h>
#include <sim_lanelet/BoundingBox.hpp>
#include <sim_lanelet/Lanelet.hpp>
#include <sim_lanelet/LaneletMap.hpp>
#include <sim_lanelet/llet_xml.hpp>



namespace util_lanelet {

class lanelet_map_wrapper {
public:
    lanelet_map_wrapper(std::string filename, const std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr);

    void setDebugFolder(std::string foldername);

    std::shared_ptr<LLet::LaneletMap> getMap();
    bool containsLanelets();
    void initializeBoundaryPolygons();
    void calculateOffsetPolygons(LLet::SIDE side, double offset);
    void getOffsetPolygon(util_eigen_geometry::polygon_t& polygon, std::vector<int32_t> laneletIds);

    std::vector<int32_t> getLaneletsFromMotionState(const automated_driving_msgs::MotionState& motionState);
    int32_t getMostLikelyLaneletIdByOrientation(const automated_driving_msgs::MotionState& motionState,
                                                std::vector<int32_t> laneletIds);
    int32_t getMostLikelyLaneletIdByPreviousLaneletId(const automated_driving_msgs::MotionState& motionState,
                                                      const int32_t prevLaneletId,
                                                      std::vector<int32_t> laneletIds);
    int32_t getMostLikelyLaneletId(const automated_driving_msgs::MotionState& motionState,
                                   const std::vector<int32_t> desiredLaneletIds);

    std::vector<int32_t> getLaneletSuccessors(int32_t laneletId);
    std::vector<int32_t> getArbitraryLaneletSequence(int32_t startLaneletId, double lengthExclStartLanelet);

    Eigen::Vector2d getEigenVector2dFromLLetPoint(const LLet::point_with_id_t& lletPoint);

    std::vector<LLet::lanelet_ptr_t> laneletIdsToLaneletPtrs(const std::vector<int32_t> laneletIds);
    static std::vector<int32_t> laneletPtrsToLaneletIds(const std::vector<LLet::lanelet_ptr_t> laneletPtrs);

private:
    std::shared_ptr<LLet::LaneletMap> theMapPtr_;
    std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr_;
    double searchDistanceDegrees_;
    std::unordered_map<int32_t, util_eigen_geometry::polygon_ptr_t> leftPolygonMap_;
    std::unordered_map<int32_t, util_eigen_geometry::polygon_ptr_t> rightPolygonMap_;
    std::unordered_map<int32_t, util_eigen_geometry::polygon_ptr_t> offsetPolygonMap_;
    std::string debugFolder_;

};

} // namespace util_lanelet
