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

#include <util_geometry_msgs/util_geometry_msgs.hpp>

#include "util_lanelet.hpp"

namespace util_lanelet {

lanelet_map_wrapper::lanelet_map_wrapper(std::string filename, const std::shared_ptr<util_geo_coordinates::CoordinateTransformRos> coordinateTransformPtr) {
    searchDistanceDegrees_ = 0.0001;
    theMapPtr_ = std::make_shared<LLet::LaneletMap>(filename);
    coordinateTransformPtr_ = coordinateTransformPtr;
}

void lanelet_map_wrapper::setDebugFolder(std::string foldername) {
    debugFolder_ = foldername;
}

void lanelet_map_wrapper::initializeBoundaryPolygons() {
    LLet::BoundingBox world(std::make_tuple(-180.0, -180.0, 180.0, 180.0));
    std::vector<LLet::lanelet_ptr_t> laneletPtrs = theMapPtr_->query(world);
    for (const LLet::lanelet_ptr_t& lanelet : laneletPtrs) {
        util_eigen_geometry::polygon_ptr_t& rightPolygonPtr = rightPolygonMap_[lanelet->id()];
        rightPolygonPtr = std::make_shared<util_eigen_geometry::polygon_t>();
        const LLet::strip_ptr_t& rightBound = std::get<LLet::RIGHT>(lanelet->bounds());
        for (size_t i = 0; i < rightBound->pts().size(); i++) {
            Eigen::Vector2d vec = getEigenVector2dFromLLetPoint(rightBound->pts()[i]);
            rightPolygonPtr->push_back(vec);
        }

        util_eigen_geometry::polygon_ptr_t& leftPolygonPtr = leftPolygonMap_[lanelet->id()];
        leftPolygonPtr = std::make_shared<util_eigen_geometry::polygon_t>();
        const LLet::strip_ptr_t& leftBound = std::get<LLet::LEFT>(lanelet->bounds());
        for (size_t i = 0; i < leftBound->pts().size(); i++) {
            Eigen::Vector2d vec = getEigenVector2dFromLLetPoint(leftBound->pts()[i]);
            leftPolygonPtr->push_back(vec);
        }
    }
}


void lanelet_map_wrapper::calculateOffsetPolygons(LLet::SIDE side, double offset) {

    for (auto const& pair : rightPolygonMap_) {
        util_eigen_geometry::polygon_ptr_t& offsetPolygonPtr = offsetPolygonMap_[pair.first];
        offsetPolygonPtr = std::make_shared<util_eigen_geometry::polygon_t>();

        util_eigen_geometry::polygon_ptr_t boundPtr;
        util_eigen_geometry::polygon_ptr_t oppositeBoundPtr;

        if (side == LLet::SIDE::RIGHT) {
            boundPtr = rightPolygonMap_[pair.first];
            oppositeBoundPtr = leftPolygonMap_[pair.first];
        } else if (side == LLet::SIDE::LEFT) {
            oppositeBoundPtr = rightPolygonMap_[pair.first];
            boundPtr = leftPolygonMap_[pair.first];
        } else {
            throw std::invalid_argument("side must be LEFT or RIGHT");
        }

        // first point
        Eigen::Vector2d firstDelta, firstPoint;
        firstDelta = oppositeBoundPtr->front() - boundPtr->front();
        firstPoint = boundPtr->front() + offset * firstDelta / firstDelta.norm();
        offsetPolygonPtr->push_back(firstPoint);

        // intermediate points
        for (size_t i = 1; i < boundPtr->size() - 1; i++) {
            Eigen::Vector2d vecA, vecB, vecC, vecAB, vecAC, point, delta;
            vecA = (*boundPtr)[i - 1];
            vecB = (*boundPtr)[i];
            vecC = (*boundPtr)[i + 1];
            delta = (vecC - vecB) / (vecC - vecB).norm() + (vecA - vecB) / (vecA - vecB).norm();
            delta /= delta.norm();

            vecAB = vecB - vecA;
            vecAC = vecC - vecA;
            double direction = vecAB[0] * vecAC[1] - vecAB[1] * vecAC[0]; // 3rd row of cross product
            int sign = (direction < 0) ? -1 : (direction > 0);            // signum

            point = vecB + sign * offset * delta;

            offsetPolygonPtr->push_back(point);
        }

        // last point
        Eigen::Vector2d lastDelta, lastPoint;
        lastDelta = oppositeBoundPtr->back() - boundPtr->back();
        lastPoint = boundPtr->back() + offset * lastDelta / lastDelta.norm();
        offsetPolygonPtr->push_back(lastPoint);
    }
}

void lanelet_map_wrapper::getOffsetPolygon(util_eigen_geometry::polygon_t& polygon, std::vector<int32_t> laneletIds) {

    polygon.clear();

    for (const int32_t& laneletId : laneletIds) {
        polygon.insert(polygon.end(), offsetPolygonMap_[laneletId]->begin(), offsetPolygonMap_[laneletId]->end());
        polygon.pop_back();
    }

    polygon.push_back(offsetPolygonMap_[laneletIds.back()]->back());
}

std::vector<int32_t> lanelet_map_wrapper::getLaneletsFromMotionState(
    const automated_driving_msgs::MotionState& motionState) {

    double lat, lon;
    std::tie(lat, lon) = coordinateTransformPtr_->xy2ll(motionState.pose.pose.position.x, motionState.pose.pose.position.y);
    LLet::BoundingBox bBox(std::make_tuple(lat - searchDistanceDegrees_,
                                           lon - searchDistanceDegrees_,
                                           lat + searchDistanceDegrees_,
                                           lon + searchDistanceDegrees_));
    std::vector<LLet::lanelet_ptr_t> laneletPtrs = theMapPtr_->query(bBox);

    return laneletPtrsToLaneletIds(laneletPtrs);
}

int32_t lanelet_map_wrapper::getMostLikelyLaneletIdByOrientation(const automated_driving_msgs::MotionState& motionState,
                                                                 std::vector<int32_t> laneletIds) {
    if (laneletIds.size() == 0) {
        throw std::invalid_argument("laneletIds.size() must not be zero");
    }

    if (laneletIds.size() == 1) {
        return laneletIds.front();
    }

    double highestCosineSimilarity = -1.0;
    int32_t mostLikelyLaneletId = 0;

    Eigen::Isometry3d eigenPose3d;
    util_geometry_msgs::conversions::fromMsg(motionState.pose.pose, eigenPose3d);
    Eigen::Affine2d eigenPose = util_eigen_geometry::affine2dFromXYOfAffine3d(eigenPose3d);

    for (size_t i : laneletIds) {

        double cosineSimilarity = util_eigen_geometry::cosineSimilarity(eigenPose, *(rightPolygonMap_[i]));

        if (cosineSimilarity > highestCosineSimilarity) {
            mostLikelyLaneletId = i;
            highestCosineSimilarity = cosineSimilarity;
        }
    }

    return mostLikelyLaneletId;
}

int32_t lanelet_map_wrapper::getMostLikelyLaneletIdByPreviousLaneletId(
    const automated_driving_msgs::MotionState& motionState,
    const int32_t prevLaneletId,
    std::vector<int32_t> laneletIds) {
    int32_t mostLikelyLaneletId = 0;

    // if previous Id is still valid, take it
    if (std::find(laneletIds.begin(), laneletIds.end(), prevLaneletId) != laneletIds.end()) {
        mostLikelyLaneletId = prevLaneletId;
        return mostLikelyLaneletId;
    }

    // else: look at the successors
    std::vector<int32_t> successors = getLaneletSuccessors(prevLaneletId);
    for (int32_t laneletId : successors) {
        if (std::find(laneletIds.begin(), laneletIds.end(), laneletId) != laneletIds.end()) {
            mostLikelyLaneletId = laneletId;
            return mostLikelyLaneletId;
        }
    }

    // else: look at the successors of the successors
    for (int32_t laneletId : successors) {
        std::vector<int32_t> sucSuccessors = getLaneletSuccessors(laneletId);
        for (int32_t laneletId2 : sucSuccessors) {
            if (std::find(laneletIds.begin(), laneletIds.end(), laneletId2) != laneletIds.end()) {
                mostLikelyLaneletId = laneletId2;
                return mostLikelyLaneletId;
            }
        }
    }

    // else, get it by orientation
    mostLikelyLaneletId = getMostLikelyLaneletIdByOrientation(motionState, laneletIds);
    return mostLikelyLaneletId;
}

int32_t lanelet_map_wrapper::getMostLikelyLaneletId(const automated_driving_msgs::MotionState& motionState,
                                                    const std::vector<int32_t> desiredLaneletIds) {
    std::cout << "not yet implemented!" << std::endl;
    return 0;
}

std::vector<int32_t> lanelet_map_wrapper::getArbitraryLaneletSequence(int32_t startLaneletId,
                                                                      double lengthExclStartLanelet) {
    std::vector<int32_t> laneletSequence;
    double remainingLenght = lengthExclStartLanelet;

    laneletSequence.push_back(startLaneletId);
    int32_t currLaneletId = startLaneletId;

    while (remainingLenght > 0) {
        std::vector<int32_t> successors = getLaneletSuccessors(currLaneletId);
        if (successors.size() == 0) {
            return laneletSequence;
        }
        size_t choice = rand() % successors.size();
        currLaneletId = successors[choice];
        laneletSequence.push_back(currLaneletId);
        remainingLenght -= theMapPtr_->lanelet_by_id(currLaneletId)->length();
    }

    return laneletSequence;
}

bool lanelet_map_wrapper::containsLanelets() {
    LLet::BoundingBox world(std::make_tuple(-180.0, -180.0, 180.0, 180.0));
    std::vector<LLet::lanelet_ptr_t> laneletPtrs = theMapPtr_->query(world);
    return (laneletPtrs.size() > 0);
}

Eigen::Vector2d lanelet_map_wrapper::getEigenVector2dFromLLetPoint(const LLet::point_with_id_t& lletPoint) {
    Eigen::Vector2d eigenVec2d;
    std::tie(eigenVec2d[0], eigenVec2d[1]) = coordinateTransformPtr_->ll2xy(std::get<LLet::LAT>(lletPoint), std::get<LLet::LON>(lletPoint));
    return eigenVec2d;
}

std::vector<LLet::lanelet_ptr_t> lanelet_map_wrapper::laneletIdsToLaneletPtrs(const std::vector<int32_t> laneletIds) {
    std::vector<LLet::lanelet_ptr_t> laneletPtrs;
    for (int32_t laneletId : laneletIds) {
        laneletPtrs.push_back(theMapPtr_->lanelet_by_id(laneletId));
    }
    return laneletPtrs;
}

std::vector<int32_t> lanelet_map_wrapper::laneletPtrsToLaneletIds(const std::vector<LLet::lanelet_ptr_t> laneletPtrs) {
    std::vector<int32_t> laneletIds;
    for (const LLet::lanelet_ptr_t& lanelet : laneletPtrs) {
        laneletIds.push_back(lanelet->id());
    }
    return laneletIds;
}

std::vector<int32_t> lanelet_map_wrapper::getLaneletSuccessors(int32_t laneletId) {

    const LLet::Graph& G = theMapPtr_->graph();
    std::vector<int32_t> successors;

    BOOST_FOREACH (const auto& edge, boost::edges(G)) {
        auto src_vtx = boost::source(edge, G);
        auto dest_vtx = boost::target(edge, G);

        if (G[src_vtx].lanelet->id() == laneletId) {
            successors.push_back(G[dest_vtx].lanelet->id());
        }
    }
    return successors;
}

} // namespace util_lanelet
