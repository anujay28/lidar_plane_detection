#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "lidar_touchoff/plane_detection.h"

using Catch::Matchers::WithinRel;

TEST_CASE("Planes to Point Normal form", "[PlaneToPointNormal]") {
    Eigen::Vector3d p1(0, 0, 0);
    Plane pl1(0, 0, 1, 1);

    REQUIRE(PlaneToPointNormal(pl1, p1) ==
            PointNormal{Eigen::Vector3d(0, 0, -1), Eigen::Vector3d(0, 0, 1)});
}

TEST_CASE("null cloud", "[PlaneEstimate]") {
    PointCloud::Ptr cloud;

    REQUIRE_THROWS_AS(EstimatePlane(cloud, 0.01), PlaneEstimatorError);
}

TEST_CASE("less than 3 points", "[PlaneEstimate]") {
    PointCloud::Ptr cloud(new PointCloud());

    cloud->points.resize(2);
    REQUIRE_THROWS_AS(EstimatePlane(cloud, 0.01), PlaneEstimatorError);
}

TEST_CASE("colinear points", "[PlaneEstimate]") {
    auto cloud = boost::make_shared<PointCloud>();
    cloud->points.resize(3);

    cloud->points[0].x = 0.0;
    cloud->points[0].y = 0.0;
    cloud->points[0].z = 0.0;

    cloud->points[0].x = 0.0;
    cloud->points[0].y = 0.0;
    cloud->points[0].z = 1.0;

    cloud->points[0].x = 0.0;
    cloud->points[0].y = 0.0;
    cloud->points[0].z = 2.0;

    REQUIRE_THROWS_AS(EstimatePlane(cloud, 0.01), PlaneEstimatorError);
}

TEST_CASE("Simple Plane Estimate", "[PlaneEstimate]") {

    PointCloud::Ptr cloud(new PointCloud());

    cloud->width = 5;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    // plane on xy plane with z = 1

    cloud->points[0].x = 0.0;
    cloud->points[0].y = 0.0;
    cloud->points[0].z = 1.0;

    cloud->points[1].x = 1.0;
    cloud->points[1].y = 0.0;
    cloud->points[1].z = 1.0;

    cloud->points[2].x = 0.0;
    cloud->points[2].y = 1.0;
    cloud->points[2].z = 1.0;

    cloud->points[3].x = 1.0;
    cloud->points[3].y = 1.0;
    cloud->points[3].z = 1.0;

    cloud->points[4].x = 1.0;
    cloud->points[4].y = 2.0;
    cloud->points[4].z = 1.0;

    Plane P = EstimatePlane(cloud, 0.01);

    REQUIRE(P == Plane(0, 0, 1, -1));
}

TEST_CASE("Plane Filtering", "[PointCloud]") {
    auto cloud = boost::make_shared<PointCloud>();
    cloud->points.resize(3);

    cloud->points[0].x = 0.0;
    cloud->points[0].y = 0.0;
    cloud->points[0].z = 0.0;

    cloud->points[1].x = 1.0;
    cloud->points[1].y = 1.0;
    cloud->points[1].z = 1.0;

    cloud->points[2].x = 2.0;
    cloud->points[2].y = 1.0;
    cloud->points[2].z = 1.0;

    cloud->width = 3;
    cloud->height = 1;

    auto filtered = CloudFilterWindow(cloud, Box3d{-2, 2, -2, 2, -2, 2});
    fmt::print("Filtered[3]: {}\n", fmt::join(filtered->points, ", "));
    CHECK(filtered->points.size() == 3);

    filtered = CloudFilterWindow(cloud, Box3d{-2, 1, -2, 2, -2, 2});
    fmt::print("Filtered[2]: {}\n", fmt::join(filtered->points, ", "));
    CHECK(filtered->points.size() == 2);

    filtered = CloudFilterWindow(cloud, Box3d{-2, 0.5, -2, 2, -2, 2});
    fmt::print("Filtered[1]: {}\n", fmt::join(filtered->points, ", "));
    CHECK(filtered->points.size() == 1);

    filtered = CloudFilterWindow(cloud, Box3d{3, 4, -2, 2, -2, 2});
    fmt::print("Filtered[0]: {}\n", fmt::join(filtered->points, ", "));
    CHECK(filtered->points.size() == 0);
}

TEST_CASE("Bounding box transformation", "[Bounding Box]") {
    auto box = Box3d{-1, 1, -1, 1, -1, 1};
    Eigen::Matrix4d trans;

    SECTION("Identity") {
        trans << 1, 0, 0, 0,    //
            0, 1, 0, 0,         //
            0, 0, 1, 0,         //
            0, 0, 0, 1;         //
        auto b = TransformBox3d(box, trans);

        CHECK(b == box);
    }
    SECTION("X becomes y") {
        trans << 0, 1, 0, 1,    //
            1, 0, 0, 0,         //
            0, 0, 1, 0,         //
            0, 0, 0, 1;         //
        auto b = TransformBox3d(box, trans);
        fmt::print("BBox: {}, B: {}\n", box, b);
        CHECK(b == Box3d{0.0, 2.0, -1, 1, -1, 1});
    }
    SECTION("Rotate on yaw") {
        trans << 0.7071068, -0.7071068, 0, 1,    //
            0.7071068, 0.7071068, 0, 0,          //
            0, 0, 1, 0,                          //
            0, 0, 0, 1;                          //
        auto b = TransformBox3d(box, trans);
        fmt::print("BBox: {}, B: {}\n", box, b);
        fmt::print(
            "{}, {}, {}, {}, {}, {}\n", b.xmin, b.xmax, b.ymin, b.ymax, b.zmin, b.zmax);

        Box3d tb{-0.41421360000000007, 2.4142136, -1.4142136, 1.4142136, -1, 1};

        CHECK_THAT(b.xmin, WithinRel(tb.xmin, 1e-6));
        CHECK_THAT(b.xmax, WithinRel(tb.xmax, 1e-6));

        CHECK_THAT(b.ymin, WithinRel(tb.ymin, 1e-6));
        CHECK_THAT(b.ymax, WithinRel(tb.ymax, 1e-6));

        CHECK_THAT(b.zmin, WithinRel(tb.zmin, 1e-6));
        CHECK_THAT(b.zmax, WithinRel(tb.zmax, 1e-6));
    }
}

TEST_CASE("Tansform Point Normal form", "[PointNormal]") {
    PointNormal pn{Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)};

    Eigen::Matrix4d m;

    SECTION("Identity") {
        m << 1, 0, 0, 0,    //
            0, 1, 0, 0,     //
            0, 0, 1, 0,     //
            0, 0, 0, 1;     //

        auto rpn = TransformPointNormal(pn, Eigen::Isometry3d(m));
        CHECK(pn == rpn);
    }

    SECTION("Translation") {
        m << 1, 0, 0, 1,    //
            0, 1, 0, 1,     //
            0, 0, 1, 1,     //
            0, 0, 0, 1;     //

        auto rpn = TransformPointNormal(pn, Eigen::Isometry3d(m));
        CHECK(PointNormal{Eigen::Vector3d(1, 1, 2), Eigen::Vector3d(0, 0, 1)} == rpn);
    }

    SECTION("Rotation Z <--> X") {
        m << 0, 0, 1, 1,    //
            0, 1, 0, 1,     //
            1, 0, 0, 1,     //
            0, 0, 0, 1;     //

        auto rpn = TransformPointNormal(pn, Eigen::Isometry3d(m));
        CHECK(PointNormal{Eigen::Vector3d(2, 1, 1), Eigen::Vector3d(1, 0, 0)} == rpn);
    }
}

TEST_CASE("Average planes", "[AveragePlanes]") {
    SECTION("Planes of different sign") {
        std::vector<Plane> planes = {
            Plane(0, 0, 1, -1), Plane(0, 0, -1, 1), Plane(0, 0, 1, -1)};
        CHECK(AveragePlanes(planes) == Plane(0, 0, -1, 1));
    }

    SECTION("Simple Average") {
        std::vector<Plane> planes = {
            Plane(0, 0, 1, -1.1), Plane(0, 0, -1.5, 1), Plane(0, 0, 0.5, -0.9)};
        CHECK(AveragePlanes(planes) == Plane(0, 0, -1, 1));
    }

    SECTION("Empty List") {
        std::vector<Plane> planes;
        CHECK_THROWS_AS(AveragePlanes(planes), std::runtime_error);
    }
}
