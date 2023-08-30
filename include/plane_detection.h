#pragma once

#include <Eigen/Eigen>
#include <fmt/format.h>
#include <fmt/ostream.h>
#include <variant>
#include <pcl/segmentation/sac_segmentation.h>
#include <range/v3/view.hpp>
#include <range/v3/numeric/accumulate.hpp>
#include <lidar_touchoff/plane_types.h>

/**
 * Finds the nearest point on a plane to a given point.
 * @param plane the 4 plane coefficients
 * @param point a 3d point
 * @return the nearest point on the plane to the input point
*/
auto NearPointOnPlane(const Plane& plane, const Eigen::Vector3d& point) -> Eigen::Vector3d {
    return point - plane.dot(Plane{point.x(), point.y(), point.z(), 1.0}) * plane({0, 1, 2});
}

/**
 * Convert a set of plane coefficients into point normal form. The point is
 * set as the closest point on the plane relative to the input point.
 * @param plane the 4 plane coefficients
 * @param point a 3d point used to find the nearest point on the plane
 * @return PointNormal struct with a point on the plane and the normal of the
 *         plane
 */
PointNormal PlaneToPointNormal(const Plane& plane, const Point& point=Point{0,0,0}) {
    PointNormal pn;
    pn.normal = plane({0, 1, 2});
    pn.normal.normalize();
    pn.point = NearPointOnPlane(plane, point);
    return pn;
}

Plane PointNormalToPlane(const PointNormal& pn) {
  double d = pn.normal.transpose() * pn.point;
  return Plane{pn.normal.x(), pn.normal.y(), pn.normal.z(), -d};
}

/**
 * Estimate plane within a point cloud.
 * @param cloud point cloud
 * @param thresh distance threshold to be considered inlier
 */
Plane EstimatePlane(const PointCloud::Ptr& cloud, const double& thresh) {
    if (cloud == nullptr) {
        throw PlaneEstimatorError("Null pointer cloud");
    }
    if (cloud->points.size() < 3) {
        throw PlaneEstimatorError(fmt::format("Not enough points to fit a plane: {} < 3",
                                              cloud->points.size()));
    }
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coeff;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(thresh);
    seg.setMaxIterations(50);
    seg.setInputCloud(cloud);

    seg.segment(inliers, coeff);

    if (coeff.values.size() != 4) {
        throw PlaneEstimatorError("Failed to estimate plane.");
    }
    int zeros = ranges::accumulate(coeff.values | ranges::views::transform([](auto v) {
                                       return std::abs(v) == 0.0;
                                   }),
                                   0.0);
    if (zeros == 4) {
        throw PlaneEstimatorError(
            fmt::format("Resulting plane is null: P({})", fmt::join(coeff.values, ", ")));
    }
    return Plane{coeff.values[0], coeff.values[1], coeff.values[2], coeff.values[3]};
}

/**
 * Cloud Boudning box filter
 * @param cloud point cloud
 * @param box bounding box params
 * @return filtered point cloud
 */
PointCloud::Ptr CloudFilterWindow(const PointCloud::Ptr& cloud, const Box3d& box) {
    auto filtered = boost::make_shared<PointCloud>();
    for (const auto& p : cloud->points) {
        if (!(p.x < box.xmin || p.x > box.xmax || p.y < box.ymin || p.y > box.ymax ||
              p.z < box.zmin || p.z > box.zmax)) {
            filtered->points.push_back(p);
        }
    }
    filtered->width = filtered->points.size();
    filtered->height = 1;
    return filtered;
}

/**
 * Transform bounding box into new coordinate frame and axis align with a larger
 * containing bounding box
 * @param box bounding box
 * @param tran affine transformation matrix
 */
Box3d TransformBox3d(const Box3d& box, const Eigen::Matrix4d tran) {
    Eigen::Matrix<double, 8, 4> points;
    points << box.xmin, box.ymin, box.zmin, 1,    // 000
        box.xmin, box.ymin, box.zmax, 1,          // 001
        box.xmin, box.ymax, box.zmin, 1,          // 010
        box.xmin, box.ymax, box.zmax, 1,          // 011
        box.xmax, box.ymin, box.zmin, 1,          // 100
        box.xmax, box.ymin, box.zmax, 1,          // 101
        box.xmax, box.ymax, box.zmin, 1,          // 110
        box.xmax, box.ymax, box.zmax, 1;          // 111

    Eigen::Matrix<double, 8, 4> t_points = (tran * points.transpose()).transpose();

    auto pmin = t_points.colwise().minCoeff();
    auto pmax = t_points.colwise().maxCoeff();
    return Box3d{pmin(0), pmax(0), pmin(1), pmax(1), pmin(2), pmax(2)};
}

/**
 * Average planes of differing signs ensuring that resulting plane will have
 * a normal pointing towards the origin
 * @param planes vector of planes
 * @return Averaged plane normalized to ensure a positive d component
 */
Plane AveragePlanes(const std::vector<Plane>& planes) {
    if (planes.size() == 0) {
        throw std::runtime_error("Empty planes vector.");
    }
    if (planes.size() == 1) {
        return planes[0];
    }
    Plane avg_plane = ranges::accumulate(planes | ranges::views::transform([](auto p) {
                                             return std::signbit(p(3)) ? p * (-1) : p;
                                         }),
                                         Plane(0.0, 0.0, 0.0, 0.0))
                          .array() /
                      planes.size();
    avg_plane({0, 1, 2}) = avg_plane({0, 1, 2}).normalized();
    return avg_plane;
}

/**
 * Transform point normal into different coordinate frame
 * @param pn point normal object
 * @param trans eigen isometry transformation
 * @return transformed point normal
 */
PointNormal TransformPointNormal(const PointNormal& pn, const Eigen::Isometry3d& trans) {
    PointNormal res;

    res.normal = (trans.rotation() * pn.normal).normalized();
    res.point = trans * pn.point;
    return res;
}

Plane TransformPlane(const Plane& p, const Eigen::Isometry3d& trans) {
    return p.transpose() * trans.inverse().matrix();
}

/**
 * Estimate an averaged plane from a list of clouds and a bouding box in a
 * different coordinate frame. This fuction can be considered Lidar Touchoff
 * as it performs all of the necessary averaging and coordinate transforms
 * to filter the points cloud within the ouster frame and returns a plane in the
 * desired output frame_id
 * @param clouds vector of points clouds
 * @param output_bounds bounds in the output_frame_id. These will be transformed
 *                      into the cloud frame_id for filtering
 * @param thresh ransac plane segmentation threshold
 * @param output_T_cloud transform from the cloud frame_id into the output frame_id
 * @result averaged point normal form plane
 */
PointNormal EstimatePlane(const std::vector<PointCloud::Ptr>& clouds,
                          const Box3d& output_bounds,
                          const double& thresh,
                          const Eigen::Isometry3d& output_T_cloud,
                          const double& d_offset) {

    // Transform bounds from output frame_id to cloud frame_id
    auto bounds = TransformBox3d(output_bounds, output_T_cloud.inverse().matrix());

    // For each cloud filter and estimate the plane
    std::vector<Plane> planes;
    for (const auto& cloud : clouds) {
        auto fcloud = CloudFilterWindow(cloud, bounds);
        planes.push_back(EstimatePlane(fcloud, thresh));
    }

    // consolidate planes
    auto plane = AveragePlanes(planes);

    // Handle Ouster calibrated distance offset
    plane(3) = plane(3) + d_offset;

    // Transform plane into output frame_id
    return TransformPointNormal(PlaneToPointNormal(plane, Eigen::Vector3d(0, 0, 0)),
                                output_T_cloud);
}

/**
 * Compute the bounds of a point cloud in a different reference frame. The
 * resulting bounds are with respect to the reference frame T. This method is
 * intended to produce bounding boxes in alternative reference frames.
 * @param cloud point cloud
 * @param T transform from cloud frame to reference frame
 * @return tuple of bounds (xmin, xmax, ymin, ymax, zmin, zmax)
 * @note The resulting 6-tuple is not with respect to the cloud frame
 *      but rather the reference frame T
 * @note @code auto [xmin, xmax, ymin, ymax, zmin, zmax] = CloudTransformedBounds(cloud, T); @endcode
*/
auto CloudTransformedBounds(const PointCloud& cloud, const Eigen::Isometry3d& T) -> std::tuple<double, double, double, double, double, double> {
    std::array<double, 6> bounds = {std::numeric_limits<double>::max(), std::numeric_limits<double>::min(),
                                    std::numeric_limits<double>::max(), std::numeric_limits<double>::min(),
                                    std::numeric_limits<double>::max(), std::numeric_limits<double>::min()};
    for (const auto& point : cloud.points) {
        Eigen::Vector3d p = T * point.getVector3fMap().cast<double>();
        for (int i = 0; i < 3; i++) {
            bounds[2 * i]     = std::min(bounds[2 * i]    , p(i));
            bounds[2 * i + 1] = std::max(bounds[2 * i + 1], p(i));
        }
    }
    return std::make_tuple(bounds[0], bounds[1], bounds[2], bounds[3], bounds[4], bounds[5]);
}

/**
 * Convert a `sensor_msgs::PointCloud2` into a `PointCloud<PointXYZ>`. This
 * method provides a simple one line input of ros message with pcl output.
 * @param ros_cloud sensor_msgs::PointCloud2 ros message
 * @return pcl PointCloud
*/
PointCloud PCL2PointCloud(const sensor_msgs::PointCloud2& ros_cloud) {
    pcl::PCLPointCloud2 pcl_cloud;
    PointCloud cloud;
    pcl_conversions::toPCL(ros_cloud, pcl_cloud);
    pcl::fromPCLPointCloud2(pcl_cloud, cloud);
    return cloud;
}


/**
 * Cube style point cloud filter that operates on a cube's width, length, and
 * height in x, y, z dimensions respectively. The points are each transformed
 * into the cube's frame of reference.
 * @param cloud point cloud
 * @param cube filtering cube parameters Ex: `{0.2, 0.3, 0.4}` a cube centered
 *  centered at the origin with width(x)=0.2, length(y)=0.3, and height(z)=0.4
 * @param trans transform of the cube in cloud frame: `cube_T_cloud * point`
 * @return new filtered cloud
*/
auto CubeFilter(const PointCloud& cloud, const Eigen::Vector3d& cube, const Eigen::Isometry3d& trans=Eigen::Isometry3d::Identity()) -> PointCloud {
  auto filtered = PointCloud();

  for (const auto& p_ : cloud.points) {
    Eigen::Vector3d p = trans * Eigen::Vector3d(p_.x, p_.y, p_.z);
    if (!(p.x() < -cube.x()/2.0 || p.x() > cube.x()/2.0 ||
          p.y() < -cube.y()/2.0 || p.y() > cube.y()/2.0 ||
          p.z() < -cube.z()/2.0 || p.z() > cube.z()/2.0)) {
      filtered.points.push_back(p_);
    }
  }

  filtered.width = filtered.points.size();
  filtered.height = 1;
  return filtered;
}
