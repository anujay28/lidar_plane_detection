# Plane Detection Module

This module contains the implementation of various functions related to plane detection and manipulation within point clouds. It includes functionality to estimate planes from point clouds, transform planes and points into different coordinate frames, and perform filtering on point clouds based on bounding boxes and cube-shaped filters.

## `plane_detection.h`

This header file defines the following functions:
### `NearPointOnPlane(const Plane& plane, const Eigen::Vector3d& point) -> Eigen::Vector3d`

Finds the nearest point on a plane to a given 3D point. The function takes the plane coefficients as a `Plane` object and the input point as a 3D vector (`Eigen::Vector3d`). It returns the 3D coordinates of the nearest point on the plane to the input point.

### `PlaneToPointNormal(const Plane& plane, const Point& point = Point{0, 0, 0})`

Converts a set of plane coefficients into point normal form. The point normal form includes a point on the plane and the normal vector of the plane. The function takes the plane coefficients as a `Plane` object and an optional `Point` object (defaulting to the origin). It returns a `PointNormal` structure containing the point on the plane and the normalized normal vector of the plane.

### `EstimatePlane(const PointCloud::Ptr& cloud, const double& thresh)`

Estimates a plane within a point cloud using the RANSAC algorithm. The function takes a pointer to a point cloud and a distance threshold for considering inliers. It performs plane segmentation using the RANSAC method and returns the estimated plane coefficients as a `Plane` object.

### `CloudFilterWindow(const PointCloud::Ptr& cloud, const Box3d& box)`

Filters a point cloud based on a given 3D bounding box. The function takes a pointer to a point cloud and a `Box3d` structure representing the bounding box. It returns a filtered point cloud containing only the points within the specified bounding box.

### `TransformBox3d(const Box3d& box, const Eigen::Matrix4d tran)`

Transforms a 3D bounding box using an affine transformation matrix. The function takes a `Box3d` bounding box and an affine transformation matrix. It applies the transformation to the eight corners of the box and returns the transformed bounding box.

### `AveragePlanes(const std::vector<Plane>& planes)`

Averages planes of differing signs, ensuring that the resulting plane will have a normal pointing towards the origin. The function takes a vector of `Plane` objects and returns an averaged plane normalized to ensure a positive d component.

### `TransformPointNormal(const PointNormal& pn, const Eigen::Isometry3d& trans)`

Transforms a `PointNormal` object (a point on a plane with a normal vector) into a different coordinate frame using an isometry transformation. The function takes a `PointNormal` object and an isometry transformation and returns the transformed `PointNormal`.

### `TransformPlane(const Plane& p, const Eigen::Isometry3d& trans)`

Transforms a plane defined by its coefficients into a different coordinate frame using an isometry transformation. The function takes a `Plane` object and an isometry transformation and returns the transformed `Plane`.

### `EstimatePlane(const std::vector<PointCloud::Ptr>& clouds, const Box3d& output_bounds, const double& thresh, const Eigen::Isometry3d& output_T_cloud, const double& d_offset)`

Estimates an averaged plane from a list of point clouds and a bounding box in a different coordinate frame. The function performs all necessary averaging and coordinate transforms to filter the point clouds within the output bounds and returns a plane in the desired output coordinate frame.

### `CloudTransformedBounds(const PointCloud& cloud, const Eigen::Isometry3d& T) -> std::tuple<double, double, double, double, double, double>`

Computes the bounds of a point cloud in a different reference frame. The resulting bounds are with respect to the reference frame `T`. The function takes a point cloud and an isometry transformation and returns the bounds as a tuple.

### `PCL2PointCloud(const sensor_msgs::PointCloud2& ros_cloud)`

Converts a ROS `sensor_msgs::PointCloud2` message into a `PointCloud<PointXYZ>`. The function takes a ROS message and returns a PCL point cloud.

### `CubeFilter(const PointCloud& cloud, const Eigen::Vector3d& cube, const Eigen::Isometry3d& trans=Eigen::Isometry3d::Identity()) -> PointCloud`

Applies a cube-style filter to a point cloud. Points are transformed into the cube's frame of reference, and points within the cube are retained. The function takes a point cloud, cube dimensions, and an optional transformation matrix, and returns a filtered point cloud.
