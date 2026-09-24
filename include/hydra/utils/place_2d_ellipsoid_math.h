#include <Eigen/Dense>
#include <utility>

/**
 * @file place_2d_ellipsoid_math.h
 * @brief Geometry for measuring how two 2D ellipses overlap. Each ellipse is given by a
 *        shape matrix and center, with a point x inside when (x - c)^T M (x - c) < 1.
 *        The routines trace the curve of weighted centers between the ellipses, locate a
 *        point inside both, and measure the width of the overlap across the connecting
 *        direction.
 */

namespace hydra::ellipse {

/// @brief Intersect a line with an ellipse boundary.
/// The line passes through center a along the normal m at signed offset d.
/// @param A Ellipse shape matrix.
/// @param a Ellipse center.
/// @param m Line direction/normal.
/// @param d Signed offset of the line.
/// @return The two boundary intersection points.
std::pair<Eigen::Vector2d, Eigen::Vector2d> intersectLineEllipseBase(
    const Eigen::Matrix2d& A,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& m,
    double d);

/// @brief Find the two innermost points where a transverse line through both ellipses
/// crosses their boundaries, bounding the overlap across that direction.
/// @param A First ellipse shape matrix.
/// @param a First ellipse center.
/// @param B Second ellipse shape matrix.
/// @param b Second ellipse center.
/// @param m Point on the connecting curve where the line is taken.
/// @param dm Tangent of the connecting curve at m.
/// @return The pair of bounding intersection points.
std::pair<Eigen::Vector2d, Eigen::Vector2d> computeMinTransverseIntersectionPoints(
    const Eigen::Matrix2d& A,
    const Eigen::Vector2d& a,
    const Eigen::Matrix2d& B,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& m,
    const Eigen::Vector2d& dm);

/// @brief Point on the curve of precision-weighted centers between the two ellipses.
/// At lambda 0 it equals the second center, at lambda 1 the first center.
/// @param lambda Interpolation parameter in [0, 1].
/// @return The interpolated center point.
Eigen::Vector2d computeMLambda(const Eigen::Matrix2d& A,
                                 const Eigen::Vector2d& a,
                                 const Eigen::Matrix2d& B,
                                 const Eigen::Vector2d& b,
                                 const double lambda);

/// @brief Tangent (derivative with respect to lambda) of the weighted-center curve.
/// @param lambda Interpolation parameter in [0, 1].
/// @return The tangent vector of the curve at lambda.
Eigen::Vector2d computeDmLambda(const Eigen::Matrix2d& A,
                                  const Eigen::Vector2d& a,
                                  const Eigen::Matrix2d& B,
                                  const Eigen::Vector2d& b,
                                  const double lambda);

/// @brief Parameters locating a center point that lies inside both ellipses.
struct IntersectionCenters {
  bool certified = false;  ///< True if a point inside both ellipses was confirmed.
  double lambda_A = 0.0;   ///< Curve parameter at which the point entered the first ellipse.
  double lambda_B = 0.0;   ///< Curve parameter at which the point entered the second ellipse.
};

/// @brief Search the weighted-center curve for a point inside both ellipses by bisection.
/// @param A First ellipse shape matrix.
/// @param a First ellipse center.
/// @param B Second ellipse shape matrix.
/// @param b Second ellipse center.
/// @param intersection_centers Output parameters describing the located point.
void findIntersectionCenterPoints(const Eigen::Matrix2d& A,
                                     const Eigen::Vector2d& a,
                                     const Eigen::Matrix2d& B,
                                     const Eigen::Vector2d& b,
                                     IntersectionCenters& intersection_centers);

/// @brief Measure the width of the overlap region between two ellipses, taken across the
/// connecting direction at their shared interior point.
/// @param A First ellipse shape matrix.
/// @param a First ellipse center.
/// @param B Second ellipse shape matrix.
/// @param b Second ellipse center.
/// @return The overlap width, or 0 if the ellipses do not overlap.
double getEllipsoidTransverseOverlapDistance(const Eigen::Matrix2d& A,
                                                 const Eigen::Vector2d& a,
                                                 const Eigen::Matrix2d& B,
                                                 const Eigen::Vector2d& b);

}  // namespace hydra::ellipse
