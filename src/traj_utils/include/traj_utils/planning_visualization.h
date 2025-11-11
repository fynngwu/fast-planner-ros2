#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <bspline/non_uniform_bspline.h>
#include <poly_traj/polynomial_traj.h>
#include <memory>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

namespace fast_planner {

class PlanningVisualization {
public:
  using Ptr = std::shared_ptr<PlanningVisualization>;
  explicit PlanningVisualization(const rclcpp::Node::SharedPtr& node);
  ~PlanningVisualization() = default;

  void displaySphereList(const std::vector<Eigen::Vector3d>& list, double resolution,
                         const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayCubeList(const std::vector<Eigen::Vector3d>& list, double resolution,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);
  void displayLineList(const std::vector<Eigen::Vector3d>& list1,
                       const std::vector<Eigen::Vector3d>& list2, double line_width,
                       const Eigen::Vector4d& color, int id, int pub_id = 0);

  void drawGeometricPath(const std::vector<Eigen::Vector3d>& path, double resolution,
                         const Eigen::Vector4d& color, int id = 0);
  void drawBspline(NonUniformBspline& bspline, double size, const Eigen::Vector4d& color,
                   bool show_ctrl_pts = false, double size2 = 0.1,
                   const Eigen::Vector4d& color2 = Eigen::Vector4d(1, 1, 0, 1), int id1 = 0,
                   int id2 = 0);
  void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d& color,
                           int id = 0);
  void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d& color, int id = 0);

  Eigen::Vector4d getColor(double h, double alpha = 1.0);

private:
  rclcpp::Node::SharedPtr node_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr> pubs_;
};

}  // namespace fast_planner

#endif
