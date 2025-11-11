#include <traj_utils/planning_visualization.h>

#include <chrono>

namespace fast_planner {

PlanningVisualization::PlanningVisualization(const rclcpp::Node::SharedPtr& node)
  : node_(node) {
  pubs_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>("planning_vis/trajectory", 20));
  pubs_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>("planning_vis/topo_path", 20));
  pubs_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>("planning_vis/prediction", 20));
  pubs_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>("planning_vis/visib_constraint", 20));
  pubs_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>("planning_vis/frontier", 20));
  pubs_.push_back(node_->create_publisher<visualization_msgs::msg::Marker>("planning_vis/yaw", 20));
}

void PlanningVisualization::displaySphereList(const std::vector<Eigen::Vector3d>& list,
                                              double resolution, const Eigen::Vector4d& color,
                                              int id, int pub_id) {
  if (pub_id >= pubs_.size()) {
    return;
  }

  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = node_->now();
  mk.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
  mk.action          = visualization_msgs::msg::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id]->publish(mk);

  mk.action             = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r            = color(0);
  mk.color.g            = color(1);
  mk.color.b            = color(2);
  mk.color.a            = color(3);
  mk.scale.x            = resolution;
  mk.scale.y            = resolution;
  mk.scale.z            = resolution;

  geometry_msgs::msg::Point pt;
  for (const auto& p : list) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::displayCubeList(const std::vector<Eigen::Vector3d>& list,
                                            double resolution, const Eigen::Vector4d& color,
                                            int id, int pub_id) {
  if (pub_id >= pubs_.size()) {
    return;
  }

  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = node_->now();
  mk.type            = visualization_msgs::msg::Marker::CUBE_LIST;
  mk.action          = visualization_msgs::msg::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id]->publish(mk);

  mk.action             = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r            = color(0);
  mk.color.g            = color(1);
  mk.color.b            = color(2);
  mk.color.a            = color(3);
  mk.scale.x            = resolution;
  mk.scale.y            = resolution;
  mk.scale.z            = resolution;

  geometry_msgs::msg::Point pt;
  for (const auto& p : list) {
    pt.x = p(0);
    pt.y = p(1);
    pt.z = p(2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::displayLineList(const std::vector<Eigen::Vector3d>& list1,
                                            const std::vector<Eigen::Vector3d>& list2,
                                            double line_width, const Eigen::Vector4d& color,
                                            int id, int pub_id) {
  if (pub_id >= pubs_.size()) {
    return;
  }

  visualization_msgs::msg::Marker mk;
  mk.header.frame_id = "map";
  mk.header.stamp    = node_->now();
  mk.type            = visualization_msgs::msg::Marker::LINE_LIST;
  mk.action          = visualization_msgs::msg::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id]->publish(mk);

  mk.action             = visualization_msgs::msg::Marker::ADD;
  mk.pose.orientation.w = 1.0;
  mk.color.r            = color(0);
  mk.color.g            = color(1);
  mk.color.b            = color(2);
  mk.color.a            = color(3);
  mk.scale.x            = line_width;

  geometry_msgs::msg::Point pt;
  for (size_t i = 0; i < list1.size() && i < list2.size(); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id]->publish(mk);
}

void PlanningVisualization::drawGeometricPath(const std::vector<Eigen::Vector3d>& path,
                                              double resolution, const Eigen::Vector4d& color,
                                              int id) {
  displaySphereList(path, resolution, color, id, 0);
}

void PlanningVisualization::drawBspline(NonUniformBspline& bspline, double size,
                                        const Eigen::Vector4d& color, bool show_ctrl_pts,
                                        double size2, const Eigen::Vector4d& color2,
                                        int id1, int id2) {
  std::vector<Eigen::Vector3d> point_set;
  double                       tm, tm_p;
  bspline.getTimeSpan(tm, tm_p);
  for (double t = tm; t <= tm_p + 1e-4; t += size) {
    point_set.push_back(bspline.evaluateDeBoor(t));
  }
  displaySphereList(point_set, size, color, id1, 0);

  if (show_ctrl_pts) {
    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
    std::vector<Eigen::Vector3d> ctrl_list(ctrl_pts.rows());
    for (int i = 0; i < ctrl_pts.rows(); ++i) {
      ctrl_list[i] = ctrl_pts.row(i);
    }
    displaySphereList(ctrl_list, size2, color2, id2, 0);
  }
}

void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                                const Eigen::Vector4d& color, int id) {
  poly_traj.init();
  std::vector<Eigen::Vector3d> poly_pts = poly_traj.getTraj();
  displaySphereList(poly_pts, resolution, color, id, 0);
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  std::vector<Eigen::Vector3d> goal_list = { goal };
  displaySphereList(goal_list, resolution, color, id, 0);
}

Eigen::Vector4d PlanningVisualization::getColor(double h, double alpha) {
  Eigen::Vector4d color(1.0, 1.0, 1.0, alpha);
  double          s = 1.0;
  double          v = 1.0;
  h                = std::max(0.0, std::min(h, 1.0)) * 6.0;
  int i            = static_cast<int>(std::floor(h));
  double f         = h - i;
  double p         = v * (1 - s);
  double q         = v * (1 - s * f);
  double t         = v * (1 - s * (1 - f));

  switch (i % 6) {
    case 0: color << v, t, p, alpha; break;
    case 1: color << q, v, p, alpha; break;
    case 2: color << p, v, t, alpha; break;
    case 3: color << p, q, v, alpha; break;
    case 4: color << t, p, v, alpha; break;
    case 5: color << v, p, q, alpha; break;
  }
  return color;
}

}  // namespace fast_planner
