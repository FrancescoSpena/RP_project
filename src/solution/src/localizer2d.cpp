#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  if (_map->initialized()) {
    // Extract obstacles
    _obst_vect.clear();
    for (size_t r = 0; r < _map->rows(); ++r) {
      for (size_t c = 0; c < _map->cols(); ++c) {
        cv::Point2i p(r, c);
        if ((*_map)(p) == CellType::Occupied) {
          Eigen::Vector2f p_in_world = _map->grid2world(p);
          _obst_vect.push_back(p_in_world);
        }
      }
    }
    // Create KD-Tree
    _obst_tree_ptr =
        std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end(), 10);
  }
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  _laser_in_world = initial_pose_;
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  ContainerType prediction;
  getPrediction(prediction);
  if (!prediction.size()) return;

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  ICP solver(prediction, scan_, 10);
  solver.X() = _laser_in_world;
  solver.run(100);
  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */

  _laser_in_world = solver.X();
  std::cerr << "[Localizer2D] chi=" << solver.chi()
            << ", position= " << _laser_in_world.translation().transpose()
            << std::endl;
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  TreeType::AnswerType answers;
  _obst_tree_ptr->fullSearch(answers, _laser_in_world.translation(),
                             _range_max);

  for (size_t i = 0; i < answers.size(); ++i) {
    prediction_.push_back(*answers[i]);
  }
}