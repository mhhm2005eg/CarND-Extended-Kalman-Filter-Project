#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"
using namespace std;

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, 
                                const std::vector<Eigen::VectorXd> &ground_truth);

  Eigen::VectorXd  CalculateRMSE(const Eigen::VectorXd &estimations, 
                                const Eigen::VectorXd &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);
  
  Eigen::VectorXd PolarToCartesian(const Eigen::VectorXd& polar);
  
  Eigen::VectorXd CartesianToPolar(const Eigen::VectorXd& cartesian);

};

#endif  // TOOLS_H_
