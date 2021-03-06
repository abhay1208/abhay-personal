#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "../../Libraries/Eigen/Dense"

class Tools
{
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

    static void normalizeTheta(double &yaw);
};

#endif // TOOLS_H_