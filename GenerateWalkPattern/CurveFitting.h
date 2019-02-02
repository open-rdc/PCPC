/*
 * http://wildpie.hatenablog.com/entry/20110219/1298108898
 */

#ifndef CURVEFITTING_H
#define CURVEFITTING_H

#include <Eigen/Dense>

class CurveFitting
{
public:
    explicit CurveFitting(int M);
    void MAP(const Eigen::VectorXd& x,
             const Eigen::VectorXd& t,
             const double lambda);
    double y(const double x) const;
	Eigen::VectorXd GetCoefficient(){ return W_; }
private:
    Eigen::VectorXd W_;
    int M_;
};

#endif // CURVEFITTING_H

