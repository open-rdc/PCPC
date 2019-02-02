#include "CurveFitting.h"
#include <iostream>
using namespace Eigen;

CurveFitting::CurveFitting(int M) : M_(M)
{
}

void CurveFitting::MAP(const VectorXd& x,
                       const VectorXd& t,
                       const double lambda)
{
    MatrixXd A(M_+1, M_+1);
    for (int i = 0; i < M_+1; ++i) {
        for (int j = 0; j < M_+1; ++j) {
            double temp = x.array().pow(i+j).sum(); // A_ij
            if (i == j) {
                temp += lambda;
            }
            A(i, j) = temp;
        }
    }

    VectorXd T(M_+1);
    for (int i = 0; i < M_+1; ++i) {
        T(i) = (x.array().pow(i) * t.array()).sum();
    }

    W_ = A.colPivHouseholderQr().solve(T);
}

double CurveFitting::y(const double x) const
{
    double result = W_(0);
    for (int i = 1; i < M_+1; ++i) {
        result += W_(i) * pow(x, i);
    }
    return result;
}

