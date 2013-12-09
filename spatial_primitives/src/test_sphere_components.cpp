#include <iostream>
#include "sphere_primitive.h"
#include <Eigen/Dense>

using namespace Eigen;

int main(int argc, char** argv)
{
    MatrixXd tpoints(3, 2);
    tpoints.col(0) << 1.0, 0.0, 0.0;
    tpoints.col(1) << 0.0, 0.0, 1.0;
    MatrixXd tnormals(3, 2);
    tnormals.col(0) << 1.0, 0.0, 0.0;
    tnormals.col(1) << 0.0, 0.0, 1.0;
    tpoints *= 0.1;

    sphere_primitive p;
    p.construct(tpoints, tnormals, 0.1, 0.1);

    int sz = 1000;
    MatrixXd points(3, sz);
    MatrixXd normals(3, sz);

    std::vector<int> inds;
    inds.reserve(sz);
    int counter = 0;
    for (double ang = -M_PI; ang < M_PI; ang += 2.0*M_PI/double(sz)) {
        if (counter >= sz) {
            std::cout << "Breaking, ang: " << ang << std::endl;
            std::cout << "Counter: " << counter << std::endl;
            break;
        }
        points.col(counter) = Vector3d(0.0, sin(ang), cos(ang));
        normals.col(counter) = points.col(counter);
        points.col(counter) *= 0.1;
        inds.push_back(counter);
        ++counter;
    }

    p.inliers(points, normals, inds, 0.1, 0.1);

    return 0;
}
