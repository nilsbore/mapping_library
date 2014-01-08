#include "cylinder_primitive.h"

#include <iostream> // DEBUG
#include <algorithm>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> // DEBUG
#include <boost/lexical_cast.hpp>

using namespace Eigen;

int cylinder_primitive::cylinders_drawn = 0;

cylinder_primitive::cylinder_primitive()
{
    max_radius = 0.25;
}

int cylinder_primitive::points_required()
{
    return 3;
}

bool cylinder_primitive::construct(const MatrixXd& points, const MatrixXd& normals,
                                double inlier_threshold, double angle_threshold)
{
    Vector3d p1 = points.col(0);
    Vector3d p2 = points.col(1);
    Vector3d n1 = normals.col(0);
    Vector3d n2 = normals.col(1);
    a = n1.cross(n2);
    a.normalize();

    basis.col(0) = a.cross(n1);
    basis.col(0).normalize();
    basis.col(1) = a.cross(basis.col(0));

    Matrix2d A;
    A.col(0) = -basis.transpose()*n1;
    A.col(1) = basis.transpose()*n2;

    Vector2d b = basis.transpose()*(p1 - p2);

    Vector2d t = A.inverse()*b;

    Vector3d c1 = p1 + t(0)*n1;
    Vector3d c2 = p2 + t(1)*n2;

    c = (c1 + c2)/2.0;

    r = (fabs(t(0)) + fabs(t(1)))/2.0;

    if (r > max_radius || r < 0.05) {
        return false;
    }

    //std::cout << "c1: " << c1.transpose() << std::endl;
    //std::cout << "c2: " << c2.transpose() << std::endl;
    //std::cout << "r: " << r << std::endl;

    Vector2d pt2;
    double sin_threshold = sin(angle_threshold);
    for (int i = 0; i < 3; ++i) {
        pt2 = basis.transpose()*(points.col(i) - c);
        if (fabs(pt2.norm() - r) > inlier_threshold || fabs(a.dot(normals.col(i))) > sin_threshold) {
            //std::cout << "Distance: " << fabs(pt2.norm() - r) << std::endl;
            return false;
        }
    }

    //exit(0);

    return true;
}

int cylinder_primitive::inliers(const MatrixXd& points, const MatrixXd& normals, const std::vector<int>& inds,
                                double inlier_threshold, double angle_threshold)
{
    min2 << INFINITY, -r*M_PI;
    max2 << -INFINITY, r*M_PI;

    Vector3d pt;
    Vector3d n;
    Vector2d rad;
    Vector2d pt2;
    std::vector<int> temp;
    std::vector<Vector2d, aligned_allocator<Eigen::Vector2d> > plane_pts;
    for (const int& i : inds) {
        pt = points.col(i);
        n = normals.col(i);
        rad = basis.transpose()*(pt - c);
        if (fabs(rad.norm() - r) < inlier_threshold && M_PI/2.0 - acos(fabs(a.dot(n))) < angle_threshold) {
            pt2(0) = a.dot(pt - c);
            pt2(1) = r*atan2(rad(0), rad(1));
            if (pt2(0) < min2(0)) {
                min2(0) = pt2(0);
            }
            if (pt2(0) > max2(0)) {
                max2(0) = pt2(0);
            }
            temp.push_back(i);
            plane_pts.push_back(pt2);
        }
    }

    if (temp.size() < min_inliers) {
        return 0;
    }

    max2(0) += connectedness_res/2.0;
    min2(0) -= connectedness_res/2.0;
    Vector2d size2 = max2 - min2;
    int width = int(ceil(size2(0)/connectedness_res));
    int height = int(ceil(size2(1)/connectedness_res));
    cv::Mat binary = cv::Mat::zeros(height, width, CV_32SC1);
    cv::Mat binary1 = cv::Mat::zeros(height, width, CV_32SC1);

    std::vector<Vector2i, aligned_allocator<Eigen::Vector2i> > plane_ptsi;
    plane_ptsi.resize(plane_pts.size());

    int counter = 0;
    Vector2i pt2i;
    for (const Vector2d& pp : plane_pts) {
        pt2i(0) = int((pp(0) - min2(0))/connectedness_res);
        pt2i(1) = int((pp(1) - min2(1))/connectedness_res);
        binary.at<int>(pt2i(1), pt2i(0)) = 1;
        binary1.at<int>(pt2i(1), pt2i(0)) = 65535;
        plane_ptsi[counter] = pt2i;
        ++counter;
    }

    //cv::imshow("Binary1", binary1);
    //cv::waitKey(0);

    cv::Mat binary2 = cv::Mat::zeros(height, width, CV_32SC1);

    supporting_inds.reserve(temp.size());
    int largest = find_blobs(binary, true);
    counter = 0;
    for (const Vector2i& pp : plane_ptsi) {
        if (binary.at<int>(pp(1), pp(0)) == largest) {
            supporting_inds.push_back(temp[counter]);
            binary2.at<int>(pp(1), pp(0)) = 65535;
        }
        ++counter;
    }

    //cv::imshow("Binary2", binary2);
    //cv::waitKey(0);

    std::sort(supporting_inds.begin(), supporting_inds.end());
    return get_inliers();
}

base_primitive::shape cylinder_primitive::get_shape()
{
    return CYLINDER;
}

base_primitive* cylinder_primitive::instantiate()
{
    return new cylinder_primitive();
}

void cylinder_primitive::draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    return; // don't draw cylinders for now
    pcl::ModelCoefficients mc;
    Vector3d tc = c + min2(0)*a;
    Vector3d ta = c + max2(0)*a - tc;
    mc.values.resize(7);
    mc.values[0] = tc(0);
    mc.values[1] = tc(1);
    mc.values[2] = tc(2);
    mc.values[3] = ta(0);
    mc.values[4] = ta(1);
    mc.values[5] = ta(2);
    mc.values[6] = r;
    viewer->addCylinder(mc, std::string("cylinder") + boost::lexical_cast<std::string>(cylinders_drawn));
    ++cylinders_drawn;
}

double cylinder_primitive::distance_to_pt(const Vector3d& pt)
{
    return fabs((basis.transpose()*(pt - c)).norm() - r);
}

void cylinder_primitive::direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center)
{
    if (a(2) > 0) {
        direction = -a;
    }
    else {
        direction = a;
    }
}

double cylinder_primitive::shape_size()
{
    return r;
}
