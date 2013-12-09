#include "plane_primitive.h"

#include <iostream> // DEBUG
#include <opencv2/highgui/highgui.hpp> // DEBUG

using namespace Eigen;

plane_primitive::plane_primitive()
{
}

int plane_primitive::points_required()
{
    return 3;
}

bool plane_primitive::construct(const MatrixXd& points, const MatrixXd& normals,
                                double inlier_threshold, double angle_threshold)
{
    Vector3d first = points.col(1) - points.col(0);
    Vector3d second = points.col(2) - points.col(0);
    Vector3d normal = first.cross(second);
    normal.normalize();
    p.segment<3>(0) = normal;
    p(3) = -normal.dot(points.col(0));

    //std::cout << "Constructing plane: " << p.transpose() << std::endl;

    if (isinf(p.sum()) || isnan(p.sum())) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        if (acos(fabs(normal.dot(normals.col(i)))) > angle_threshold) {
            //std::cout << "Threshold: " << angle_threshold << std::endl;
            //std::cout << "Angle: " << acos(fabs(normal.dot(normals.col(i)))) << std::endl;
            return false;
        }
    }
    basis.col(0) = first;
    basis.col(0).normalize();
    basis.col(1) = normal.cross(basis.col(0));
    basis.col(1).normalize();

    return true;
}

int plane_primitive::inliers(const MatrixXd& points, const MatrixXd& normals, const std::vector<int>& inds,
                             double inlier_threshold, double angle_threshold)
{
    Vector2d min2;
    min2 << INFINITY, INFINITY;
    Vector2d max2;
    max2 << -INFINITY, -INFINITY;

    Vector2d pt2;
    Vector3d pt;
    Vector3d n;
    std::vector<int> temp;
    std::vector<Vector2d, aligned_allocator<Eigen::Vector2d> > plane_pts;
    double cos_threshold = cos(angle_threshold);
    for (const int& i : inds) {
        pt = points.col(i);
        n = normals.col(i);
        if (fabs(pt.dot(p.segment<3>(0)) + p(3)) < inlier_threshold &&
                fabs(n.dot(p.segment<3>(0))) > cos_threshold) {
            //std::cout << "Inlier!" << std::endl;
            pt2 = basis.transpose()*pt;
            if (pt2(0) < min2(0)) {
                min2(0) = pt2(0);
            }
            if (pt2(1) < min2(1)) {
                min2(1) = pt2(1);
            }
            if (pt2(0) > max2(0)) {
                max2(0) = pt2(0);
            }
            if (pt2(1) > max2(1)) {
                max2(1) = pt2(1);
            }
            plane_pts.push_back(pt2);
            temp.push_back(i);
        }
        else {
            /*std::cout << "No inlier!" << std::endl;
            std::cout << "1: " << pt.dot(p.segment<3>(0)) + p(3) << std::endl;
            std::cout << "2: " << acos(fabs(n.dot(p.segment<3>(0)))) << std::endl;
            std::cout << "Point: " << pt.transpose() << std::endl;
            std::cout << "Normal: " << n.transpose() << std::endl;
            std::cout << "Plane: " << p.transpose() << std::endl;*/
        }
    }

    if (temp.size() < min_inliers) {
        return 0;
    }

    max2.array() += connectedness_res/2.0;
    min2.array() -= connectedness_res/2.0;
    Vector2d size2 = max2 - min2;
    int width = int(ceil(size2(0)/connectedness_res));
    int height = int(ceil(size2(1)/connectedness_res));
    cv::Mat binary = cv::Mat::zeros(height, width, CV_32SC1);

    std::vector<Vector2i, aligned_allocator<Eigen::Vector2i> > plane_ptsi;
    plane_ptsi.resize(plane_pts.size());

    int counter = 0;
    Vector2i pt2i;
    for (const Vector2d& pp : plane_pts) {
        pt2i(0) = int((pp(0) - min2(0))/connectedness_res);
        pt2i(1) = int((pp(1) - min2(1))/connectedness_res);
        binary.at<int>(pt2i(1), pt2i(0)) = 1;
        plane_ptsi[counter] = pt2i;
        ++counter;
    }

    //cv::imshow("Binary", binary);
    //cv::waitKey(0);

    cv::Mat binary2 = cv::Mat::zeros(height, width, CV_32SC1);

    supporting_inds.reserve(temp.size());
    int largest = find_blobs(binary);
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

base_primitive::shape plane_primitive::get_shape()
{
    return PLANE;
}

base_primitive* plane_primitive::instantiate()
{
    return new plane_primitive();
}

void plane_primitive::draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{

}

double plane_primitive::distance_to_pt(const Vector3d& pt)
{
    return fabs(pt.dot(p.segment<3>(0)) + p(3));
}
