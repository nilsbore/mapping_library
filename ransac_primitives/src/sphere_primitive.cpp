#include "sphere_primitive.h"

#include <iostream> // DEBUG
#include <Eigen/LU>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp> // DEBUG

using namespace Eigen;

int sphere_primitive::spheres_drawn = 0;

sphere_primitive::sphere_primitive()
{
    max_radius = 0.1;
}

int sphere_primitive::points_required()
{
    return 2;
}

bool sphere_primitive::construct(const MatrixXd& points, const MatrixXd& normals,
                                double inlier_threshold, double angle_threshold)
{
    Vector3d p1 = points.col(0);
    Vector3d p2 = points.col(1);
    Vector3d n1 = normals.col(0);
    Vector3d n2 = normals.col(1);

    /*Matrix2d A;
    A(0, 0) = n1.dot(n1);
    A(0, 1) = n1.dot(n2);
    A(1, 0) = -A(0, 1);
    A(1, 1) = n2.dot(n2);
    Vector2d b;
    b(0) = n1.dot(p1 - p2);
    b(1) = n2.dot(p1 - p2);

    Vector2d x;
    //x = A.inverse()*b;
    x = A.lu().solve(b);*/

    // Try 2
    /*Vector3d R = (p2 - p1).cross(n1.cross(n2));
    Vector2d x;
    x(0) = R.dot(n2);
    x(1) = R.dot(n1);

    c = 0.5*(p1 + x(0)*n1 + p2 + x(1)*n2);
    r = 0.5*((p1 - c).norm() + (p2 - c).norm());*/

    // Try 3
    Vector3d n = p1 - p2;
    n.normalize();
    double d = -n.dot(0.5*(p1 + p2));
    double t1 = -(d + n.dot(p1))/n.dot(n1);
    double t2 = -(d + n.dot(p2))/n.dot(n2);

    c = 0.5*(p1 + t1*n1 + p2 + t2*n2);
    r = 0.5*((p1 - c).norm() + (p2 - c).norm());

    /*std::cout << "Radius: " << r << std::endl;
    std::cout << "C1: " << (p1 + t1*n1).transpose() << std::endl;
    std::cout << "C2: " << (p2 + t2*n2).transpose() << std::endl;*/

    // Try 4, without normals
    /*Matrix3d N;
    N.row(0) = p1 - p2;
    N.row(0).normalize();
    N.row(1) = p3 - p1;
    N.row(1).normalize();
    N.row(2) = p2 - p3;
    N.row(2).normalize();
    Vector3d d;
    d(0) = -N.row(0)*(0.5*(p1 + p2));
    d(1) = -N.row(1)*(0.5*(p3 + p1));
    d(2) = -N.row(2)*(0.5*(p2 + p3));
    c = -N.lu().solve(d);
    r = 1.0/3.0*((p1 - c).norm() + (p2 - c).norm() + (p3 - c).norm());*/

    if (isnan(r) || fabs(r) < 0.02 || r > max_radius) {
        return false;
    }

    /*if (isinf(x.sum()) || isnan(x.sum())) {
        return false;
    }*/

    Vector3d rad;
    double radnorm = rad.norm();
    for (int i = 0; i < 2; ++i) {
        rad = points.col(i) - c;
        if (fabs(rad.norm() - r) > inlier_threshold || acos(fabs(normals.col(i).dot(rad/radnorm))) > angle_threshold) {
            //std::cout << "Proximity: " << fabs((rad).norm() - r) << std::endl;
            //std::cout << "Angle: " << acos(fabs(normals.col(i).dot(rad))) << std::endl;
            /*if (isnan(acos(fabs(normals.col(i).dot(rad))))) {
                std::cout << "Angle NAN!" << std::endl;
                //exit(0);
            }*/
            return false;
        }
    }

    //std::cout << "Succeded" << std::endl;
    //std::cout << "c: " << c.transpose() << std::endl;
    //std::cout << "r: " << r << std::endl;

    return true;
}

int sphere_primitive::inliers(const MatrixXd& points, const MatrixXd& normals, const std::vector<int>& inds,
                             double inlier_threshold, double angle_threshold)
{
    Vector2d min2;
    min2 << -r*M_PI/2.0, -r*M_PI/2.0;
    Vector2d max2;
    max2 << r*M_PI/2.0, r*M_PI/2.0;

    // check for inliers to primitive
    Vector3d pt;
    Vector3d n;
    Vector3d rad;
    double rad_norm;
    std::vector<int> temp;
    std::vector<Vector3d, aligned_allocator<Eigen::Vector3d> > sphere_pts;
    double cos_threshold = cos(angle_threshold);
    for (const int& i : inds) {
        pt = points.col(i);
        n = normals.col(i);
        rad = pt - c;
        rad_norm = rad.norm();
        rad *= 1.0/rad_norm;
        if (fabs(n.dot(rad)) > cos_threshold &&
                fabs(rad_norm - r) < inlier_threshold) {
            //std::cout << "Inlier!" << std::endl;
            sphere_pts.push_back(rad);
            temp.push_back(i);
        }
    }

    if (temp.size() < min_inliers) {
        return 0;
    }

    Vector2d size2 = max2 - min2;
    int width = int(ceil(size2(0)/connectedness_res));
    int height = int(ceil(size2(1)/connectedness_res));
    cv::Mat binary = cv::Mat::zeros(2*height, width, CV_32SC1);
    cv::Mat binary1 = cv::Mat::zeros(2*height, width, CV_32SC1);

    // check for largest connected components
    std::vector<Vector2i, aligned_allocator<Eigen::Vector2i> > sphere_ptsi;
    sphere_ptsi.resize(sphere_pts.size());

    int counter = 0;
    Vector2d pt2;
    Vector2i pt2i;
    /*int minx = 10000;
    int miny = 10000;
    int maxx = -1;
    int maxy = -1;*/
    for (const Vector3d& pt3 : sphere_pts) {
        bool isupper = sphere_to_grid(pt2, pt3);
        pt2i(0) = int((pt2(0) - min2(0))/connectedness_res);
        pt2i(1) = int((pt2(1) - min2(1))/connectedness_res);
        if (isupper) {
            pt2i(1) += height;
        }

        /*if (pt2i(0) < minx) {
            minx = pt2i(0);
        }
        if (pt2i(1) < miny) {
            miny = pt2i(1);
        }
        if (pt2i(0) > maxx) {
            maxx = pt2i(0);
        }
        if (pt2i(1) > maxy) {
            maxy = pt2i(1);
        }*/

        //std::cout << pt2i.transpose() << std::endl;
        //std::cout << width << ", " << 2*height << std::endl;
        binary.at<int>(pt2i(1), pt2i(0)) = 1;
        binary1.at<int>(pt2i(1), pt2i(0)) = 65535;
        sphere_ptsi[counter] = pt2i;
        ++counter;
    }

    /*if (maxx >= width || maxy >= 2*height || minx < 0 || miny < 0) {
        std::cout << "width: " << width << std::endl;
        std::cout << "height: " << height << std::endl;
        std::cout << "maxx: " << maxx << std::endl;
        std::cout << "minx: " << minx << std::endl;
        std::cout << "maxy: " << maxy << std::endl;
        std::cout << "miny: " << miny << std::endl;
        exit(0);
    }*/

    //cv::imshow("Binary1", binary1);
    //cv::waitKey(0);
    cv::Mat binary2 = cv::Mat::zeros(2*height, width, CV_32SC1);

    supporting_inds.reserve(temp.size());
    int largest = find_blobs(binary, true, true);
    counter = 0;
    for (const Vector2i& pp : sphere_ptsi) {
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

base_primitive::shape sphere_primitive::get_shape()
{
    return SPHERE;
}

base_primitive* sphere_primitive::instantiate()
{
    return new sphere_primitive();
}

void sphere_primitive::draw(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    if (false) {
        std::cout << "Radius: " << r << std::endl;
        exit(0);
    }
    pcl::PointXYZ p(c(0), c(1), c(2));
    viewer->addSphere<pcl::PointXYZ>(p, r, 0.0, 1.0, 0.0, std::string("sphere") + boost::lexical_cast<std::string>(spheres_drawn));
    ++spheres_drawn;
}

bool sphere_primitive::sphere_to_grid(Vector2d& gridpt, const Vector3d& spherept)
{
    Vector3d pt = spherept;
    bool isupper;
    if (pt(2) > 0) {
        isupper = true;
    }
    else {
        isupper = false;
        pt(1) *= -1.0; // fold it around
        pt(2) *= -1.0;
    }
    Vector2d circlept;
    double angle = acos(fabs(spherept(2))); // max 1
    circlept = angle/(M_PI/2.0)*spherept.segment<2>(0);
    circle_to_grid(gridpt, circlept);
    gridpt.array() -= 0.5; // change to [-1, 1] instead of [0, 1]
    gridpt *= 2.0;
    gridpt *= r*M_PI/2.0;
    return isupper;
}

double sphere_primitive::distance_to_pt(const Vector3d& pt)
{
    return fabs((pt - c).norm() - r);
}

void sphere_primitive::direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center)
{
    direction.setZero();
    center = c;
}

double sphere_primitive::shape_size()
{
    return r;
}

double sphere_primitive::shape_data(VectorXd& data)
{
    data.resize(4);
    data.segment<3>(0) = c;
    data(3) = r;
}

void sphere_primitive::shape_points(std::vector<Vector3d, aligned_allocator<Vector3d> >& points)
{
    points.clear();
}
