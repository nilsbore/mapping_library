#include "plane_primitive.h"

#include <iostream> // DEBUG
#include <opencv2/highgui/highgui.hpp> // DEBUG
#include <Eigen/Eigenvalues>

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

// compute the enclosing box from a set of points defining the convex hull
void plane_primitive::find_smallest_enclosing_box(Vector2d& cmin, Matrix2d& axes,
                                                  Vector2d& lengths, std::vector<cv::Point>& pts)
{
    std::vector<Vector2d, aligned_allocator<Vector2d> > dpts;
    dpts.resize(pts.size());
    for (int i = 0; i < pts.size(); ++i) {
        dpts[i] = Vector2d(double(pts[i].x), double(pts[i].y));
    }
    double areamin = INFINITY;
    for (int i = 0; i < dpts.size(); ++i) { // all lines, find smallest area
        Vector2d vec = dpts[(i+1)%int(dpts.size())] - dpts[i];
        Vector2d ovec(-vec(1), vec(0));
        vec.normalize();
        ovec.normalize();
        double widthmin = INFINITY;
        double widthmax = -INFINITY;
        double heightmax;
        for (int j = 0; j < dpts.size(); ++j) { // find width and height
            double proj = vec.dot(dpts[j] - dpts[i]);
            double oproj = ovec.dot(dpts[j] - dpts[i]);
            if (proj < widthmin) {
                widthmin = proj;
            }
            if (proj > widthmax) {
                widthmax = proj;
            }
            if (fabs(oproj) > fabs(heightmax)) {
                heightmax = oproj;
            }
        }
        double width = (widthmax - widthmin);
        double area = fabs(heightmax)*width;
        if (area < areamin) {
            areamin = area;
            axes.col(0) = vec;
            axes.col(1) = ovec;
            cmin = dpts[i] + 0.5*(widthmin*vec + widthmax*vec + heightmax*ovec);
            lengths = Vector2d(width, heightmax);
        }
    }
}

void plane_primitive::compute_shape_size(const MatrixXd& points)
{
    Vector2i pt;
    Vector2i minpt;
    minpt << INT32_MAX, INT32_MAX;
    Vector2i maxpt;
    maxpt << -INT32_MAX, -INT32_MAX;
    std::vector<Vector2i, aligned_allocator<Vector2i> > pts;
    pts.resize(supporting_inds.size());
    int counter = 0;
    for (const int& i : supporting_inds) {
        pt = (0.5/connectedness_res*basis.transpose()*points.col(i)).cast<int>();
        if (pt(0) < minpt(0)) {
            minpt(0) = pt(0);
        }
        if (pt(1) < minpt(1)) {
            minpt(1) = pt(1);
        }
        if (pt(0) > maxpt(0)) {
            maxpt(0) = pt(0);
        }
        if (pt(1) > maxpt(1)) {
            maxpt(1) = pt(1);
        }
        pts[counter] = pt;
        ++counter;
    }

    int width = 1 + maxpt(0) - minpt(0);
    int height = 1 + maxpt(1) - minpt(1);

    cv::Mat binary = cv::Mat::zeros(height, width, CV_8UC1);

    for (const Vector2i& pp : pts) {
        binary.at<unsigned char>(pp(1) - minpt(1), pp(0) - minpt(0)) = 255;
    }

    /*cv::namedWindow("Original", CV_WINDOW_AUTOSIZE);
    cv::imshow("Original", binary);
    cv::waitKey(0);*/

    //cv::Mat threshold_output;

    //int thresh = 100;
    //int max_thresh = 255;
    // Detect edges using Threshold
    //cv::threshold(binary, threshold_output, thresh, 255, cv::THRESH_BINARY);

    //cv::namedWindow("Threshold", CV_WINDOW_AUTOSIZE);
    //cv::imshow("Threshold", threshold_output);
    //cv::waitKey(0);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    // Find contours
    cv::findContours(binary, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    int maxind = 0;
    int maxval = contours[0].size();
    for (int i = 1; i < contours.size(); ++i) {
        if (contours[i].size() > maxval) {
            maxval = contours[i].size();
            maxind = i;
        }
    }

    // Find the convex hull object for the contour
    std::vector<std::vector<cv::Point> > hull;
    hull.resize(1);
    cv::convexHull(cv::Mat(contours[maxind]), hull[0], false);

    /*cv::RNG rng(12345);
    // Draw contours + hull results
    cv::Mat drawing = cv::Mat::zeros(binary.size(), CV_8UC3);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    cv::Scalar color2 = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    cv::drawContours(drawing, contours, maxind, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    cv::drawContours(drawing, hull, 0, color2, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

    // Show in a window
    cv::namedWindow("Hull demo", CV_WINDOW_AUTOSIZE);
    cv::imshow("Hull demo", drawing);
    cv::waitKey(0);*/

    Matrix2d axes;
    Vector2d lengths;
    Vector2d c2;
    find_smallest_enclosing_box(c2, axes, lengths, hull[0]);
    Vector3d c3 = 2.0*connectedness_res*basis*(minpt.cast<double>() + c2);
    c = c3 - (p(3) + c3.dot(p.head<3>()))*p.head<3>();
    sizes = 2.0*connectedness_res*lengths.array().abs();

    Matrix3d R;
    R.col(0) = p.head<3>();
    R.col(0).normalize();
    R.col(1) = basis*axes.col(0);
    R.col(1).normalize();
    R.col(2) = basis*axes.col(1);
    R.col(2).normalize();
    quat = Quaterniond(R);

    convex_hull.resize(hull[0].size());
    for (int i = 0; i < hull[0].size(); ++i) {
        Vector2i p2(hull[0][i].x, hull[0][i].y);
        Vector3d p3 = 2.0*connectedness_res*basis*(minpt + p2).cast<double>();
        convex_hull[i] = p3 - (p(3) + p3.dot(p.head<3>()))*p.head<3>();
    }
}

void plane_primitive::compute_inliers(std::vector<int>& inliers, const MatrixXd& points, const MatrixXd& normals,
                                     const std::vector<int>& inds, double inlier_threshold, double angle_threshold)
{
    Vector3d pt;
    Vector3d n;
    double cos_threshold = cos(angle_threshold);
    for (const int& i : inds) {
        pt = points.col(i);
        n = normals.col(i);
        if (fabs(pt.dot(p.segment<3>(0)) + p(3)) < inlier_threshold &&
                fabs(n.dot(p.segment<3>(0))) > cos_threshold) {
            inliers.push_back(i);
        }
    }
}

void plane_primitive::largest_connected_component(std::vector<int>& inliers, const MatrixXd& points)
{
    Vector2i pt;
    Vector2i minpt;
    minpt << INT32_MAX, INT32_MAX;
    Vector2i maxpt;
    maxpt << -INT32_MAX, -INT32_MAX;
    std::vector<Vector2i, aligned_allocator<Vector2i> > pts;
    pts.resize(conforming_inds.size());
    int counter = 0;
    for (const int& i : conforming_inds) {
        pt = (1.0/current_connectedness_res()*basis.transpose()*points.col(i)).cast<int>();
        if (pt(0) < minpt(0)) {
            minpt(0) = pt(0);
        }
        if (pt(1) < minpt(1)) {
            minpt(1) = pt(1);
        }
        if (pt(0) > maxpt(0)) {
            maxpt(0) = pt(0);
        }
        if (pt(1) > maxpt(1)) {
            maxpt(1) = pt(1);
        }
        pts[counter] = pt;
        ++counter;
    }

    int width = 1 + maxpt(0) - minpt(0);
    int height = 1 + maxpt(1) - minpt(1);

    if (width < 50 || height < 50) {
        inliers = conforming_inds;
    }

    cv::Mat binary = cv::Mat::zeros(height, width, CV_32SC1);
    //cv::Mat binary0 = cv::Mat::zeros(height, width, CV_32SC1);

    for (Vector2i& pp : pts) {
        pp -= minpt;
        binary.at<int>(pp(1), pp(0)) = 1;
        //binary0.at<int>(pp(1), pp(0)) = 65535;
    }

    //cv::imshow("Binary", binary0);
    //cv::waitKey(0);

    //cv::Mat binary2 = cv::Mat::zeros(height, width, CV_32SC1);

    inliers.reserve(pts.size());
    int largest = find_blobs(binary);
    counter = 0;
    for (const Vector2i& pp : pts) {
        if (binary.at<int>(pp(1), pp(0)) == largest) {
            inliers.push_back(conforming_inds[counter]);
            //binary2.at<int>(pp(1), pp(0)) = 65535;
        }
        ++counter;
    }

    //cv::imshow("Binary2", binary2);
    //cv::waitKey(0);
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

void plane_primitive::direction_and_center(Eigen::Vector3d& direction, Eigen::Vector3d& center)
{
    if (p(2) > 0) {
        direction = -p.segment<3>(0);
    }
    else {
        direction = p.segment<3>(0);
    }
}

double plane_primitive::shape_size()
{
    return sizes(1);//sizes(0)*sizes(1);
}

double plane_primitive::shape_data(VectorXd& data)
{
    data.resize(13);
    data.segment<4>(0) = p;
    data.segment<2>(4) = sizes;
    data(6) = c(0);
    data(7) = c(1);
    data(8) = c(2);
    data(9) = quat.x();
    data(10) = quat.y();
    data(11) = quat.z();
    data(12) = quat.w();
}

void plane_primitive::shape_points(std::vector<Vector3d, aligned_allocator<Vector3d> >& points)
{
    points = convex_hull;
}
