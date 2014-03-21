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

/*void plane_primitive::compute_shape_size(const MatrixXd& points)
{
    int n = supporting_inds.size();
    MatrixXd A(n, 4);
    int counter = 0;
    for (const int& i : supporting_inds) {
        A.block<1, 3>(counter, 0) = points.col(i).transpose();
        A(counter, 3) = 1;
        ++counter;
    }
    JacobiSVD<MatrixXd> svd(A, ComputeThinV);
    std::cout << svd.matrixV() << std::endl;
    A.block(0, 0, n, 3) *= svd.matrixV().block<3, 3>(0, 0);

    MatrixXd RR = svd.matrixV().block<3, 3>(1, 1);
    RR.col(0).normalize();
    RR.col(1).normalize();
    RR.col(2).normalize();
    std::cout << "MATRIX:\n" << RR.transpose()*RR << std::endl;

    RowVector4d maxc = A.colwise().maxCoeff();
    RowVector4d minc = A.colwise().minCoeff();

    sizes(0) = fabs(maxc(1) - minc(1));
    sizes(1) = fabs(maxc(2) - minc(2));
    //maxc(0) = minc(0) = 0.5*(maxc(0) + minc(0));

    // compute the center point of the plane
    Vector3d maxo = svd.matrixV().block<3, 3>(0, 0)*maxc.segment<3>(0).transpose();
    Vector3d mino = svd.matrixV().block<3, 3>(0, 0)*minc.segment<3>(0).transpose();
    std::cout << maxo.transpose() << std::endl;
    std::cout << mino.transpose() << std::endl;
    c = 0.5*(maxo + mino);

    std::cout << "V rows: " << svd.matrixV().rows() << ", cols: " << svd.matrixV().cols() << std::endl;

    // compute a rotation matrix with normal and largest directions
    Matrix3d R;
    R.col(0) = p.segment<3>(0);
    R.col(0).normalize();
    Vector3d v = svd.matrixV().col(1);
    Vector3d a = v - v.dot(R.col(0))*R.col(0);
    a.normalize();
    R.col(1) = a;
    R.col(2) = R.col(0).cross(a);
    R.col(2).normalize();
    quat = Quaterniond(R);
    quat.normalize();
    //quat = Quaterniond(svd.matrixV().block<3, 3>(0, 0));
}*/

/*void plane_primitive::compute_shape_size(const MatrixXd& points)
{
    // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
    Eigen::Matrix<double, 9, 1> accu = Eigen::Matrix<double, 9, 1>::Zero();
    size_t point_count;
    point_count = supporting_inds.size();
    Matrix3d A(3, point_count);
    Vector3d pt;
    int counter = 0;
    for (const int& ind : supporting_inds) {
        //const PointT& point = cloud[*iIt];
        pt = points.col(ind);
        accu.segment<3>(0) += pt(0)*pt;
        accu.segment<2>(3) += pt(1)*pt.segment<2>(1);
        accu(5) += pt(2)*pt(2);
        accu.segment<3>(6) += pt;
        A.col(counter) = pt;
    }

    std::cout << "accu:\n" << accu << std::endl;

    accu /= double(point_count);
    Eigen::Vector3d centroid;
    centroid(0) = accu(6); centroid(1) = accu(7); centroid(2) = accu(8);
    Eigen::Matrix3d covariance_matrix;
    covariance_matrix.col(0) = accu.segment<3>(0) - accu(6)*accu.segment<3>(6);
    covariance_matrix.block<2, 1>(1, 1) = accu.segment<2>(3) - accu(7)*accu.segment<2>(7);
    covariance_matrix(2, 2) = accu(5) - accu(8)*accu(8);
    covariance_matrix(0, 1) = covariance_matrix(1, 0);
    covariance_matrix(0, 2) = covariance_matrix(2, 0);
    covariance_matrix(1, 2) = covariance_matrix(2, 1);

    JacobiSVD<Matrix3d> svd(covariance_matrix, ComputeThinV);
    EigenSolver<Matrix3d> evd(covariance_matrix, true);

    Vector3d V = evd.eigenvalues().real();
    Vector3i I(1, 2, 3);

    // simple sort of the eigenvalues
    for (int i = 1; i < 3; ++i) {
        for (int j = i; j > 0; --j) {
            if (V(j) < V(j-1)) {
                break;
            }
            double tempval = V(j-1);
            V(j-1) = V(j);
            V(j) = tempval;
            int tempind = I(j-1);
            I(j-1) = I(j);
            I(j) = tempind;
        }
    }

    std::cout << "Eigen values:\n" << V << std::endl;*/

    // compute orientation of plane
    /*Matrix3d R;
    R.col(0) = p.segment<3>(0);
    R.col(0).normalize();
    Vector3d v = evd.eigenvectors().col(I(0)).real();
    Vector3d a = v - v.dot(R.col(0))*R.col(0);
    a.normalize();
    R.col(1) = a;
    R.col(2) = R.col(0).cross(a);
    R.col(2).normalize();
    quat = Quaterniond(Matrix3d(evd.eigenvalues().real()));*/
    /*Matrix3d R;
    R.col(0) = svd.matrixV().col(1);
    R.col(1) = svd.matrixV().col(2);
    R.col(2) = svd.matrixV().col(0);
    quat = Quaterniond(R);

    A = R.transpose()*A;

    Vector3d maxc = A.rowwise().maxCoeff();
    Vector3d minc = A.rowwise().minCoeff();

    //sizes(0) = fabs(maxc(0) - minc(0));
    //sizes(1) = fabs(maxc(1) - minc(1));
    //maxc(0) = minc(0) = 0.5*(maxc(0) + minc(0));

    sizes(0) = 1.0;
    sizes(1) = 1.0;

    // compute the center point of the plane
    Vector3d maxo = R*maxc;
    Vector3d mino = R*minc;
    std::cout << maxo.transpose() << std::endl;
    std::cout << mino.transpose() << std::endl;
    //c = 0.5*(maxo + mino);
    c = centroid;

    std::cout << "U rows: " << evd.eigenvectors().rows() << ", cols: " << evd.eigenvectors().cols() << std::endl;
}*/

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

    // legacy
    //sizes(0) = 1.0;
    //sizes(1) = 1.0;

    //c.setZero();

    //quat.setIdentity();
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
    std::vector<Vector2d, aligned_allocator<Vector2d> > plane_pts;
    double cos_threshold = cos(angle_threshold);
    // a more robust and elegant solution would
    // be to discretize before finding min and max
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

    std::vector<Vector2i, aligned_allocator<Vector2i> > plane_ptsi;
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
