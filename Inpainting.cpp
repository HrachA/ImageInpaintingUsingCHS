//
//  Inpainting.cpp
//  Test
//
//  Created by Hrach Ayunts on 3/8/20.
//  Copyright © 2020 Hrach Ayunts. All rights reserved.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include "Utils.hpp"

bool finished = false;
cv::Mat img, selectedImg;
std::vector<cv::Point> vertices;

void CallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_RBUTTONDOWN) {
        std::cout << "Right mouse button clicked at (" << x << ", " << y << ")" << std::endl;
        if (vertices.size() < 2) {
            std::cout << "You need a minimum of three points!" << std::endl;
            return;
        }
//        cv::line(img, vertices[vertices.size() - 1], vertices[0], cv::Scalar(0, 0, 0));

        cv::Mat mask = img.clone();
        std::vector<std::vector<cv::Point>> pts{ vertices };
        cv::fillPoly(mask, pts, cv::Scalar(0, 0, 0));
        img.copyTo(selectedImg, mask);
        finished = true;
        return;
    }
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        cv::circle(img, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), cv::FILLED);
        std::cout << "Left mouse button clicked at (" << x << ", " << y << ")" << std::endl;
        if (vertices.size() == 0) {
            img.at<cv::Vec3b>(x, y) = cv::Vec3b(255, 0, 0);
        } else {
//            line(img, cv::Point(x, y), vertices[vertices.size() - 1], cv::Scalar(0, 0, 0));
        }
        vertices.push_back(cv::Point(x, y));
        return;
    }
}

class CubicPolynom {
public:
    CubicPolynom(float a, float b, float c, float d)
    :a(a), b(b), c(c), d(d) {}
    
    float operator()(int x) {
        return a*x*x*x + b*x*x + c*x + d;
    }
    
private:
    float a;
    float b;
    float c;
    float d;
};

void printMatrix(const cv::Mat& m) {
    for (int i = 0; i < m.rows; ++i) {
        for (int j = 0; j < m.cols; ++j) {
            std::cout << m.at<float>(i, j) << ' ';
        }
        std::cout << std::endl;
    }
}

// to do
// try derivateive approximation using image gradient
float pixelGradient(int i, int j) {
    cv::Mat gradient;
    cv::Mat greyMat(img.size(), CV_8U);
    cv::cvtColor(img, greyMat, cv::COLOR_BGR2GRAY);

    Sobel(greyMat, gradient, CV_32F, 1, 0, 5);
    float x = gradient.at<float>(i, j);
    Sobel(greyMat, gradient, CV_32F, 0, 1, 5);
    float y = gradient.at<float>(i, j);
    
    std::cout << "x/y is " << x/y << std::endl;
    std::cout <<  "y/x is " << y/x << std::endl;
    
    return -y/x;
}

void correctVertices(cv::Mat& src, cv::Mat& mask) {
    
}

CubicPolynom recoverPolynomial(std::vector<cv::Point> points) {
    
    cv::Mat A(4, 4, CV_32F);
    cv::Mat A_1(4, 4, CV_32F);
    A.at<float>(0, 0) = pow(points[1].x, 3);
    A.at<float>(0, 1) = pow(points[1].x, 2);
    A.at<float>(0, 2) = points[1].x;
    A.at<float>(0, 3) = 1;
    A.at<float>(1, 0) = pow(points[2].x, 3);
    A.at<float>(1, 1) = pow(points[2].x, 2);
    A.at<float>(1, 2) = points[2].x;
    A.at<float>(1, 3) = 1;
    A.at<float>(2, 0) = 3 * pow(points[1].x, 2);
    A.at<float>(2, 1) = 2 * points[1].x;
    A.at<float>(2, 2) = 1;
    A.at<float>(2, 3) = 0;
    A.at<float>(3, 0) = 3 * pow(points[2].x, 2);
    A.at<float>(3, 1) = 2 * points[2].x;
    A.at<float>(3, 2) = 1;
    A.at<float>(3, 3) = 0;
    
//    printMatrix(A);
    
    cv::invert(A, A_1);
//    printMatrix(A_1);
    
    cv::Mat b(4, 1, CV_32F);
    b.at<float>(0, 0) = points[1].y;
    b.at<float>(0, 1) = points[2].y;
    b.at<float>(0, 2) = 1.0 * (points[1].y - points[0].y) / (points[1].x - points[0].x);
//                         (points[2].y - points[1].y) / (points[2].x - points[1].x)) / 2;
    b.at<float>(0, 3) = 1.0 * (points[3].y - points[2].y) / (points[3].x - points[2].x);
//                         (points[1].y - points[2].y) / (points[1].x - points[2].x)) / 2;
    
    std::cout << "(0, 2) " << (float)b.at<float>(0, 2) << std::endl;
    std::cout << "(0, 3) " << (float)b.at<float>(0, 3) << std::endl;
    
//    b.at<float>(0, 2) = pixelGradient(points[1].y, points[1].x);
//    b.at<float>(0, 3) = pixelGradient(points[2].y, points[2].x);
//    printMatrix(b);
    
    cv::Mat x = A_1 * b;
//    printImageInfo(x);
    
//    printMatrix(x);
    return CubicPolynom(x.at<float>(0, 0), x.at<float>(0, 1),
                        x.at<float>(0, 2), x.at<float>(0, 3));
}

bool hasInpaintingRegions(cv::Mat& mask) {
    for (int i = 0; i < mask.rows; ++i) {
        for (int j = 0; j < mask.cols; ++j) {
            if (mask.at<uchar>(i, j) < 100)
                return true;
        }
    }
    return false;
}

bool checkAndInpaintPixel(cv::Mat& src, cv::Mat& mask, cv::Mat& outMask, int i, int j, CubicPolynom& cp) {
    if (mask.at<uchar>(i, j) > 100)
        return false;

    int count = 0;
    int sum[3] = {0};
    int t = cp(j) <= i ? 1 : -1;
    for (int i1 = i - 1; i1 <= i + 1; i1++) {
        for (int j1 = j - 1; j1 <= j + 1; j1++) {
            if (!(i1 == i && j1 == j) && mask.at<uchar>(i1, j1) > 100 && t*cp(j1) < t*i1) {
                sum[0] += src.at<cv::Vec3b>(i1, j1)[0];
                sum[1] += src.at<cv::Vec3b>(i1, j1)[1];
                sum[2] += src.at<cv::Vec3b>(i1, j1)[2];
                count++;
            }
        }
    }
    if (count == 0)
        return false;
    
    src.at<cv::Vec3b>(i, j)[0] = sum[0] / count;
    src.at<cv::Vec3b>(i, j)[1] = sum[1] / count;
    src.at<cv::Vec3b>(i, j)[2] = sum[2] / count;
    outMask.at<uchar>(i, j) = 255;
    return true;
}

cv::Mat inpaint(cv::Mat& src, cv::Mat& mask, CubicPolynom& cp) {
    cv::Mat res = src.clone();
    
    for (int x = vertices[1].x; x < vertices[2].x; x++) {
        int y = cp(x);
        src.at<cv::Vec3b>(y, x) = cv::Vec3b({0, 0, 255});
    }
    cv::imshow("Edge Reconstruction", src);
    cv::waitKey(2000);
    
    while (hasInpaintingRegions(mask)) {
        cv::Mat outMask = mask.clone();
        for (int i = 0; i < src.rows; ++i) {
            for (int j = 0; j < src.cols; ++j) {
                if (!checkAndInpaintPixel(res, mask, outMask, i, j, cp))
                    continue;
//                if (outMask.at<uchar>(i, j) < 100) {
//                    if (cp(j) <= i) {
//                        res.at<cv::Vec3b>(i, j) = src.at<cv::Vec3b>(200, 200);
//                    } else {
//                        res.at<cv::Vec3b>(i, j) = src.at<cv::Vec3b>(0, 0);
//                    }
//                    outMask.at<uchar>(i, j) = 255;
//                }
            }
        }
        mask = outMask;
        cv::imshow("Inpainting Process", res);
        cv::waitKey(25);
    }
//    for (int x = vertices[0].x; x < vertices[3].x; x++) {
//        int y = cp(x);
//        res.at<cv::Vec3b>(y, x) = cv::Vec3b({0, 0, 0});
//    }
    return res;
}


int main(int argc, const char * argv[]) {
    
    img = cv::imread("elipse_source.png");
    
    cv::namedWindow("Source Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Source Image", CallBackFunc);

    // Main loop
    while (!finished)
    {
        imshow("Source Image", img);
        cv::waitKey(50);
    }
    
    
    
    cv::Mat src = cv::imread("elipse_source.png");
    cv::Mat mask = cv::imread("elipse_mask.png");
    
    cv::Mat grayMask;
    cv::cvtColor(mask, grayMask, cv::COLOR_BGR2GRAY);
    
    cv::erode(grayMask, grayMask, cv::Mat());
//    cv::erode(grayMask, grayMask, cv::Mat());

//    printImageInfo(grayMask);
    
    cv::imwrite("elipse_mask_er.png", grayMask);
    
    correctVertices(src, grayMask);
    
    CubicPolynom cp = recoverPolynomial({vertices});
    
    cv::Mat res = inpaint(src, grayMask, cp);

    cv::imshow("Final Result", res);
    cv::imwrite("elipse_result.png", res);

    cv::waitKey(0);

    return 0;
}

