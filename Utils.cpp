//
//  Utils.cpp
//  Test
//
//  Created by Hrach Ayunts on 2/28/20.
//  Copyright © 2020 Hrach Ayunts. All rights reserved.
//

#include "Utils.hpp"

using namespace std;
using namespace cv;

std::string getImageType(int number) {
    int imgTypeInt = number % 8;
    std::string imgTypeString;

    switch (imgTypeInt) {
    case 0:
        imgTypeString = "8U";
        break;
    case 1:
        imgTypeString = "8S";
        break;
    case 2:
        imgTypeString = "16U";
        break;
    case 3:
        imgTypeString = "16S";
        break;
    case 4:
        imgTypeString = "32S";
        break;
    case 5:
        imgTypeString = "32F";
        break;
    case 6:
        imgTypeString = "64F";
        break;
    default:
        break;
    }

    // find channel
    int channel = (number / 8) + 1;

    std::stringstream type;
    type << "CV_" << imgTypeString << "C" << channel;

    return type.str();
}

void printImageInfo(Mat& image) {
    double min, max;
    minMaxLoc(image, &min, &max);

    cout << "Image width is " << image.size().width << " and height is " << image.size().height << ".\n";
    cout << "Image is in range (" << min << ", " << max << ")." << endl;
    cout << "Image type is " << getImageType(image.type()) << ".\n";
}

void show(Mat& image) {
    double min, max;
    minMaxLoc(image, &min, &max);

    namedWindow("Result", WINDOW_AUTOSIZE);
    //    imshow("Result", 255 * (image / max));
    imshow("Result", image);
    waitKey(0);
}
