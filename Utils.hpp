//
//  Utils.hpp
//  Test
//
//  Created by Hrach Ayunts on 2/28/20.
//  Copyright © 2020 Hrach Ayunts. All rights reserved.
//

#ifndef Utils_hpp
#define Utils_hpp

#include <opencv2/opencv.hpp>

std::string getImageType(int number);

void printImageInfo(cv::Mat& image);

void show(cv::Mat& image);

#endif /* Utils_hpp */
