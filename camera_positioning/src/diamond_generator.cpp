#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>

#include <string>

int main(int /*argc*/, char** /*argv*/)
{
    cv::Mat diamondImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawCharucoDiamond(dictionary, cv::Vec4i(45, 68, 28, 74), 350, 296, diamondImage); // Original 150 instead of 212
    cv::imwrite("DiamondImage.jpg", diamondImage);

    return 0;
}
