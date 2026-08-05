#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "Usage: load_calibration <stereo_calibration.yaml> <rectify_maps.yml.gz> <sbs_image>\n";
    return 2;
  }

  cv::FileStorage calibration(argv[1], cv::FileStorage::READ);
  cv::FileStorage maps(argv[2], cv::FileStorage::READ);
  if (!calibration.isOpened() || !maps.isOpened()) {
    std::cerr << "Failed to open calibration or maps file\n";
    return 1;
  }

  int width = 0;
  int height = 0;
  calibration["image_width"] >> width;
  calibration["image_height"] >> height;
  cv::Mat left_map1, left_map2, right_map1, right_map2;
  maps["left_map1"] >> left_map1;
  maps["left_map2"] >> left_map2;
  maps["right_map1"] >> right_map1;
  maps["right_map2"] >> right_map2;

  const cv::Mat sbs = cv::imread(argv[3], cv::IMREAD_COLOR);
  if (sbs.empty() || sbs.cols != width * 2 || sbs.rows != height) {
    std::cerr << "SBS image size does not match calibration: expected "
              << width * 2 << "x" << height << "\n";
    return 1;
  }

  const cv::Mat left = sbs(cv::Rect(0, 0, width, height));
  const cv::Mat right = sbs(cv::Rect(width, 0, width, height));
  cv::Mat left_rectified, right_rectified;
  cv::remap(left, left_rectified, left_map1, left_map2, cv::INTER_LINEAR);
  cv::remap(right, right_rectified, right_map1, right_map2, cv::INTER_LINEAR);
  cv::Mat preview;
  cv::hconcat(left_rectified, right_rectified, preview);
  for (int y = 0; y < preview.rows; y += 40) {
    cv::line(preview, cv::Point(0, y), cv::Point(preview.cols - 1, y), cv::Scalar(0, 255, 0));
  }
  if (!cv::imwrite("rectified_cpp.png", preview)) {
    std::cerr << "Failed to write rectified_cpp.png\n";
    return 1;
  }
  std::cout << "Wrote rectified_cpp.png\n";
  return 0;
}

