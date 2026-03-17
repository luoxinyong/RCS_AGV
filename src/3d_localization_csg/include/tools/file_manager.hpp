#ifndef SLAM_AD_TOOLS_FILE_MANAGER_HPP_
#define SLAM_AD_TOOLS_FILE_MANAGER_HPP_

#include <fstream>
#include <iostream>
#include <string>

namespace h_x {
class FileManager {
 public:
  static bool CreateFile(std::ofstream& ofs, std::string file_path);
  static bool InitDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path, std::string use_for);
  static bool CreateDirectory(std::string directory_path);
};
}  // namespace h_x

#endif
