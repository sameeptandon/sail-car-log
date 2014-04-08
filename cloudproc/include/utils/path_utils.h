#pragma once

#include <vector>
#include <string>

template <typename T>
T str2num ( const std::string &text );

// Gets files in directory ordered by number
// regex_str match 1 should extract number, e.g. "(\\d+).pcd"
void get_numbered_files(const std::string& file_dir, const std::string& regex_str, std::vector<std::string>& file_paths);
