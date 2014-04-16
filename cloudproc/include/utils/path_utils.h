#pragma once

#include <vector>
#include <string>

template <typename T>
T str2num ( const std::string &text );

// Gets files in directory ordered by number
// regex_str match 1 should extract number, e.g. "(\\d+).pcd"
void get_numbered_files(const std::string& file_dir, const std::string& regex_str, std::vector<std::string>& file_paths);

// Get files of certain format in specified directory in
// range(start, start + step*count, step) excluding start + step*count
void get_range_files(const std::string& file_dir, int start, int step, int count, const std::string& format_str, std::vector<std::string>& file_paths, bool check_exists=false);
