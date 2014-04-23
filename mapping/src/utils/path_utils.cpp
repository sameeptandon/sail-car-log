#include "utils/path_utils.h"

#include <sstream>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

namespace fs = boost::filesystem;


typedef std::multimap<int, fs::path> int_path_map;

template <typename T>
T str2num ( const std::string &text )
{
     std::stringstream ss(text);
     T result;
     return ss >> result ? result : -1;
}

void get_numbered_files(const std::string& file_dir, const std::string& reg_str, std::vector<std::string>& file_paths)
{
    assert(fs::exists(file_dir) && fs::is_directory(file_dir));
    fs::directory_iterator end_iter;

    int_path_map file_path_map;

    boost::smatch match;
    boost::regex re(reg_str);
    for (fs::directory_iterator dir_iter(file_dir); dir_iter != end_iter; ++dir_iter)
    {
        if (fs::is_regular_file(dir_iter->status()))
        {
            fs::path p = *dir_iter;
            std::string leaf = p.leaf().string();
            if (boost::regex_match(leaf, match, re))
            {
                std::string file_ind(match[1].first, match[1].second);
                int ind = str2num<int>(file_ind.c_str());
                file_path_map.insert(int_path_map::value_type(ind, *dir_iter));
            }
        }
    }

    file_paths.clear();
    for (int_path_map::iterator iter = file_path_map.begin(); iter != file_path_map.end(); ++iter)
    {
        file_paths.push_back(iter->second.string());
    }
}

void get_range_files(const std::string& file_dir, int start, int step, int count, const std::string& format_str, std::vector<std::string>& file_paths, bool check_exists)
{
    file_paths.clear();

    int end = start + step * count;
    for (int k = start; k < end; k += step)
    {
        fs::path p = fs::path(file_dir) / fs::path((boost::format(format_str) % k).str());
        if (check_exists && !fs::exists(p))
            break;
        file_paths.push_back(p.string());
    }
}
