#include "files.h"

#include <algorithm>
#include <dirent.h>
#include <iostream>

namespace convenience {

bool read_directory(std::vector<std::string>& files, const std::string& path)
{
    files.clear(); // just in case
    DIR* dir = opendir(path.c_str());
    if (dir == NULL) {
        std::cout << "Cannot read directory " << path << std::endl;
        return false;
    }
    struct dirent* ent;
    // read all files in directory
    while ((ent = readdir(dir)) != NULL) {
        std::string entry(ent->d_name);
        if (ent->d_type != DT_DIR && entry != "." && entry != "..") {
            files.push_back(path + "/" + entry);
        }
    }
    closedir(dir);
    // sort based on file name
    std::sort(files.begin(), files.end());
    return !files.empty();
}

} // namespace convenience
