#ifndef _FILE_MANAGER_H
#define _FILE_MANAGER_H

#include "../../head.hpp"

namespace multisensor_localization
{
    class FileManager
    {
    public:
        static bool CreateFile(ofstream &ofs, string file_path);
        static bool CreateDirectory(string directory_path);
    };
} // namespace multisensor_localization

#endif