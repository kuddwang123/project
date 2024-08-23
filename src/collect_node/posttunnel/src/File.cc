#include "File.h"
#include "log.h"

namespace collect_node_posttunnel {
File::File(std::string filepath):
    filepath_(filepath)
{

}

std::string File::getFileName()
{
    return filepath_.filename().string();
}

std::string File::getFullNameWithPath()
{
    return filepath_.string();
}

std::time_t File::getLastWriteTime()
{
    return boost::filesystem::last_write_time(filepath_);
}

bool File::isFileExist()
{
    return boost::filesystem::exists(filepath_) && 
        boost::filesystem::is_regular_file(filepath_);
}

uint64_t File::getFileSize()
{
    return boost::filesystem::file_size(filepath_);
}

bool File::isAbsolutePath()
{
    return filepath_.is_absolute();
}

bool File::remove()
{
    return boost::filesystem::remove(filepath_);
}

bool File::copyFileToOther(const File& other)
{
    if (!this->isFileExist()) {
        return false;
    }

    boost::filesystem::copy_file(filepath_, other.filepath_, 
        boost::filesystem::copy_option::overwrite_if_exists);
    
    return true;
}

std::string File::getMd5()
{   
    char buffer[64] = {0};
    FILE* fp = NULL;
    std::string cmdStr = "md5sum ";
    cmdStr.append(filepath_.string());

    fp = popen(cmdStr.c_str(), "r");
    
    if (!fp) {
        HJ_ERROR("popen [%s] failed\n", cmdStr.c_str());
        return "";
    }

    fgets(buffer, sizeof(buffer), fp);

    fclose(fp);

    std::string result = buffer;
    std::string md5 = result.substr(0, 32);
    
    return md5;
}

bool File::operator==(File& other)
{
    return  this->isFileExist() && 
        other.isFileExist() && 
        (this->getMd5() == other.getMd5());
}

} //namespace collect_node_posttunnel