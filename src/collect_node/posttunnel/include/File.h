#pragma once

#include <boost/filesystem.hpp>
#include <string>
#include <ctime>

namespace collect_node_posttunnel {

class File
{
public:
    explicit File(std::string filepath);
    
    /******************************************************************************
	* libboost_filesystem C++ interface
	******************************************************************************/

    //获取文件名
    std::string getFileName();
    //获取完整文件名
    std::string getFullNameWithPath();
    //获取文件md5
    std::string getMd5();
    //获取最后一次修改时间
    std::time_t getLastWriteTime();
    //获取文件大小
    uint64_t getFileSize();
    //文件是否存在
    bool isFileExist();
    //是否是绝对路径
    bool isAbsolutePath();
    //删除文件
    bool remove();
    //拷贝文件 
    bool copyFileToOther(const File& other);

    bool operator==(File& other);

private:
    boost::filesystem::path filepath_;
};

} //namespace collect_node_posttunnel