#include "Utils.h"
#include "nlohmann/json.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include "Common.h"
#include "hjlog.h"

namespace aiper_ota {
namespace utils {

std::string getFileMd5(const char* filepath)
{
    char buffer[64] = {0};
    FILE* fp = NULL;
    std::string cmdStr = "md5sum ";
    cmdStr.append(filepath);

    fp = popen(cmdStr.c_str(), "r");
    
    if (!fp) {
        //HJ_ERROR("popen [%s] failed\n", cmdStr.c_str());
        return "";
    }

    fgets(buffer, sizeof(buffer), fp);

    fclose(fp);

    std::string result = buffer;
    std::string md5 = result.substr(0, 32);
    
    return md5;
}

bool isFileExist(const char* filepath)
{
    boost::filesystem::path boostfile(filepath);
    return boost::filesystem::exists(boostfile);
}

bool removeDir(const char* dirpath)
{
    boost::filesystem::path dir(dirpath);
    try {
        if (boost::filesystem::exists(dir)) {
            boost::filesystem::remove_all(dir);
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        fprintf(stderr, "Error deleting directory [%s]:[%s]\n",
            dirpath, e.what());
        return false;
    }

    fprintf(stdout, "Remove directory [%s] done\n", dirpath);
    return true;
}

bool emptyDir(const char* dirpath)
{
    boost::filesystem::path dir(dirpath);
    try {
        if (boost::filesystem::exists(dir) && boost::filesystem::is_directory(dir)) {
            for (auto& entry : boost::filesystem::directory_iterator(dir)) {  
                boost::filesystem::remove_all(entry);
            }
        }
    } catch (const boost::filesystem::filesystem_error& e) {
        fprintf(stderr, "Error deleting directory [%s]:[%s]\n",
            dirpath, e.what());
        return false;
    }

    fprintf(stdout, "Clear directory [%s] done\n", dirpath);
    return true;
}

bool removeFile(const char* filepath)
{
    boost::filesystem::path file(filepath);
    try {
        if (boost::filesystem::exists(file)) {
            if (!boost::filesystem::remove(file)) {
                return false;
            }
        } 
    } catch (const boost::filesystem::filesystem_error& e) {
        fprintf(stderr, "Error deleting file [%s]:[%s]\n",
            filepath, e.what());
        return false;
    }

    fprintf(stdout, "Remove file [%s] done\n", filepath);
    return true;
}

bool touchFile(const char* file)
{
    std::ofstream ofs(file, std::ios::out | std::ios::trunc);
    if (ofs) {
        ofs.close();
        return true;
    } else {
        return false;
    }
}

bool renameFile(const char* oldf, const char* newf)
{
    boost::filesystem::path old_file(oldf);
    boost::filesystem::path new_file(newf);

    try {
        if (!boost::filesystem::exists(old_file)) {
            fprintf(stderr, "Error: Source file does not exist\n");
            return false;
        }

        // 检查目标文件夹是否存在，不存在则创建
        if (!boost::filesystem::exists(new_file.parent_path())) {
            boost::filesystem::create_directories(new_file.parent_path());
        }

        boost::filesystem::copy_file(old_file, new_file, boost::filesystem::copy_option::overwrite_if_exists);
        boost::filesystem::remove(old_file);
    } catch (const boost::filesystem::filesystem_error& e) {
        fprintf(stderr, "%s\n", e.what());
        return false;
    }

    return true;

}

uintmax_t getAvailDiskSpace(const char* path)
{
    boost::filesystem::space_info si = boost::filesystem::space(path);
    return si.available;
}

DevInfo::DevInfo(const char* filepath):
    ifs_(filepath)
{
    parse();
}

DevInfo::~DevInfo()
{
    if (this->openSucc())
        ifs_.close();
}

bool DevInfo::openSucc()
{
    return ifs_.is_open();
}

void DevInfo::parse()
{
    if (!ifs_.is_open())
        return;
    
    std::string jsonString((std::istreambuf_iterator<char>(ifs_)),
                    std::istreambuf_iterator<char>());

    try {
        nlohmann::json dev = nlohmann::json::parse(jsonString);
        model_ = dev["model"];
        sn_ = dev["sn"];
        version_ = dev["version"];
        bleName_ = dev["bleName"];
        waddr_ = dev["waddr"];
        mode_ = dev["mode"];
    } catch (nlohmann::json::exception& e) {
        fprintf(stderr, "parse json fail:%s\n", jsonString.c_str());
        ifs_.close();
    }
}

}
}//namespace aiper_ota