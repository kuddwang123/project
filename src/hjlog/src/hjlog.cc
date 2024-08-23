#include "hjlog.h"
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <string>
#include <mutex>
#include <boost/filesystem.hpp>

namespace hjlog {
#define FILENAME(x) strrchr(x, '/')?strrchr(x,'/')+1:x

struct logger
{
    std::string* _file;
    unsigned int _fd;
    unsigned int _level;
    unsigned int _size;
    unsigned int _filenum;
    std::mutex _mtx;
    
    logger(const char* name, unsigned int level, unsigned int size, unsigned int filenum):
        _file(new std::string(name)),_fd(0),_level(0),_size(size),_filenum(filenum) {}
        

    ~logger() {
        if(_file)
            delete _file;
    }
};

std::string funformat(const char *format, va_list args)
{
    std::string buf(512, '\0');
    va_list args_copy;
    va_copy(args_copy, args);

    int ret = vsnprintf((char *)buf.data(), buf.size(), format, args_copy);
    va_end(args_copy);

    if (ret >= buf.size()) {
        // 重新分配内存
        buf.resize(ret + 1);  // 加上终止符
        va_copy(args_copy, args);
        ret = vsnprintf((char *)buf.data(), ret + 1, format, args_copy);
        va_end(args_copy);
    }
    return buf;
}

std::string level2str(unsigned int ll)
{
    switch (ll)
    {
    case TRACE_LOG:
        return "[TRACE]";
    case DEBUG_LOG:
        return "[DEBUG]";
    case INFO_LOG:
        return "[INFO]";
    case WARN_LOG:
        return "[WARN]";
    case ERROR_LOG:
        return "[ERROR]";
    case FATAL_LOG:
        return "[FATAL]";
    default:
        return "[NONE]";
    }
}

void rollover(struct logger* usr)
{
    if (!usr || usr->_fd <= 0)
        return;
    
    close(usr->_fd);
    
    if (usr->_filenum > 1) {
        std::string fileMax(*(usr->_file) + "." + std::to_string(usr->_filenum));
        std::remove(fileMax.c_str());
        for (unsigned int i = usr->_filenum-1; i > 0;) {
            std::string target, source;
            --i;
            if (i == 0) {
                source = std::string(*(usr->_file));
                target = std::string(*(usr->_file) + ".1");
            } else {
                source = std::string(*(usr->_file) + "." + std::to_string(i));
                target = std::string(*(usr->_file) + "." + std::to_string(i+1));
            }

            std::rename(source.c_str(), target.c_str());
        }

        std::string newfile(*(usr->_file) + ".1");
        std::rename(usr->_file->data(), newfile.c_str());
    }

    usr->_fd = ::open(usr->_file->data(), O_WRONLY|O_CREAT|O_APPEND, 0666);
    
    if(usr->_fd <= 0) {
        fprintf(stderr, "open file [%s] fail\n", usr->_file->c_str());
    }
}

} //namespace hjlog

ssize_t hjlog_append(void* log, unsigned int level, 
               const char* file, unsigned int line, bool needtime, const char *format, ...)
{
    if(!log) {
        return -1;
    }

    struct hjlog::logger* usr = (struct hjlog::logger*)log;

    if(usr->_fd <= 0 || usr->_level > level) {
        return 0;
    }

    va_list args;
    va_start(args, format);
    auto data = std::move(hjlog::funformat(format,args));
    va_end(args);
    size_t len = 256 + data.length();
    char buf[len] = {0};
    struct stat fstat;
    stat(usr->_file->data(), &fstat);
    ssize_t written = 0;

    if(needtime) {
        time_t now = time(NULL);
        struct tm* local_time = localtime(&now);
        struct timeval tv;
        gettimeofday(&tv,NULL);
        snprintf(buf, len, "%s[%04d-%02d-%02d %02d:%02d:%02d.%03d][%s:%d] %s",
            hjlog::level2str(level).c_str(),
            local_time->tm_year+1900,
            local_time->tm_mon+1,
            local_time->tm_mday,
            local_time->tm_hour,
            local_time->tm_min,
            local_time->tm_sec,
            (int)tv.tv_usec/1000,
            FILENAME(file),
            line,
            data.data());
    } else {
        snprintf(buf, len, "%s[%s:%d] %s",
            hjlog::level2str(level).c_str(),
            FILENAME(file),
            line,
            data.data());
    }

    {
        std::lock_guard<std::mutex> lc(usr->_mtx);
        if(fstat.st_size >= usr->_size) {
            rollover(usr);
        }
        written = write(usr->_fd, buf, strlen(buf));
    }

    return written;
}

void* hj_cst_log_add(const char* file, unsigned int level, unsigned int size, unsigned int num)
{
    int fd = -1;
    boost::filesystem::path boostfile(file); 

    if (!boostfile.is_absolute()) {
        fprintf(stderr, "hj cst log add fail, not absolute path:%s\n", file);
        return NULL;
    }

    std::string parent_path = boostfile.parent_path().string();

    if ((parent_path != "/tmp/logging") && (parent_path != "/userdata/hj/log")) {
        fprintf(stderr, "hj cst log add fail, invalid parent path:%s\n", parent_path.c_str());
        return NULL;
    }

    if ((fd = ::open(file, O_RDWR|O_CREAT|O_APPEND, 0666)) <= 0) {
        fprintf(stderr, "open [%s] fail:%d, ret=%d", file, errno, fd);
        return NULL;
    } else {
        fprintf(stdout, "open [%s] success\n", file);
    }

    struct hjlog::logger* usr = nullptr;
    usr = new hjlog::logger(file, level, size, num);
    if (!usr) {
        return NULL;
    }

    usr->_fd = fd;
    
    if(!usr->_file) {
        return NULL;
    }
    
    return (void*)usr;
}

void hj_cst_log_del(void* log)
{
    if(!log)
        return;
    
    struct hjlog::logger* usr = (struct hjlog::logger*)log;
    if(usr->_fd > 0) {
        fsync(usr->_fd);
        close(usr->_fd);
    }

    delete usr;
    log = NULL;

    return;
}

void hj_log_set_level(void* log, unsigned int level)
{
    if(!log)
        return;
    
    struct hjlog::logger* usr = (struct hjlog::logger*)log;
    if(usr->_fd > 0) {
        std::lock_guard<std::mutex> lc(usr->_mtx);
        usr->_level = level;
    }

    return;
}