#include "hjlog.h"
#include <cstdarg>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <iostream>
#include <ctime>
#include <sys/time.h>
#include <string>
#include <mutex>

namespace collect_node_mcu {

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
        delete _file;
    }
};

std::string funformat(const char *format, va_list args)
{
    std::string buf(512, '\0');
    va_list args_copy;
    va_copy(args_copy, args);

    int ret = vsnprintf(const_cast<char *>(buf.data()), buf.size(), format, args_copy);
    va_end(args_copy);

    if (ret >= buf.size()) {
        // 重新分配内存
        buf.resize(ret + 1);  // 加上终止符
        va_copy(args_copy, args);
        vsnprintf(const_cast<char *>(buf.data()), ret + 1, format, args_copy);
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
    if (usr == nullptr|| usr->_fd <= 0) {
        return;
    }
    
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
        std::cerr << "open file fail" << std::endl;
    }
}

} //namespace hjlog

ssize_t hjlog_append(void* log, unsigned int level, 
               const char* file, unsigned int line, bool needtime, const char *format, ...)
{
    if(log == nullptr) {
        return -1;
    }

    struct hjlog::logger* usr = static_cast<struct hjlog::logger*>(log);

    if(usr->_fd <= 0 || usr->_level > level) {
        return 0;
    }

    va_list args;
    va_start(args, format);
    auto data = std::move(hjlog::funformat(format,args));
    va_end(args);
    size_t len = 256 + data.length();
    //char buf[len] = {0};
    char* buf = new char[len]; //clang tidy require
    struct stat fstat;
    int res = stat(usr->_file->data(), &fstat);
    if (res != 0) {
       return 0;
    }
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
            static_cast<int>(tv.tv_usec/1000),
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
    delete[] buf;
    return written;
}

ssize_t mcukey_append(void* log, const char *format, ...)
{
    if(log ==  nullptr) {
        return -1;
    }

    struct hjlog::logger* usr = static_cast<struct hjlog::logger*>(log);

    if(usr->_fd <= 0) {
        return 0;
    }

    va_list args;
    va_start(args, format);
    auto data = std::move(hjlog::funformat(format,args));
    va_end(args);
    size_t len = 128 + data.length();
    //char buf[len] = {0};
    char* buf = new char[len]; //clang tidy require
    struct stat fstat;
    int res = stat(usr->_file->data(), &fstat);
    if (res != 0) {
       return 0;
    }
    ssize_t written = 0;
    
    time_t now = time(NULL);
    struct tm* local_time = localtime(&now);
    struct timeval tv;
    gettimeofday(&tv,NULL);
    snprintf(buf, len, "[%02d-%02d %02d:%02d:%02d.%03d] %s",
        local_time->tm_mon+1,
        local_time->tm_mday,
        local_time->tm_hour,
        local_time->tm_min,
        local_time->tm_sec,
        static_cast<int>(tv.tv_usec/1000),
        data.data());
    
    {
        std::lock_guard<std::mutex> lc(usr->_mtx);
        if(fstat.st_size >= usr->_size) {
            rollover(usr);
        }
        written = write(usr->_fd, buf, strlen(buf));
    }

    delete[] buf;
    return written;
}

void* hj_cst_log_add(const char* file, unsigned int level, unsigned int size, unsigned int num)
{
    int fd = -1;
    if ((fd = ::open(file, O_RDWR|O_CREAT|O_APPEND, 0666)) <= 0) {
        std::cerr << "open: " << file << "  fail:" << errno << std::endl;
        return NULL;
    }

    struct hjlog::logger* usr = nullptr;
    usr = new hjlog::logger(file, level, size, num);
    if (usr == nullptr) {
        return NULL;
    }

    usr->_fd = fd;
    
    if(usr->_file == nullptr) {
        return NULL;
    }
    
    return reinterpret_cast<void*>(usr);
}

void hj_cst_log_del(void* log)
{
    if(log == nullptr) {
        return;
    }
    
    struct hjlog::logger* usr = static_cast<struct hjlog::logger*>(log);
    if(usr->_fd > 0) {
        fsync(usr->_fd);
        close(usr->_fd);
    }

    delete usr;
    log = NULL;
}

void hj_log_set_level(void* log, unsigned int level)
{
    if(log == nullptr) {
        return;
    }
    
    struct hjlog::logger* usr = static_cast<struct hjlog::logger*>(log);
    if(usr->_fd > 0) {
        std::lock_guard<std::mutex> lc(usr->_mtx);
        usr->_level = level;
    }
}

} // namespace collect_node_mcu