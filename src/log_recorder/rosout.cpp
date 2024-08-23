#include <cstring>
#include <cstdlib>
#include <cctype>

#include "ros/ros.h"
#include "ros/file_log.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include "rosgraph_msgs/Log.h"
#include "rapidjson/document.h"
#include "rapidjson/error/en.h"
#include "rapidjson/filereadstream.h"
class Rosout
{
public:
  std::string log_file_name_;
  FILE* handle_;
  size_t max_file_size_;
  size_t current_file_size_;
  size_t max_backup_index_;
  size_t current_backup_index_;
  ros::NodeHandle node_;
  ros::Subscriber rosout_sub_;
  ros::Publisher agg_pub_;
  bool omit_topics_;

  Rosout() :
    log_file_name_("/tmp/log_recorder.log"),
    handle_(NULL),
    max_file_size_(100*1024*1024),
    current_file_size_(0),
    max_backup_index_(10),
    current_backup_index_(0),
    omit_topics_(false)
  {
    init();
  }

  void init()
  {
    std::string temp_config_path = LOG_CONFIG_PATH;
    const char* new_file_logging_env = getenv("LOG_RECORDER_NEW_PATH");
    std::string new_file_logging(new_file_logging_env ? new_file_logging_env : temp_config_path);
    std::string temp_log_file_name =  log_file_name_;
    FILE* fp = fopen(new_file_logging.c_str(), "rb");
    if (!fp) {
      std::cerr << "log_recorder config file not exist!" << std::endl;
    }else{
      fseek(fp, 0L, SEEK_END);
      auto length = ftell(fp);
      fseek(fp, 0L, SEEK_SET);
      char readBuffer[length + 16];
      rapidjson::FileReadStream is(fp, readBuffer, sizeof(readBuffer));
      rapidjson::Document document;
      rapidjson::ParseResult ok =
          document.ParseStream<rapidjson::kParseCommentsFlag | rapidjson::kParseTrailingCommasFlag>(is);
      fclose(fp);
      if (!ok) {
        char errstr[2000];
        snprintf(errstr, sizeof(errstr), "JSON parse error: %s (%u)", rapidjson::GetParseError_En(ok.Code()),
                static_cast<unsigned int>(ok.Offset()));
        std::cerr << errstr << std::endl;
      } else{
        if (document.IsObject()) {
          if (document.HasMember("all_log_file_path") && document["all_log_file_path"].IsString()) {
            std::string all_log_file_path = document["all_log_file_path"].GetString();
            if (::mkdir(all_log_file_path.c_str(), S_IRWXU | S_IRGRP | S_IXGRP) < 0) {
              std::cerr << "create path error:" << all_log_file_path<<std::endl;
            }
            if (document.HasMember("log_file_name") && document["log_file_name"].IsString()) {
              std::string log_file_name = document["log_file_name"].GetString();
              temp_log_file_name = all_log_file_path+log_file_name;
            }
          }
          if (document.HasMember("max_size") && document["max_size"].IsInt()) {
            int max_size = document["max_size"].GetInt();
            max_file_size_ = max_size*1024;
          }
          if (document.HasMember("max_index") && document["max_index"].IsInt()) {
            int max_index = document["max_index"].GetInt();
            max_backup_index_ = max_index;
          }
        }
      }
    }
    log_file_name_ = temp_log_file_name;
    std::cout << "new_file_logging :" << new_file_logging << std::endl;
    std::cout << "log_file_name_ :" << log_file_name_ << std::endl;
    handle_ = fopen(log_file_name_.c_str(), "w");

    if (handle_ == 0)
    {
      std::cerr << "Error opening rosout log file '" << log_file_name_.c_str() << "': " << strerror(errno);
    }
    else
    {
      std::cout << "logging to " << log_file_name_.c_str() << std::endl;

      std::stringstream ss;
      ss <<  "\n\n" << ros::Time::now() << "  Node Startup\n";
      int written = fprintf(handle_, "%s", ss.str().c_str());
      if (written < 0)
      {
        std::cerr << "Error writing to rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_)) << std::endl;
      }
      else if (written > 0)
      {
        current_file_size_ += written;
        if (fflush(handle_))
        {
          std::cerr << "Error flushing rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_));
        }
      }
    }

//    agg_pub_ = node_.advertise<rosgraph_msgs::Log>("/rosout_agg", 0);
//    std::cout << "re-publishing aggregated messages to /rosout_agg" << std::endl;

    rosout_sub_ = node_.subscribe("/log_recorder", 0, &Rosout::rosoutCallback, this);
    std::cout << "subscribed to /log_recorder" << std::endl;
  }

  void rosoutCallback(const rosgraph_msgs::Log::ConstPtr& msg)
  {
//    agg_pub_.publish(msg);

    if (!handle_)
    {
      return;
    }

    std::stringstream ss;
    ss << msg->header.stamp << " ";
    switch (msg->level)
    {
    case rosgraph_msgs::Log::FATAL:
      ss << "FATAL ";
      break;
    case rosgraph_msgs::Log::ERROR:
      ss << "ERROR ";
      break;
    case rosgraph_msgs::Log::WARN:
      ss << "WARN ";
      break;
    case rosgraph_msgs::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case rosgraph_msgs::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << msg->level << " ";
    }

    ss << msg->name << " ";
    ss << "[" << msg->file << ":" << msg->line << "(" << msg->function << ")] ";

    // check parameter server for omit_topics flag and set class member
    node_.getParamCached("/rosout/omit_topics", omit_topics_);
/*
    if (!omit_topics_)
    {
      ss << "[topics: ";
      std::vector<std::string>::const_iterator it = msg->topics.begin();
      std::vector<std::string>::const_iterator end = msg->topics.end();
      for ( ; it != end; ++it )
      {
        const std::string& topic = *it;

        if ( it != msg->topics.begin() )
        {
          ss << ", ";
        }

        ss << topic;
      }
      ss << "] ";
    }
*/
    ss << msg->msg;
    ss << "\n";
    int written = fprintf(handle_, "%s", ss.str().c_str());
    if (written < 0)
    {
      std::cerr << "Error writing to rosout log file '" << log_file_name_.c_str() << "': " << strerror(ferror(handle_)) << std::endl;
    }
    else if (written > 0)
    {
      current_file_size_ += written;
      if (fflush(handle_))
      {
        std::cerr << "Error flushing rosout log file '" << log_file_name_.c_str() << "': " << strerror(errno);
      }

      // check for rolling
      if (current_file_size_ > max_file_size_)
      {
        std::cout << "rosout log file " << log_file_name_.c_str() << " reached max size, rotating log files" << std::endl;
        if (fclose(handle_))
        {
          std::cerr << "Error closing rosout log file '" << log_file_name_.c_str() << "': " << strerror(errno) << std::endl;
        }
        if (current_backup_index_ == max_backup_index_)
        {
          std::stringstream backup_file_name;
          backup_file_name << log_file_name_ << "." << max_backup_index_;
          int rc = remove(backup_file_name.str().c_str());
          if (rc != 0)
          {
            std::cerr << "Error deleting oldest rosout log file '" << backup_file_name.str().c_str() << "': " << strerror(errno) << std::endl;
          }
        }
        std::size_t i = std::min(max_backup_index_, current_backup_index_ + 1);
        while (i > 0)
        {
          std::stringstream current_file_name;
          current_file_name << log_file_name_;
          if (i > 1)
          {
            current_file_name << "." << (i - 1);
          }
          std::stringstream rotated_file_name;
          rotated_file_name << log_file_name_ << "." << i;
          int rc = rename(current_file_name.str().c_str(), rotated_file_name.str().c_str());
          if (rc != 0)
          {
            std::cerr << "Error rotating rosout log file '" << current_file_name.str().c_str() << "' to '" << rotated_file_name.str().c_str() << "': " << strerror(errno) << std::endl;
          }
          --i;
        }
        if (current_backup_index_ < max_backup_index_)
        {
          ++current_backup_index_;
        }
        handle_ = fopen(log_file_name_.c_str(), "w");
        if (handle_ == 0)
        {
          std::cerr << "Error opening rosout log file '" << log_file_name_.c_str() << "': " << strerror(errno);
        }
        current_file_size_ = 0;
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "log_recorder", ros::init_options::NoRosout);
  ros::NodeHandle n;
  Rosout r;

  ros::spin();

  return 0;
}

