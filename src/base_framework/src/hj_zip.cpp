#include "hj_zip.h"
#include "boost/filesystem.hpp"
#include "log.h"


namespace hj_bf {
static bool CreateZipFileToDir(zip_t* zip_fd, const std::string& dir_path, const std::string& root_dir_path);
static void AddZipFile(zip_t* zip_fd, const std::string& path, zip_source *source);

static void AddZipFile(zip_t* zip_fd, const std::string& path, zip_source *source) {
  zip_int64_t idx = zip_file_add(zip_fd, path.data(), source, ZIP_FL_OVERWRITE|ZIP_FL_ENC_GUESS);
  if (idx < 0) {
    HJ_ERROR("Failed to add file to zip file: %s", path.c_str());
    return;
  }
  int ret = zip_set_file_compression(zip_fd, idx, ZIP_CM_DEFLATE, 1);
  if (ret != 0) {
    HJ_ERROR("zip_set_file_compression errro. file: %s", path.data());
    return;
  }
}

static bool CreateZipFileToDir(zip_t* zip_fd, const std::string& dir_path, const std::string& root_dir_path) {
  boost::filesystem::path dir_path_obj(dir_path);
  for (boost::filesystem::directory_iterator it(dir_path_obj); it != boost::filesystem::directory_iterator(); ++it) {
    const boost::filesystem::path& file_path = it->path();
    if (boost::filesystem::is_directory(file_path)) {
      if (file_path.string() != root_dir_path) {
        // std::string relative_path = boost::filesystem::relative(file_path, root_dir_path).string();
        zip_dir_add(zip_fd, file_path.c_str(), ZIP_FL_ENC_GUESS);
      }
      CreateZipFileToDir(zip_fd, file_path.string(), root_dir_path);
    } else {
      if (!boost::filesystem::exists(file_path)) {
        HJ_ERROR("The file %s is not exist", file_path.c_str());
        continue;
      }
      struct zip_source *source = zip_source_file(zip_fd, file_path.string().c_str(),
                                                  ZIP_FILE_START_OFFSET,  ZIP_LENGTH_TO_END);
      if (source == nullptr) {
        HJ_ERROR("zip_source_file failed for %s with the reason %s\n",
                 file_path.string().c_str(), zip_strerror(zip_fd));
        continue;
      }
      AddZipFile(zip_fd, file_path.string(), source);
    }
  }
  return true;
}

bool CreateZipFileByDir(const std::string& zip_file_name, const std::string& dir_path) {
  boost::filesystem::path dir_path_obj(dir_path);
  if (!boost::filesystem::is_directory(dir_path_obj)) {
    HJ_ERROR("The path %s is not a directory", dir_path.c_str());
    return false;
  }
  int err = 0;
  zip_t *zip_fd = nullptr;
  zip_fd = zip_open(zip_file_name.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &err);
  if (zip_fd == nullptr) {
    HJ_ERROR("Failed to create zip file: %s", zip_file_name.c_str());
    return false;
  }
  CreateZipFileToDir(zip_fd, dir_path, dir_path);
  int ret = zip_close(zip_fd);
  if (ret != 0) {
    zip_error_t *error_ptr = zip_get_error(zip_fd);
    const char *str_error = zip_error_strerror(error_ptr);
    HJ_ERROR("save zip file failed., Error message: %s", str_error);
    return false;
  }
  return true;
}

bool CreateZipFileByFile(const std::string& zip_file_name, const std::string& file_path) {
  int err = 0;
  zip_t *zip_fd = nullptr;
  zip_fd = zip_open(zip_file_name.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &err);
  if (zip_fd == nullptr) {
    HJ_ERROR("Failed to create zip file: %s", zip_file_name.c_str());
    return false;
  }
  if (!boost::filesystem::exists(file_path)) {
    HJ_ERROR("The file %s is not exist", file_path.c_str());
    zip_close(zip_fd);
    return false;
  }
  struct zip_source *source = zip_source_file(zip_fd, file_path.data(),
                                              ZIP_FILE_START_OFFSET, ZIP_LENGTH_TO_END);
  if (source == nullptr) {
    HJ_ERROR("Failed to create zip source for file: %s", file_path.c_str());
    zip_close(zip_fd);
    return false;
  }

  boost::filesystem::path file_path_obj(file_path);
  std::string file_name = file_path_obj.filename().string();
  AddZipFile(zip_fd, file_name, source);
  int ret = zip_close(zip_fd);
  if (ret != 0) {
    zip_error_t *error_ptr = zip_get_error(zip_fd);
    const char *str_error = zip_error_strerror(error_ptr);
    HJ_ERROR("save zip file failed. file: %s, Error message: %s", file_path.data(), str_error);
    return false;
  }
  return true;
}

bool CreateZipFileByFiles(const std::string& zip_file_name, const std::vector<std::string>& file_lists) {
  int err = 0;
  zip_t *zip_fd = nullptr;
  zip_fd = zip_open(zip_file_name.c_str(), ZIP_CREATE | ZIP_TRUNCATE, &err);
  if (zip_fd == nullptr) {
    HJ_ERROR("Failed to create zip file: %s", zip_file_name.c_str());
    return false;
  }

  for (auto file : file_lists) {
    if (!boost::filesystem::exists(file)) {
      HJ_ERROR("The file %s is not exist", file.c_str());
      continue;
    }
    struct zip_source *source = zip_source_file(zip_fd, file.c_str(),
                                                ZIP_FILE_START_OFFSET, ZIP_LENGTH_TO_END);
    if (source == nullptr) {
      HJ_ERROR("zip_source_file failed for %s with the reason %s\n", file.c_str(), zip_strerror(zip_fd) );
    }
    AddZipFile(zip_fd, file, source);
  }
  int ret = zip_close(zip_fd);
  if (ret != 0) {
    zip_error_t *error_ptr = zip_get_error(zip_fd);
    const char *str_error = zip_error_strerror(error_ptr);
    HJ_ERROR("save zip file failed., Error message: %s", str_error);
    return false;
  }
  return true;
}


}  // namespace hj_bf
