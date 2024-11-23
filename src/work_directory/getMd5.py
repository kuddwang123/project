import os
import hashlib
import sys


def get_md5(file_path):
  """
  Get the md5 of a file.
  """
  if not os.path.isfile(file_path):
      return None
  md5_sum = None
  with open(file_path, 'rb') as f:
      md5_sum = hashlib.md5(f.read()).hexdigest()
  return md5_sum

def get_md5_dir(dir_path):
  """
  Get the md5 of all files in a directory.
  """
  md5_dict = {}
  for root, dirs, files in os.walk(dir_path):
      for file in files:
          if file.endswith('.so'):
            file_path = os.path.join(root, file)
            md5_sum = get_md5(file_path)
            if md5_sum:
                key = os.path.relpath(file_path, dir_path)
                md5_dict[key] = md5_sum
  return md5_dict


if __name__ == '__main__':
  if len(sys.argv) != 2:
    print("example: python getMd5.py x9")
    exit(1)
  project_name = sys.argv[1]
  if project_name not in ['x9', 'T1pro']:
    print("please input project name:  x9 or T1pro, but not %s" % project_name)
    exit(1)
  current_path = os.getcwd()
  lib_path = "%s/%s/data/hj/lib" % (current_path, project_name)
  md5_save_path = '%s/%s/Readme/md5.txt' % (current_path, project_name)
  md5 = get_md5_dir(lib_path)
  with open(md5_save_path,'w+') as file0:
    for k, v in md5.items():
      print(k, v, file=file0)
  print("MD5 saved to %s" % md5_save_path)