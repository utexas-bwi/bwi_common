#pragma once

#include <iomanip>
#include <stdio.h>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/static_assert.hpp>
#include <iostream>
#include <fstream>
#include <actasp/asp/AspTerm.h>

namespace actasp {

static std::vector<std::string> dirToAllAspFilesInDir(const std::string &dirPath) {
  std::vector<std::string> result;

  for(auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(dirPath), {})) {
    if (boost::filesystem::is_directory(entry.path()) || boost::filesystem::extension(entry.path()) != ".asp") {
      continue;
    }
    result.emplace_back(entry.path().string());

  }
  return result;
}

static int hash_vector(std::vector<long> const& vec) {
  std::size_t seed = vec.size();
  std::hash<long> longHasher;
  for(auto& i : vec) {
    seed ^= longHasher(i) + 0x9e3779b9 + longHasher(seed << 6) + longHasher(seed >> 2);
  }
  //std::cerr << ("Seed " + std::to_string(seed)) << std::endl;
  return seed;
}

static void recursive_copy(const boost::filesystem::path &src, const boost::filesystem::path &dst) {
  if (boost::filesystem::exists(dst)) {
    throw std::runtime_error(dst.generic_string() + " exists");
  }

  if (boost::filesystem::is_directory(src)) {
    boost::filesystem::create_directories(dst);

    #if BOOST_VERSION >= 106200
    for (boost::filesystem::directory_entry &item : boost::filesystem::directory_iterator(src)) {
      recursive_copy(item.path(), dst / item.path().filename());
    }
    #else
    boost::filesystem::directory_iterator end_iter;

    for (boost::filesystem::directory_iterator dir_itr(src);
         dir_itr != end_iter;
         ++dir_itr)
    {
      recursive_copy(dir_itr->path(), dst / dir_itr->path().filename());
    }
    #endif
  } else if (boost::filesystem::is_regular_file(src)) {
    boost::filesystem::copy(src, dst);
  } else {
    throw std::runtime_error(dst.generic_string() + " not dir or file");
  }
}

static std::string
getQueryDirectory(const std::vector<std::string> &linkFiles, const std::vector<actasp::AspAtom> *knowledge) {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream stampstream;
  const auto format = "%d-%m-%Y_%H-%M-%S";
  // std::put_time is part of cpp11 but isn't supported by old compilers, including the version of GCC that comes
  // with Indigo
#if __GNUC__ >= 5
  stampstream << std::put_time(&tm, format);
#else
  char date_buf[1000];
  time_t now;
  time(&now);
  strftime(date_buf, 1000, format, localtime(&now));
  stampstream << date_buf;
#endif

  // Until we have a solid hashing plan for knowledge, no caching.
  // It's also probably more performant to keep an in memory cache... This may just stick
  // around as a debugging convenience
  const boost::filesystem::path queryRootPath("/tmp/actasp/");

  const auto queryDir = queryRootPath / stampstream.str();


  boost::filesystem::create_directories(queryDir);

  return queryDir.string();
}

static std::string getQueryDirectory(const std::vector<std::string> &linkFiles, const std::vector<std::string> &copyFiles) {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::stringstream stampstream;
  const auto format = "%d-%m-%Y_%H-%M-%S";
  // std::put_time is part of cpp11 but isn't supported by old compilers, including the version of GCC that comes
  // with Indigo
 #if __GNUC__ >= 5
  stampstream << std::put_time(&tm, format);
 #else
  char date_buf[1000];
  time_t now;
  time(&now);
  strftime(date_buf, 1000, format, localtime(&now));
  stampstream << date_buf;
 #endif

  std::vector<long> queryFilesSignature;
  std::hash<std::string> str_hash;
  for (const auto &linkFile: linkFiles) {
    queryFilesSignature.push_back(str_hash(linkFile));
  }
  for (const auto &copyFile: copyFiles) {
    std::ifstream t(copyFile);
    std::stringstream buffer;
    buffer << t.rdbuf();
    //auto last_changed_time = boost::filesystem::last_write_time(copyFile);
    queryFilesSignature.push_back(str_hash(buffer.str()));
  }
  const boost::filesystem::path queryRootPath("/tmp/actasp/");
  const auto hash = hash_vector(queryFilesSignature);
  auto hashAsString = std::to_string(hash);

  hashAsString = hashAsString.substr(std::max(0, ((int)hashAsString.size() - 8))) + "/";
  const auto queryDir = queryRootPath / hashAsString ;

  if (!boost::filesystem::is_directory(queryDir)) {
    boost::filesystem::create_directories(queryDir);
  }
  auto convenienceLinkPath = queryRootPath / stampstream.str();
  if (!boost::filesystem::is_symlink(convenienceLinkPath)) {
    boost::filesystem::create_symlink(queryDir, convenienceLinkPath);
  }
  return queryDir.string();
}

static std::vector<boost::filesystem::path> populateDirectory(const boost::filesystem::path &dirPath, const std::vector<std::string> &linkFiles, const std::vector<std::string> &copyFiles) {
  std::vector<boost::filesystem::path> outFilePaths;
  for (const boost::filesystem::path linkFile: linkFiles) {
    auto target = dirPath / linkFile.filename();
    outFilePaths.emplace_back(target);
    if (boost::filesystem::exists(target)){
      continue;
    }
    boost::filesystem::create_symlink(linkFile, target);
  }
  for (const boost::filesystem::path copyFile: copyFiles) {
    // This should silently return if there's already a file there
    auto target = dirPath / copyFile.filename();
    outFilePaths.emplace_back(target);
    if (boost::filesystem::exists(target)){
      continue;
    }
    boost::filesystem::copy(copyFile, target);
  }
  return outFilePaths;
}

static std::vector<boost::filesystem::path>
populateDirectory(const boost::filesystem::path &dirPath, const std::vector<std::string> &linkFiles) {
  return populateDirectory(dirPath, linkFiles, {});
}

}