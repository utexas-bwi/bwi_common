#ifndef UTIL_T1FR2WSR
#define UTIL_T1FR2WSR

/*
File: Util.h
Author: Samuel Barrett
Description: a set of utility functions
Created:  2011-08-23
Modified: 2011-12-02
*/

#include <bwi_tools/json/json.h>
#include <boost/function.hpp>
#include <string>
#include <vector>
#include <ostream>
#include <map>
#include <sys/time.h>

#ifndef NULL
#define NULL 0
#endif

inline double getTime() {
  struct timeval time;

  gettimeofday(&time,NULL);
  return time.tv_sec + time.tv_usec / 1000000.0;
}
void tic(int id=0);
double toc(int id=0);
void toc(double &counter, int id=0);

class Timer {
public:
  Timer():
    last(0),
    counter(0)
  {}
  inline void tic() {last = getTime(); }
  inline void toc() {counter += getTime()-last;}
  inline double get() {return counter;}
private:
  double last;
  double counter;
};

template <class T>
inline int sgn(const T &x) {
  return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

template <class T>
inline T min(const T &x1, const T &x2) {
  return x1 < x2 ? x1 : x2;
}

template <class T>
inline T max(const T &x1, const T &x2) {
  return x1 > x2 ? x1 : x2;
}

unsigned int vectorMaxInd(const std::vector<float> &arr);
void vectorMax(const std::vector<float> &arr, float &maxVal, unsigned int &maxInd);

float softmax(float x1, float x2, float factor); // returns the probability of x1 using a softmax with the given factor
void softmax(const std::vector<unsigned int> &vals, float factor, std::vector<float> &probs); // fills out probs with the probabilities of the vals using a softmax with the given factor

bool readJson(const std::string &filename, Json::Value &value);

void jsonReplaceStrings(Json::Value &value, const std::map<std::string,std::string> &replacementMap);
void jsonReplaceStrings(Json::Value &value, boost::function<void (Json::Value &)> replace);

std::string indent(unsigned int indentation);

template <class T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &vect) {
  out << "[";
  for (unsigned int i = 0; i < vect.size(); i++) {
    out << vect[i];
    if (i != vect.size() - 1)
      out << ",";
  }
  out << "]";
  return out;
}

std::string tempFilename();

#endif /* end of include guard: UTIL_T1FR2WSR */
