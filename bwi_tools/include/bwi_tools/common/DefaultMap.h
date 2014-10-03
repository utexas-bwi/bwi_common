#ifndef DEFAULTMAP_E8UQ8T6
#define DEFAULTMAP_E8UQ8T6

/*
File: DefaultMap.h
Author: Samuel Barrett
Description: a map that returns a default value without inserting when get is called.
Created:  2011-08-23
Modified: 2011-12-13
*/

//#define DEFAULTMAP_USE_BOOST

#ifdef DEFAULTMAP_USE_BOOST
#include <boost/unordered_map.hpp>
#else
#include <map>
#endif

template <class Key, class T>
class DefaultMap {
public:
#ifdef DEFAULTMAP_USE_BOOST
  typedef typename boost::unordered_map<Key,T>::const_iterator const_iterator;
  typedef typename boost::unordered_map<Key,T>::iterator iterator;
#else
  typedef typename std::map<Key,T>::const_iterator const_iterator;
  typedef typename std::map<Key,T>::iterator iterator;
#endif
  DefaultMap(T defaultValue):
    defaultValue(defaultValue)
  {}

  T get(const Key &key) const {
#ifdef DEFAULTMAP_USE_BOOST
    typename boost::unordered_map<Key,T>::const_iterator it = vals.find(key);
#else
    typename std::map<Key,T>::const_iterator it = vals.find(key);
#endif
    if (it == vals.end())
      return defaultValue;
    else
      return it->second;
  }

  T& operator[](const Key &key) {
    if (vals.count(key) == 0)
      vals[key] = defaultValue;
    return vals[key];
  }

  void set(const Key &key, const T &val) {
    vals[key] = val;
  }

  void clear() {
    vals.clear();
  }

  void erase(iterator position) {
    vals.erase(position);
  }

  size_t erase(const Key &key) {
    return vals.erase(key);
  }

  unsigned int size() {
    return vals.size();
  }

  const_iterator begin() const {
    return vals.begin();
  }
  
  iterator begin() {
    return vals.begin();
  }
  
  const_iterator end() const {
    return vals.end();
  }
  
  iterator end() {
    return vals.end();
  }

private:
#ifdef DEFAULTMAP_USE_BOOST
  boost::unordered_map<Key,T> vals;
#else
  std::map<Key,T> vals;
#endif
  const T defaultValue;
};

#endif /* end of include guard: DEFAULTMAP_E8UQ8T6 */
