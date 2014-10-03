#ifndef ENUM_1E2YM5S5
#define ENUM_1E2YM5S5

/*
* @file Tools/Enum.h
* Defines a macro that declares an enum and provides
* a function to access the names of its elements.
*
* @author Thomas Röfer
* @author Samuel Barrett
*/

#include <string>
#include <vector>
#include <string.h>
#include <bwi_rl/common/Params.h>

/**
* @class EnumName
* The class converts a single comma-separated string of enum names
* into single entries that can be accessed in constant time.
* It is the worker class for the templated version below.
*/
class EnumName {
private:
  std::vector<std::string> names; /**< The vector of enum names. */
  
  /**
  * A method that trims a string, i.e. removes spaces from its
  * beginning and end.
  * @param s The string that is trimmed.
  * @return The string without spaces at the beginning and at its end.
  */
  static std::string trim(const std::string& s);
  
public:
  /**
  * Constructor.
  * @param enums A string that contains a comma-separated list of enum
  *              elements. It is allowed that an element is initialized
  *              with the value of its predecessor. Any other 
  *              initializations are forbidden.
  *              "a, b, numOfLettersBeforeC, c = numOfLettersBeforeC, d" 
  *              would be a legal parameter.
  * @param numOfEnums The number of enums in the string. Reassignments do
  *                   not count, i.e. in the example above, this 
  *                   parameter had to be 4.
  */
  EnumName(const std::string& enums, size_t numOfEnums);

  /**
  * The method returns the name of an enum element.
  * @param index The index of the enum element.
  * @return Its name.
  */
  const char* getName(size_t e) {return e >= names.size() ? 0 : names[e].c_str();}
};

/**
* Defining an enum and a function getName(<Enum>) that can return
* the name of each enum element. The enum will automatically
* contain an element NUM that reflects the number of
* elements defined.
*/
#define ENUM(Enum, ...) \
  namespace Enum { \
    enum type {__VA_ARGS__, NUM}; \
  } \
  typedef Enum::type Enum##_t; \
  inline static const char* getName(Enum::type e) {static EnumName en(#__VA_ARGS__, (size_t) Enum::NUM); return en.getName((size_t) e);} \
  namespace Enum { \
    inline type fromName(const char* name, bool ignoreUnknown = false) { \
      for (size_t i = 0; i < NUM; i++) { \
        if (strcmp(getName((type)i),name) == 0) \
          return (type)i; \
      } \
      if (ignoreUnknown) \
        return NUM; \
      std::cerr << "Problem converting " << name << " to type " << #Enum << std::endl; \
      exit(13); \
    } \
    inline type fromName(const std::string &name, bool ignoreUnknown = false) {return fromName(name.c_str(),ignoreUnknown);} \
  } \
  SET_FROM_JSON_ENUM(Enum)

#endif /* end of include guard: ENUM_1E2YM5S5 */
