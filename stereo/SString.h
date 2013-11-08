#ifndef STRING_H
#define STRING_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

class String : public std::string {
public:
  String() {}
  String(const String& s, size_type pos = 0, size_type n = npos) : std::string(s,pos,n) {}
  String(const std::string& s, size_type pos = 0, size_type n = npos) : std::string(s,pos,n) {}
  String(const char* s) : std::string(s) {}
  String(const char* s, size_type n) : std::string(s,n) {}
  String(size_type n, value_type c) : std::string(n,c) {}

  template <class InputIterator>
  String(InputIterator first, InputIterator last) : std::string(first,last) {}

  using std::string::operator=;

  /**
   * Checks if the string ends with a given substring.
   */
  bool endsWith(const String& str) const {
    if (str.size() > size()) return false;
    return (compare(size()-str.size(), str.size(), str) == 0);
  }

  /**
   * Checks if the string starts with a given substring.
   */
  bool startsWith(const String& str) const {
    if (str.size() > size()) return false;
    return (compare(0, str.size(), str) == 0);
  }

  /**
   * Checks whether this string contains any of the characters in s.
   */
  bool contains(const String& s) const {
    return (find_first_of(s) != std::string::npos);
  }

  /**
   * Returns this string, but without white space on the ends.
   */
  String withoutWhite(void) const {
    size_type start = find_first_not_of(" \t\n\r");
    size_type end = find_last_not_of(" \t\n\r");
    if (start == npos) return "";
    if (end == npos) return ""; // shouldn't happen?
    return substr(start, end+1-start);
  }

  /**
   * Returns something printed as a String.
   */
  template<typename T>
  static String valueOf(const T& i) {
    std::ostringstream oss; 
    oss << i;
    return oss.str();
  }

  template<typename T>
  bool parse(T* x) const {
    return parse(*this, x);
  }
  template<typename T>
  void parseArray(std::vector<T>* arr) const {
    parseArray<T>(*this, arr);
  }

  template<typename T>
  static bool parse(const String& str, T* x) {
    std::istringstream iss(str);
    iss >> *x; 
    return !iss.fail();
  }
  template<typename T>
  static void parseArray(const String& str, std::vector<T>* arr) {
    std::istringstream iss(str);
    while (!iss.fail()) {
      arr->push_back(T());
      iss >> arr->back();
      if (iss.fail()) arr->pop_back();
    }
  }

  // Returns only the leading "directory" from the string.  Does not
  // include the trailing '/'.
  String dirname(void) const { return String::dirname(*this); }
  static String dirname(const String& str);

  // Returns the string without leading "directory"
  String basename(void) const { return String::basename(*this); }
  static String basename(const String& str);

  // Returns the string without extension
  String stripExtension(void) const { return String::stripExtension(*this); }
  static String stripExtension(const String& str);

  // Returns the extension from the string (if any;  "" otherwise).
  String getExtension(void) const { return String::getExtension(*this); }
  static String getExtension(const String& str);
};



String String::basename(const String& str) {
  size_type pos = str.find_last_of("\\/");
  if (pos == npos) return str;
  return str.substr(pos+1, str.size()-pos);
}

String String::dirname(const String& str) {
  size_type pos = str.find_last_of("\\/");
  if (pos == npos) return ".";
  return str.substr(0, pos);
}

// Returns the string without extension
String String::stripExtension(const String& str) {
  size_type pos = str.find_last_of(".");
  if (pos == npos) return str;
  return str.substr(0, pos);
}
// Returns the extension of a string
String String::getExtension(const String& str) {
  size_type pos = str.find_last_of(".");
  if (pos == npos) return "";
  return str.substr(pos);
}


#endif /* STRING_H */
