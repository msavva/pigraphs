#pragma once

#include <sstream>

#include "libsg.h"  // NOLINT

namespace sg {
namespace util {

#ifdef _WIN32

#include <conio.h>
inline int wasKeyboardHit() { return static_cast<int>(_kbhit()); }

#define THREAD_LOCAL __declspec(thread)

#else // assume linux

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>

inline int wasKeyboardHit() {
  struct termios oldt, newt;
  int ch;
  int oldf;

  // don't echo and don't wait for ENTER
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);

  // make it non-blocking (so we can check without waiting)
  if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK)) {
    return 0;
  }

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  if (0 != fcntl(STDIN_FILENO, F_SETFL, oldf)) {
    return 0;
  }

  if (ch != EOF) {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

inline void Sleep(int millisecs) { usleep(millisecs * 1000); }

#define THREAD_LOCAL __thread

#endif // WIN32

//! Decompress gzip format file at filenameIn into uncompressed data in filenameOut
//! NOTE: Currently only safe to use for gzipped ASCII files
void decompressedGZipFile(const string& filenameIn, const string& filenameOut);

//! Return current time as Year-Month-Day-Hours-Seconds string
string timeNowYMDHSstring();

//! Return current time Month-Day-Hours-Seconds string
string timeNowMDHSstring();

//! Returns number of microseconds since Jan 1st 1601 UTC
uint64_t timeNow();

template <typename T>
inline void print(const T& t) { cout << t; }

template <typename T>
inline void println(const T& t) { cout << t << endl; }

// Find and removes one instance of target from a collection
template <typename VecT, typename T>
bool findAndRemove(VecT& vec, const T& target) {
  auto it = std::find(vec.begin(), vec.end(), target);
  if (it != vec.end()) {
    // Found - Remove
    vec.erase(it);
    return true;
  } else {
    return false;
  }
}

// Find and removes all instances of target from a collection
template <typename VecT, typename T>
int findAndRemoveAll(VecT& vec, const T& target) {
  int found = 0;
  while (findAndRemove(vec, target)) {
    found++;
  }
  return found;
}

// Find and removes all instances of target from a collection
template <typename VecT, typename T>
bool contains(VecT& vec, const T& target) {
  auto it = std::find(vec.begin(), vec.end(), target);
  return it != vec.end();
}


// Use the comparator functor comp to return a sorted list of
// indices in indices, for the range between iterBegin and iterEnd
template <typename Iter, typename Comparator>
void sortIndices(Iter iterBegin, Iter iterEnd, Comparator comp,
                 vec<size_t>& indices) {
  vec<pair<size_t, Iter>> pv;
  pv.reserve(iterEnd - iterBegin);

  Iter iter;
  size_t k;
  for (iter = iterBegin, k = 0; iter != iterEnd; iter++, k++) {
    pv.push_back(pair<size_t, Iter>(k, iter));
  }

  std::sort(pv.begin(), pv.end(),
            [&comp](const pair<size_t, Iter>& a,
                    const pair<size_t, Iter>& b) -> bool
  { return comp(*a.second, *b.second); });

  indices.resize(pv.size());
  std::transform(pv.begin(), pv.end(), indices.begin(),
                 [](const pair<size_t, Iter>& a) -> size_t
  { return a.first ; });
}

//! Convert a map m to a vector v containing all the values in m
template <typename M, typename V>
void mapToVec(const M& m, V& v) {
  v.reserve(m.size());
  for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
    v.push_back(it->second);
  }
}

//! Convert a map m to a vector v containing all the keys in m
template <typename M, typename V>
void mapToKeyVec(const M& m, V& v) {
  v.reserve(m.size());
  for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
    v.push_back(it->first);
  }
}

//! Convert a map m to a vector v containing all <key,value> pairs
template <typename M, typename V>
void mapToPairVec(const M& m, V& v) {
  v.reserve(m.size());
  for (typename M::const_iterator it = m.begin(); it != m.end(); ++it) {
    v.push_back(*it);
  }
}

//! Generic pair printing function assuming key-value pair
template<typename T1, typename T2>
ostream& operator<<(ostream& os, const pair<T1, T2>& p) {
  os << p.first << ":" << p.second;
  return os;
}

//! Generic to string
template<typename T>
string toString(const T& x) {
  std::stringstream s;
  s << x;
  return s.str();
}

//! Outputs container type C items to ostream os separated by string sep
template <typename C>
ostream& output(ostream& os, const C& items, const string& sep = ",") {
  int i = 0;
  for (const auto& item : items) {
    if (i > 0) {
      os << sep;
    }
    os << item;
    ++i;
  }
  return os;
}

template <typename T1, typename T2, class Pred1 = std::less<T1>, class Pred2 = std::less<T2>>
struct predicate_pair_check2then1 : public std::binary_function<std::pair<T1, T2>, std::pair<T1, T2>, bool> {
  predicate_pair_check2then1() : p1(), p2() {}
  bool operator ()(const std::pair<T1, T2> &l, const std::pair<T1, T2> &r) const {
    return p2(l.second, r.second) || ((l.second == r.second) && p1(l.first, r.first));
  }
 protected:
  Pred1 p1;
  Pred2 p2;
};

template <typename T1, typename T2>
struct predicate_pair_desc2asc1 : predicate_pair_check2then1<T1,T2, std::less<T1>, std::greater<T2>> { };

template <typename T1, typename T2>
struct predicate_pair_desc2 : public std::binary_function<std::pair<T1, T2>, std::pair<T1, T2>, bool> {
  bool operator ()(const std::pair<T1, T2> &l, const std::pair<T1, T2> &r) const {
    return l.second > r.second;
  }
};

// TODO: Check!!!
template <typename Gen, typename T, typename num_t = double, class ScorePred = std::greater<num_t>>
void getTopK(Gen gen, const std::function<num_t(const T&)>& scoreFunc, 
             int k, num_t threshold, int maxIters, vec<pair<T,num_t>>* out_pairs) {
  ScorePred cmp;
  predicate_pair_check2then1<T, num_t, std::less<T>, ScorePred> pair_cmp;
  out_pairs->clear();
  if (k > 0) {
    out_pairs->reserve(k);
  }
  int i = 0;
  while (i < maxIters) {
    const T& element = gen();
    i++;
    num_t score = scoreFunc(element);
    // Skip if score is threshold is better than score
    if (cmp(threshold, score)) continue;  
    if (k > 0 && out_pairs->size() >= k) {
      num_t worst_score = out_pairs->front().second;
      if (cmp(score, worst_score)) {
        // Remove worst element
        std::pop_heap(out_pairs->begin(), out_pairs->end(), pair_cmp);
        out_pairs->pop_back();
        // Add new element
        out_pairs->push_back(std::make_pair(element, score));
        std::push_heap(out_pairs->begin(), out_pairs->end(), pair_cmp);
      }
    } else {
      // Add new element
      out_pairs->push_back(std::make_pair(element, score));
      std::push_heap(out_pairs->begin(), out_pairs->end(), pair_cmp);
    }
  }
  std::sort_heap(out_pairs->begin(), out_pairs->end(), pair_cmp);
}

template <typename Iter>
ostream& pairsToDelimited(ostream& os, const Iter& pairs, const string delimiter = "\t") {
  for (const auto& p : pairs) {
    os << p.first << delimiter << p.second << std::endl;
  }
  return os;
}

//! Return a vector of strings by splitting line on any of the characters in the string delimiterCharList
vec<string> tokenize(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims);
void tokenize(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims, vec<string>* tokens);

//! Return a vector of strings by splitting line on any of the characters in the string delimiterCharList
//!  Delimiters in quotes are ignored.
vec<string> tokenizeWithQuotes(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims);
void tokenizeWithQuotes(const string& line, const string& delimiterCharList, int limit, bool mergeAdjDelims, vec<string>* tokens);

//! Return a vector of strings by splitting line on any of the characters in the string delimiterCharList
vec<string> tokenize(const string& line, const string& delimiterCharList);

//! Split string line on any of the characters in string delimiterCharList and store tokens into tokens
//! (NOTE: tokens is cleared at start)
void tokenize(const string& line, const string& delimiterCharList, vec<string>* tokens);

//! Joins container type C items into string, separated by string sep
template <typename C>
string join(const C& items, const string& sep = ",") {
  std::stringstream s;
  output(s, items, sep);
  return s.str();
}

//! Joins container type C items into string, separated by string sep
template <typename C, typename T>
string join(const C& items, const std::function<string(const T&)>& toStr, const string& sep = ",") {
  std::stringstream s;
  output(s, items, sep);
  return s.str();
}

//! Trims whitespace from begin and end of string
string trim(const string& str);
void trim_in_place(string& str);

//! Replace all instances of search in subject with replace
std::string replaceStringAll(std::string subject, const std::string& search,
                             const std::string& replace);

//! Returns whether s starts with exactly t
inline bool startsWith(const string& s, const string& t) {
  return s.compare(0, t.length(), t) == 0;
}

//! Returns whether s ends with exactly t
inline bool endsWith(const string& s, const string& t) {
  if (s.length() >= t.length()) {
    return s.compare(s.length()-t.length(), s.length(), t) == 0;
  } else {
    return false;
  }
}

//! Iterator pair for happy iterating over ranges
//! Maybe boost.range is better but this is very simple
template <typename Iterator>
class iterator_pair {
public:
  iterator_pair(Iterator first, Iterator last) : f_(first), l_(last) { }
  Iterator begin() const { return f_; }
  Iterator end() const { return l_; }

private:
  Iterator f_;
  Iterator l_;
};

//! Helper for creating a iterator pair
template <typename Iterator>
iterator_pair<Iterator> make_iterator_pair(Iterator f, Iterator l) {
  return iterator_pair<Iterator>(f, l);
}

// Use special aliasing constructor for wrapping raw pointer into a shared pointer
// See http://www.codesynthesis.com/~boris/blog/2012/04/25/shared-ptr-aliasing-constructor/
template <typename S,typename T>
std::shared_ptr<S> ptr_to_shared(T* raw) {
  return std::shared_ptr<S>(std::shared_ptr<S>(), raw);
}

//! Generic argmax function returns index of element in C that maximizes F(c_i)
template<typename C, typename F>
size_t argmax(const C& c, F&& eval) {
  if (std::begin(c) == std::end(c)) {
    throw std::invalid_argument("empty container is not allowed.");
  }
  typedef decltype(*std::begin(c)) V;
  auto cmp = [&] (const V& a, const V& b) { return eval(a) < eval(b); };
  const auto pMax = std::max_element(std::begin(c), std::end(c), cmp);
  const size_t iMax = std::distance(std::begin(c), pMax);
  return iMax;
}

//! Generic argmin function returns index of element in C that maximizes F(c_i)
template<typename C, typename F>
size_t argmin(const C& c, F&& eval) {
  if (std::begin(c) == std::end(c)) {
    throw std::invalid_argument("empty container is not allowed.");
  }
  typedef decltype(*std::begin(c)) V;
  auto cmp = [&] (const V& a, const V& b) { return eval(a) > eval(b); };
  const auto pMax = std::max_element(std::begin(c), std::end(c), cmp);
  const size_t iMax = std::distance(std::begin(c), pMax);
  return iMax;
}

}  // namespace util
}  // namespace sg


