#pragma once

#include "libsg.h"  // NOLINT

#include "stats/Counter.h"
#include "util/Index.h"

namespace sg {
namespace core {

class CategoryCounts { 
 public:
  bool load(const string& filename);
  void getCounterByCat1(const string& cat1, 
                        const set<string>& filterCats,
                        stats::Counter<string,float>* pCounter) const;
  void getCounterByCat2(const string& cat2, 
                        const set<string>& filterCats,
                        stats::Counter<string,float>* pCounter) const;

  // For now just have a vector of entries
  struct Entry {
    string cat1;
    string cat2;
    int count1;
    int count2;
    float value;
  };

 private:
  // Vector of entries
  vec<Entry> entries;
  // Convenience maps for looking up entries using cat1 or cat2
  map<string, vec<int>>  cat1Map;
  map<string, vec<int>>  cat2Map;
};

class CategoryHierarchy {
 public:
  bool load(const string& filename);
  //! Get ancestors (from self to top category)
  vec<string> getAncestors(const string& cat, bool excludeSelf = false) const;
  const util::Index<string>& getCategoryIndex() const {
    return m_categoryIndex;
  }

 private:
  //! Index of category name to int (parents should have lower index than children)
  util::Index<string> m_categoryIndex;
  //! Vector of child to parent categories (index based)
  vec<int> m_childToParentCat; 
  //! Vector of parent to child categories (index based)
  vec<vec<int>> m_parentToChildCats;
};

//! Hold information about priors on objects learned from scenes
class ScenePriors {
 public:
  //! Initializes the scene priors
  void init(Database* db);
  void getSceneTypeProbs(const vec<string>& cats, stats::Counter<string,float>* pCounter) const;
  void getParentGivenChildProbs(const string& childCat, stats::Counter<string,float>* pCounter) const {
    m_parentGivenChildSupportPriors.getCounterByCat2(childCat, set<string>(), pCounter);
  }
  void getParentGivenChildProbs(const string& childCat, 
                                const set<string>& filterCategories, 
                                stats::Counter<string,float>* pCounter) const {
    m_parentGivenChildSupportPriors.getCounterByCat2(childCat, filterCategories, pCounter);
  }

  void getChildGivenParentProbs(const string& parentCat, stats::Counter<string,float>* pCounter) const {
    m_childGivenParentSupportPriors.getCounterByCat1(parentCat, set<string>(), pCounter);
  }
  void getChildGivenParentProbs(const string& parentCat, 
                                const set<string>& filterCategories, 
                                stats::Counter<string,float>* pCounter) const {
    m_childGivenParentSupportPriors.getCounterByCat1(parentCat, filterCategories, pCounter);
  }

  const CategoryHierarchy& getCategoryHierarchy() const {
    return m_categoryHierarchy;
  }

 private:
  //! Database from which we get scans and compute scene priors
  Database* m_pDatabase = nullptr;

  //! Category hierarchy
  CategoryHierarchy m_categoryHierarchy;
  //! Parent given child support priors
  //! Cat1 is parent, Cat2 is child, value is P(parent|child)
  CategoryCounts m_parentGivenChildSupportPriors;
  //! Child given parent support priors
  //! Cat1 is parent, Cat2 is child, value is P(child|parent)
  CategoryCounts m_childGivenParentSupportPriors;
};

}  // namespace core
}  // namespace sg




