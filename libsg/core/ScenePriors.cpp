#include "common.h"  // NOLINT

#include "core/ScenePriors.h"

#include "core/Database.h"
#include "io/io.h"

namespace sg {
namespace core {

void ScenePriors::init(Database* pDatabase) {
  m_pDatabase = pDatabase;

  const util::Params& params = *pDatabase->params;
  const string& dataDir = params.get<string>("dataDir");

  const string& supportParentGivenChildCsv = dataDir + params.get<string>("supportParentGivenChildFile");
  m_parentGivenChildSupportPriors.load(supportParentGivenChildCsv);

  const string& supportChildGivenParentCsv = dataDir + params.get<string>("supportChildGivenParentFile");
  m_childGivenParentSupportPriors.load(supportChildGivenParentCsv);

  const string& subcategoriesFilename = dataDir + params.get<string>("subcategoriesFile");
  m_categoryHierarchy.load(subcategoriesFilename);
}

#if 1
void ScenePriors::getSceneTypeProbs(const vec<string>& nouns, stats::Counter<string, float>* pCounter) const {
  const auto& stats = m_pDatabase->interactions.getNounSceneTypeStats();
  vec<double> probs(stats.size2());
  double total = 0.0;//nouns.size();
  for (const string& n : nouns) {
    int i = stats.index1().indexOf(n);
    if (i >= 0) {
      for (int j = 0; j < stats.size2(); ++j) {
//        probs[j] += stats.condProb12(i, j);
        probs[j] += stats.rawCooccurrenceCounts(i,j);
      }
      total += stats.rawCounts1[i];
    }
  }
  pCounter->clear();
  for (int j = 0; j < stats.size2(); ++j) {
    pCounter->set( stats.index2()[j], static_cast<float>(probs[j]/total) );
  }
}
#else
void ScenePriors::getSceneTypeProbs(const vec<string>& nouns, stats::Counter<string, float>* pCounter) const {
  const auto& stats = m_pDatabase->interactions.getNounSceneTypeStats();
  vec<double> logprobs(stats.size2());
  // Don't worry about marginals since our dataset don't reflect real priors
  //for (int j = 0; j < stats.size2(); ++j) {
  //  logProbs[j] = stats.marginalLogProb2[j];
  //}
  for (const string& n : nouns) {
    int i = stats.index1().indexOf(n);
    if (i >= 0) {
      for (int j = 0; j < stats.size2(); ++j) {
        logprobs[j] += log(stats.condProb21(i, j));
      }
    }
  }
  pCounter->clear();
  math::normalizeLog<double>(logprobs);
  for (int j = 0; j < stats.size2(); ++j) {
    pCounter->set(stats.index2()[j], static_cast<float>(exp(logprobs[j])));
  }
}
#endif


bool CategoryHierarchy::load(const string& filename) { 
  SG_LOG_INFO << "Loading CategoryHierarchy from " << filename;
  if (!io::fileExists(filename)) {
    SG_LOG_ERROR << "Cannot load file " << filename;
    return false;
  }
  // Load from file
  vec<string> lines = io::getLines(filename);
  vec<string> parentSubcats;
  vec<string> subcats;
  map<string, string> subCatToParent;
  map<string, vec<string>> parentToChildren;
  for (string& line : lines) {
    util::trim_in_place(line);
    if (line.empty() || line[0] == '#') {
      // Skip comments and blank lines
      continue;
    }
    util::tokenize(line, "\t ", 2, false, &parentSubcats);
    if (parentSubcats.size() > 1) {      
      const string& parentCat = parentSubcats[0];
      const string& subCats = parentSubcats[1];
      if (subCatToParent.count(parentCat) == 0) {
        subCatToParent[parentCat] = "";
      }
      util::tokenize(subCats, ",", &subcats);
      for (string& subcat : subcats) {
        util::trim_in_place(subcat);
        subCatToParent[subcat] = parentCat;
        parentToChildren[parentCat].push_back(subcat);
      }
    } else {
      const string& parentCat = line;
      if (parentCat == "") {
        SG_LOG_DEBUG << "debug";
      }
      if (subCatToParent.count(parentCat) == 0) {
        subCatToParent[parentCat] = "";
      }
    }
  }
  // Put parents and children into the index (with parents having smaller indices)
  vec<string> roots;
  for (const auto& it : subCatToParent) {
    if (it.second.empty()) {
      roots.push_back(it.first);
    }
  }
  std::queue<string> todo;
  for (const string& root : roots) {
    todo.push(root);
  }
  m_childToParentCat.resize(subCatToParent.size());
  fill(m_childToParentCat.begin(), m_childToParentCat.end(), -1);
  m_parentToChildCats.resize(subCatToParent.size());
  while (!todo.empty()) {
    const string cat = todo.front();
    todo.pop();
    int iCat = m_categoryIndex.add(cat);
    const string& parentCat = subCatToParent[cat];
    int iParentCat = parentCat.empty()? -1 : m_categoryIndex.indexOf(parentCat, true);
    m_childToParentCat[iCat] = iParentCat;
    if (iParentCat >= 0) {
      m_parentToChildCats[iParentCat].push_back(iCat);
    }
    if (parentToChildren.count(cat) > 0) {
      for (const string& childCat : parentToChildren[cat]) {
        todo.push(childCat);
      }
    }
  }
  return true;
}

vec<string> CategoryHierarchy::getAncestors(const string& cat, bool excludeSelf) const {
  vec<string> ancestors;
  int iCat = m_categoryIndex.indexOf(cat);
  int iParentCat = iCat;
  while (iParentCat >= 0) {
    if (excludeSelf && iCat == iParentCat) {
      // Don't include
    } else {
      ancestors.push_back(m_categoryIndex[iParentCat]);
    }
    // Follow the links up
    iParentCat = m_childToParentCat[iParentCat];
  } 
  return ancestors;
}

bool CategoryCounts::load(const string& filename) {
  SG_LOG_INFO << "Loading CategoryCounts from " << filename;
  if (!io::fileExists(filename)) {
    SG_LOG_ERROR << "Cannot load file " << filename;
    return false;
  }
  // Load from file
  vec<string> lines = io::getLines(filename);
  vec<string> fields;
  // Skip first line (header)
  entries.reserve(lines.size());
  for (int i = 1; i < lines.size(); ++i) {
    util::tokenize(lines[i], ",", &fields);
    if (fields.size() >= 5) {
      for (string& field : fields) {
        if (field[0] == '\"' && field[field.length()-1] == '\"') {
          field = field.substr(1, field.length()-2);
        }
      }
      entries.push_back({fields[0], fields[1], stoi(fields[2]), stoi(fields[3]), stof(fields[4])});
    } else {
      SG_LOG_ERROR << "Error processing " << filename << ":" << i << " - Got " << fields.size() << " fields";
    }    
  }

  // Populate convenience maps
  for (int i = 0; i < entries.size(); ++i) {
    const Entry& entry = entries[i];
    cat1Map[entry.cat1].push_back(i);
    cat2Map[entry.cat2].push_back(i);
  }
  return true;
}


void CategoryCounts::getCounterByCat1(const string& cat1, const set<string>& filterCats, stats::Counter<string, float>* pCounter) const {
  pCounter->clear();
  if (cat1Map.count(cat1) > 0) {
    const vec<int> indices = cat1Map.at(cat1);
    for (int i : indices) {
      const Entry& entry = entries.at(i);
      if (filterCats.empty() || filterCats.count(entry.cat2) > 0) {
        pCounter->set(entry.cat2, entry.value);
      }
    }
  }
}

void CategoryCounts::getCounterByCat2(const string& cat2, const set<string>& filterCats, stats::Counter<string, float>* pCounter) const {
  pCounter->clear();
  if (cat2Map.count(cat2) > 0) {
    const vec<int> indices = cat2Map.at(cat2);
    for (int i : indices) {
      const Entry& entry = entries.at(i);
      if (filterCats.empty() || filterCats.count(entry.cat1) > 0) {
        pCounter->set(entry.cat1, entry.value);
      }
    }
  }
}

}  // namespace core
}  // namespace sg

