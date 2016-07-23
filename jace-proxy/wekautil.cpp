#include "./wekautil.h"

#include <string>
#include <exception>
#include <iostream>

#include "jace/Jace.h"
#include "jace/StaticVmLoader.h"
#include "jace/OptionList.h"
#include "jace/JArray.h"
#include "jace/JNIException.h"
#include "jace/VirtualMachineShutdownError.h"

#include "jace/proxy/java/io/IOException.h"
#include "jace/proxy/java/lang/String.h"
#include "jace/proxy/edu/stanford/graphics/wekautils/Classer.h"

namespace wekautil {

int handleJaceExceptions() {
  using std::cout;
  using std::endl;
  try {
    // Rethrow any passed in exception
    throw;
  } catch (jace::VirtualMachineShutdownError&) {
    cout << "The JVM was terminated in mid-execution. " << endl;
    return -1;
  } catch (jace::JNIException& jniException) {
    cout << "An unexpected JNI error has occurred: " << jniException.what() << endl;
    return -2;
  } catch (jace::proxy::java::lang::Throwable& t) {
    t.printStackTrace();
    return -3;
  } catch (std::exception& e) {
    cout << "An unexpected C++ error has occurred: " << e.what() << endl;
    return -4;
  }
}

int initJVM() {
  try {
    // Check whether JVM is already running and return
    if (jace::getJavaVm() != nullptr) {
      //std::cerr << "WARNING: Tried to initialize JVM but it was already running" << std::endl;
      return 0;
    }

    // Boot up JVM with appropriate classpath for dependencies
    jace::StaticVmLoader loader(JNI_VERSION_1_6);
    jace::OptionList list;
    //list.push_back(jace::CustomOption("-Xcheck:jni"));
    list.push_back(jace::CustomOption("-Xmx1024M"));
    list.push_back(jace::ClassPath("../jace-runtime.jar;../../JavaApps/target/SceneGrok-0.0.1.jar;../../JavaApps/target/SceneGrok-deps-0.0.1.jar"));
    jace::createVm(loader, list, false);
    return 0;
  } catch (...) {
    return handleJaceExceptions();
  }
}



// Tests jace proxy and makes sure relevant functions are pulled in
int test() {
  using std::cout;
  using std::endl;
  using namespace jace::proxy;
  using namespace jace::proxy::edu::stanford::graphics;
  try {
    // Load up training set
    const std::string classerDir = "V:/data/scenegrok/classifiers/";
    const std::string classerCSV = "feet_segment-pose.csv";
    const std::string classerModelFile = "test.model";
    wekautils::Classer cls = jace::java_new<wekautils::Classer>("RandomForest", "", classerDir + classerCSV);

    // Train and save classifier
    cls.train(classerDir + classerCSV);
    cls.save(classerModelFile);

    // Test classifier
    wekautils::Classer cls2 = jace::java_new<wekautils::Classer>("RandomForest", "", "");
    cls.load(classerModelFile);
    cls.classify(classerDir + "walk_segment-pose.csv");

    cout << "Press any key...";
    int c = getchar();
    return 0;
  } catch (...) {
    return handleJaceExceptions();
  }
}

bool getClassifierWeights(Classifier* pClassifier, std::vector<double>* pWeights) {
  try {
    JNIEnv* env = jace::attach();
    auto& javaArr = pClassifier->getWeightsForPositiveClass();
    pWeights->clear();
    pWeights->resize(javaArr.length());    
    for (int i = 0; i < javaArr.length(); i++) {
      (*pWeights)[i] = javaArr[i];
    }
    return true;
  } catch (...) {
    wekautil::handleJaceExceptions();
    return false;
  }
}
std::vector<double> getKmeansCentroids(const std::string& csvFile, const size_t numCentroids) {
  const auto javaArr = wekautil::Clustering::getKmeansCentroids(csvFile, static_cast<jint>(numCentroids));
  std::vector<double> out(javaArr.length());
  std::memcpy(&out[0], &javaArr[0], javaArr.length());
  return out;
}

std::vector<double> getKmeansCentroids(const std::vector<double>& rawFeats, const size_t numFeats, const size_t numInstances, const size_t numCentroids) {
  assert(numFeats == rawFeats.size() / numInstances);

  JNIEnv* env = jace::attach();
  jdoubleArray featsArr = env->NewDoubleArray(static_cast<jsize>(rawFeats.size()));
  jsize featsSize = static_cast<jsize>(rawFeats.size());
  env->SetDoubleArrayRegion(featsArr, 0, featsSize, &rawFeats[0]);
  
  auto javaArr = wekautil::Clustering::getKmeansCentroids(featsArr, static_cast<jint>(numFeats), static_cast<jint>(numInstances), static_cast<jint>(numCentroids));
  env->DeleteLocalRef(featsArr);
  
  std::vector<double> out(javaArr.length());
  std::copy(javaArr.begin(), javaArr.end(), out.begin());
  
  return out;
}

}  // namespace weka
