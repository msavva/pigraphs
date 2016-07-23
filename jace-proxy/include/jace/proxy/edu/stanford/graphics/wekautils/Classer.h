#ifndef JACE_PROXY_EDU_STANFORD_GRAPHICS_WEKAUTILS_CLASSER_H
#define JACE_PROXY_EDU_STANFORD_GRAPHICS_WEKAUTILS_CLASSER_H

#include "jace/OsDep.h"
#include "jace/Namespace.h"
#include "jace/proxy/JObject.h"
#include "jace/JArray.h"
#include "jace/JFieldProxy.h"
#include "jace/JMethod.h"
#include "jace/JField.h"
#include "jace/JClassImpl.h"

/**
 * The super class for this class.
 */
#include "jace/proxy/java/lang/Object.h"
/**
 * Forward declarations for the classes that this class uses.
 */
BEGIN_NAMESPACE_3(jace, proxy, types)
class JDouble;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JBoolean;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JVoid;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_4(jace, proxy, java, lang)
class String;
END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE_4(jace, proxy, java, lang)
class Exception;
END_NAMESPACE_4(jace, proxy, java, lang)


BEGIN_NAMESPACE_6(jace, proxy, edu, stanford, graphics, wekautils)

/**
 * The Jace C++ proxy class for edu.stanford.graphics.wekautils.Classer.
 * Please do not edit this class, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
class Classer: public virtual ::jace::proxy::java::lang::Object
{
  public:
  class Factory
  {
  public:
    /**
     * Creates a new Classer.
     */
    JACE_PROXY_API static Classer create(::jace::proxy::java::lang::String p0);
    /**
     * Creates a new Classer.
     */
    JACE_PROXY_API static Classer create(::jace::proxy::java::lang::String p0, ::jace::JArray< ::jace::proxy::java::lang::String > p1, ::jace::proxy::java::lang::String p2);
    /**
     * Creates a new Classer.
     */
    JACE_PROXY_API static Classer create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2);
  };
  
  public: 
  /**
   * Creates a new null reference.
   * 
   * All subclasses of JObject should provide this constructor
   * for their own subclasses.
   */
  JACE_PROXY_API explicit Classer();
  /**
   * Copy an existing reference.
   */
  JACE_PROXY_API Classer(const Classer&);
  /**
   * getWeightsForPositiveClass
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::types::JDouble > getWeightsForPositiveClass();
  /**
   * getWeights
   */
  JACE_PROXY_API ::jace::JArray< ::jace::JArray< ::jace::proxy::types::JDouble > > getWeights();
  /**
   * headerMatches
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean headerMatches(::jace::proxy::java::lang::String p0);
  /**
   * train
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean train(::jace::proxy::java::lang::String p0);
  /**
   * train
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean train(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JDouble p1, ::jace::proxy::types::JDouble p2);
  /**
   * save
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean save(::jace::proxy::java::lang::String p0);
  /**
   * load
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean load(::jace::proxy::java::lang::String p0);
  /**
   * classify
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean classify(::jace::proxy::java::lang::String p0);
  /**
   * classifyMulti
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::types::JDouble > classifyMulti(::jace::JArray< ::jace::JArray< ::jace::proxy::types::JDouble > > p0);
  /**
   * classify
   */
  JACE_PROXY_API ::jace::proxy::types::JDouble classify(::jace::JArray< ::jace::proxy::types::JDouble > p0);
  /**
   * reportCrossValidation
   */
  JACE_PROXY_API static void reportCrossValidation(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2);
  /**
   * test
   */
  JACE_PROXY_API void test(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2, ::jace::JArray< ::jace::proxy::types::JDouble > p3);
  /**
   * reportAllTestResults
   */
  JACE_PROXY_API static void reportAllTestResults(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2);
  /**
   * main
   */
  JACE_PROXY_API static void main(::jace::JArray< ::jace::proxy::java::lang::String > p0);
  JACE_PROXY_API virtual const JClass& getJavaJniClass() const throw (::jace::JNIException);
  JACE_PROXY_API static const JClass& staticGetJavaJniClass() throw (::jace::JNIException);
  JACE_PROXY_API explicit Classer(jvalue);
  JACE_PROXY_API explicit Classer(jobject);
  /**
   * public static final RANDOM_FOREST
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::java::lang::String > RANDOM_FOREST();
  
  /**
   * public static final SMO
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::java::lang::String > SMO();
  
  /**
   * public static final LOGISTIC
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::java::lang::String > LOGISTIC();
  
  /**
   * public static final DEFAULT_CLASSIFIER_TYPE
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::java::lang::String > DEFAULT_CLASSIFIER_TYPE();
  
  /**
   * public classifierType
   */
  JACE_PROXY_API ::jace::JFieldProxy< ::jace::proxy::java::lang::String > classifierType();
  
  /**
   * public id
   */
  JACE_PROXY_API ::jace::JFieldProxy< ::jace::proxy::java::lang::String > id();

private:
  /**
   * The following methods are required to integrate this class
   * with the Jace framework.
   */
  template <typename T> friend T (::jace::java_cast)(const ::jace::proxy::JObject&);
  template <typename T> friend T (::jace::java_new)();
  template <typename T, typename A0> friend T (::jace::java_new)(A0 a0);
  template <typename T, typename A0, typename A1> friend T (::jace::java_new)(A0 a0, A1 a1);
  template <typename T, typename A0, typename A1, typename A2> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2);
  template <typename T, typename A0, typename A1, typename A2, typename A3> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3);
  template <typename T, typename A0, typename A1, typename A2, typename A3, typename A4> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3, A4 a4);
  template <typename T, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5);
  template <typename T, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6);
  template <typename T, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7);
  template <typename T, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8);
  template <typename T, typename A0, typename A1, typename A2, typename A3, typename A4, typename A5, typename A6, typename A7, typename A8, typename A9> friend T (::jace::java_new)(A0 a0, A1 a1, A2 a2, A3 a3, A4 a4, A5 a5, A6 a6, A7 a7, A8 a8, A9 a9);
  template <typename T> friend T (::jace::java_new)(const char*);
  template <typename T> friend T (::jace::java_new)(const ::std::string&);
  template <typename T> friend class ::jace::ElementProxy;
  template <typename T> friend class ::jace::JFieldProxy;
  template <typename T> friend class ::jace::JMethod;
};

END_NAMESPACE_6(jace, proxy, edu, stanford, graphics, wekautils)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::ElementProxy(jarray array, jvalue element, int index);
  template <> ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::ElementProxy(const jace::ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >& proxy);
#else
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Classer(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::ElementProxy(const jace::ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >& proxy): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Classer(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif

#ifndef PUT_TSDS_IN_HEADER
  template <> JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent);
  template <> JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass);
  template <> JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::JFieldProxy(const ::jace::JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >& object);
#else
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Classer(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Classer(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >::JFieldProxy(const JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Classer >& object): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Classer(object)
  {
    fieldID = object.fieldID; 

    if (object.parent)
    {
      JNIEnv* env = attach();
      parent = newGlobalRef(env, object.parent);
    }
    else
      parent = 0;

    if (object.parentClass)
    {
      JNIEnv* env = attach();
      parentClass = static_cast<jclass>(newGlobalRef(env, object.parentClass));
    }
    else
      parentClass = 0;
  }
#endif

END_NAMESPACE(jace)

#endif // #ifndef JACE_PROXY_EDU_STANFORD_GRAPHICS_WEKAUTILS_CLASSER_H

