#include "jace/proxy/edu/stanford/graphics/wekautils/Classer.h"

/**
 * Standard Jace headers needed to implement this class.
 */
#include "jace/JArguments.h"
#include "jace/JMethod.h"
#include "jace/JField.h"
#include "jace/JClassImpl.h"
#include "jace/BoostWarningOff.h"
#include <boost/thread/mutex.hpp>
#include "jace/BoostWarningOn.h"

/**
 * Headers for the classes that this class uses.
 */
#include "jace/proxy/types/JDouble.h"
#include "jace/proxy/types/JBoolean.h"
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/java/lang/String.h"
#include "jace/proxy/java/lang/Exception.h"

BEGIN_NAMESPACE_6(jace, proxy, edu, stanford, graphics, wekautils)

/**
 * The Jace C++ proxy class source for edu/stanford/graphics/wekautils/Classer.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Classer_INITIALIZER

Classer Classer::Factory::create(::jace::proxy::java::lang::String p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Classer::staticGetJavaJniClass(), arguments);
  Classer result = Classer(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Classer Classer::Factory::create(::jace::proxy::java::lang::String p0, ::jace::JArray< ::jace::proxy::java::lang::String > p1, ::jace::proxy::java::lang::String p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(Classer::staticGetJavaJniClass(), arguments);
  Classer result = Classer(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Classer Classer::Factory::create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(Classer::staticGetJavaJniClass(), arguments);
  Classer result = Classer(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::JArray< ::jace::proxy::types::JDouble > Classer::getWeightsForPositiveClass()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JDouble > >("getWeightsForPositiveClass").invoke(*this, arguments);
}

::jace::JArray< ::jace::JArray< ::jace::proxy::types::JDouble > > Classer::getWeights()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::JArray< ::jace::proxy::types::JDouble > > >("getWeights").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Classer::headerMatches(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("headerMatches").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Classer::train(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("train").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Classer::train(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JDouble p1, ::jace::proxy::types::JDouble p2)
{
  JArguments arguments;
  arguments << p0 << p1 << p2;
  return JMethod< ::jace::proxy::types::JBoolean >("train").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Classer::save(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("save").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Classer::load(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("load").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Classer::classify(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("classify").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::types::JDouble > Classer::classifyMulti(::jace::JArray< ::jace::JArray< ::jace::proxy::types::JDouble > > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JDouble > >("classifyMulti").invoke(*this, arguments);
}

::jace::proxy::types::JDouble Classer::classify(::jace::JArray< ::jace::proxy::types::JDouble > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JDouble >("classify").invoke(*this, arguments);
}

void Classer::reportCrossValidation(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2)
{
  JArguments arguments;
  arguments << p0 << p1 << p2;
  JMethod< ::jace::proxy::types::JVoid >("reportCrossValidation").invoke(staticGetJavaJniClass(), arguments);
}

void Classer::test(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2, ::jace::JArray< ::jace::proxy::types::JDouble > p3)
{
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  JMethod< ::jace::proxy::types::JVoid >("test").invoke(*this, arguments);
}

void Classer::reportAllTestResults(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2)
{
  JArguments arguments;
  arguments << p0 << p1 << p2;
  JMethod< ::jace::proxy::types::JVoid >("reportAllTestResults").invoke(staticGetJavaJniClass(), arguments);
}

void Classer::main(::jace::JArray< ::jace::proxy::java::lang::String > p0)
{
  JArguments arguments;
  arguments << p0;
  JMethod< ::jace::proxy::types::JVoid >("main").invoke(staticGetJavaJniClass(), arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Classer::Classer()
{}

Classer::Classer(jvalue value) Classer_INITIALIZER
{
  setJavaJniValue(value);
}

Classer::Classer(jobject object) Classer_INITIALIZER
{
  setJavaJniObject(object);
}

Classer::Classer(const Classer& object) Classer_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * public static final RANDOM_FOREST
 */
::jace::JFieldProxy< ::jace::proxy::java::lang::String > Classer::RANDOM_FOREST()
{
  return ::jace::JField< ::jace::proxy::java::lang::String >("RANDOM_FOREST").get(staticGetJavaJniClass());
}

/**
 * public static final SMO
 */
::jace::JFieldProxy< ::jace::proxy::java::lang::String > Classer::SMO()
{
  return ::jace::JField< ::jace::proxy::java::lang::String >("SMO").get(staticGetJavaJniClass());
}

/**
 * public static final LOGISTIC
 */
::jace::JFieldProxy< ::jace::proxy::java::lang::String > Classer::LOGISTIC()
{
  return ::jace::JField< ::jace::proxy::java::lang::String >("LOGISTIC").get(staticGetJavaJniClass());
}

/**
 * public static final DEFAULT_CLASSIFIER_TYPE
 */
::jace::JFieldProxy< ::jace::proxy::java::lang::String > Classer::DEFAULT_CLASSIFIER_TYPE()
{
  return ::jace::JField< ::jace::proxy::java::lang::String >("DEFAULT_CLASSIFIER_TYPE").get(staticGetJavaJniClass());
}

/**
 * public classifierType
 */
::jace::JFieldProxy< ::jace::proxy::java::lang::String > Classer::classifierType()
{
  return ::jace::JField< ::jace::proxy::java::lang::String >("classifierType").get(*this);
}

/**
 * public id
 */
::jace::JFieldProxy< ::jace::proxy::java::lang::String > Classer::id()
{
  return ::jace::JField< ::jace::proxy::java::lang::String >("id").get(*this);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Classer::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("edu/stanford/graphics/wekautils/Classer"));
  return *result;
}

const JClass& Classer::getJavaJniClass() const throw (::jace::JNIException)
{
  return Classer::staticGetJavaJniClass();
}

END_NAMESPACE_6(jace, proxy, edu, stanford, graphics, wekautils)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
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

