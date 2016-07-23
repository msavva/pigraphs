#include "jace/proxy/java/lang/Exception.h"

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
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class source for java/lang/Exception.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Exception_INITIALIZER : ::jace::proxy::java::lang::Throwable()

Exception Exception::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(Exception::staticGetJavaJniClass(), arguments);
  Exception result = Exception(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Exception Exception::Factory::create(::jace::proxy::java::lang::String p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Exception::staticGetJavaJniClass(), arguments);
  Exception result = Exception(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Exception Exception::Factory::create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::Throwable p1){
  JArguments arguments;
  arguments << p0 << p1;
  jobject localRef = newObject(Exception::staticGetJavaJniClass(), arguments);
  Exception result = Exception(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Exception Exception::Factory::create(::jace::proxy::java::lang::Throwable p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Exception::staticGetJavaJniClass(), arguments);
  Exception result = Exception(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Exception::Exception()
{}

Exception::Exception(jvalue value) Exception_INITIALIZER
{
  setJavaJniValue(value);
}

Exception::Exception(jobject object) Exception_INITIALIZER
{
  setJavaJniObject(object);
}

Exception::Exception(const Exception& object) Exception_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Exception::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/Exception"));
  return *result;
}

const JClass& Exception::getJavaJniClass() const throw (::jace::JNIException)
{
  return Exception::staticGetJavaJniClass();
}

JEnlister< Exception > Exception::enlister;

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::lang::Exception >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::Exception(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::Exception >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Exception >& proxy): 
    ::jace::proxy::java::lang::Exception(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Exception >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::Exception(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Exception >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::Exception(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Exception >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::Exception >& object): 
    ::jace::proxy::java::lang::Exception(object)
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

