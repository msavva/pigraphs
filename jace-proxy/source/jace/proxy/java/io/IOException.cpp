#include "jace/proxy/java/io/IOException.h"

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
#include "jace/proxy/java/lang/Throwable.h"
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_4(jace, proxy, java, io)

/**
 * The Jace C++ proxy class source for java/io/IOException.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define IOException_INITIALIZER : ::jace::proxy::java::lang::Exception()

IOException IOException::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(IOException::staticGetJavaJniClass(), arguments);
  IOException result = IOException(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

IOException IOException::Factory::create(::jace::proxy::java::lang::String p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(IOException::staticGetJavaJniClass(), arguments);
  IOException result = IOException(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

IOException IOException::Factory::create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::Throwable p1){
  JArguments arguments;
  arguments << p0 << p1;
  jobject localRef = newObject(IOException::staticGetJavaJniClass(), arguments);
  IOException result = IOException(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

IOException IOException::Factory::create(::jace::proxy::java::lang::Throwable p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(IOException::staticGetJavaJniClass(), arguments);
  IOException result = IOException(localRef);
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
IOException::IOException()
{}

IOException::IOException(jvalue value) IOException_INITIALIZER
{
  setJavaJniValue(value);
}

IOException::IOException(jobject object) IOException_INITIALIZER
{
  setJavaJniObject(object);
}

IOException::IOException(const IOException& object) IOException_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& IOException::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/io/IOException"));
  return *result;
}

const JClass& IOException::getJavaJniClass() const throw (::jace::JNIException)
{
  return IOException::staticGetJavaJniClass();
}

JEnlister< IOException > IOException::enlister;

END_NAMESPACE_4(jace, proxy, java, io)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::io::IOException >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::io::IOException(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::io::IOException >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::io::IOException >& proxy): 
    ::jace::proxy::java::io::IOException(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::io::IOException >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::io::IOException(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::io::IOException >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::io::IOException(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::io::IOException >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::io::IOException >& object): 
    ::jace::proxy::java::io::IOException(object)
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

