#include "jace/proxy/java/lang/Throwable.h"

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
 * The Jace C++ proxy class source for java/lang/Throwable.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Throwable_INITIALIZER

Throwable Throwable::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(Throwable::staticGetJavaJniClass(), arguments);
  Throwable result = Throwable(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Throwable Throwable::Factory::create(::jace::proxy::java::lang::String p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Throwable::staticGetJavaJniClass(), arguments);
  Throwable result = Throwable(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Throwable Throwable::Factory::create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::Throwable p1){
  JArguments arguments;
  arguments << p0 << p1;
  jobject localRef = newObject(Throwable::staticGetJavaJniClass(), arguments);
  Throwable result = Throwable(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Throwable Throwable::Factory::create(::jace::proxy::java::lang::Throwable p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Throwable::staticGetJavaJniClass(), arguments);
  Throwable result = Throwable(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::proxy::java::lang::String Throwable::getMessage()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("getMessage").invoke(*this, arguments);
}

::jace::proxy::java::lang::String Throwable::getLocalizedMessage()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("getLocalizedMessage").invoke(*this, arguments);
}

::jace::proxy::java::lang::Throwable Throwable::getCause()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::Throwable >("getCause").invoke(*this, arguments);
}

::jace::proxy::java::lang::Throwable Throwable::initCause(::jace::proxy::java::lang::Throwable p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Throwable >("initCause").invoke(*this, arguments);
}

::jace::proxy::java::lang::String Throwable::toString()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("toString").invoke(*this, arguments);
}

void Throwable::printStackTrace()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("printStackTrace").invoke(*this, arguments);
}

::jace::proxy::java::lang::Throwable Throwable::fillInStackTrace()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::Throwable >("fillInStackTrace").invoke(*this, arguments);
}

void Throwable::addSuppressed(::jace::proxy::java::lang::Throwable p0)
{
  JArguments arguments;
  arguments << p0;
  JMethod< ::jace::proxy::types::JVoid >("addSuppressed").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Throwable > Throwable::getSuppressed()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Throwable > >("getSuppressed").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Throwable::Throwable()
{}

Throwable::Throwable(jvalue value) Throwable_INITIALIZER
{
  setJavaJniValue(value);
}

Throwable::Throwable(jobject object) Throwable_INITIALIZER
{
  setJavaJniObject(object);
}

Throwable::Throwable(const Throwable& object) Throwable_INITIALIZER
{
  setJavaJniObject(object);
}

Throwable::~Throwable() throw ()
{}

const char* Throwable::what() const throw()
{
  // JACE really isn't const correct like it should be, yet.
  // For now, the easiest way to get around this is to cast constness away.
  Throwable* t = const_cast<Throwable*>(this);

  /* Get the string contents of this exception.
   */
  t->msg = t->toString();

  /* Return a handle to the msg.
   */
  return t->msg.c_str();
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Throwable::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/Throwable"));
  return *result;
}

const JClass& Throwable::getJavaJniClass() const throw (::jace::JNIException)
{
  return Throwable::staticGetJavaJniClass();
}

JEnlister< Throwable > Throwable::enlister;

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::lang::Throwable >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::Throwable(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::Throwable >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Throwable >& proxy): 
    ::jace::proxy::java::lang::Throwable(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Throwable >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::Throwable(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Throwable >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::Throwable(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Throwable >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::Throwable >& object): 
    ::jace::proxy::java::lang::Throwable(object)
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

