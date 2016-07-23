#include "jace/proxy/java/lang/Comparable.h"

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
#include "jace/proxy/types/JInt.h"

BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class source for java/lang/Comparable.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Comparable_INITIALIZER

::jace::proxy::types::JInt Comparable::compareTo(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("compareTo").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Comparable::Comparable()
{}

Comparable::Comparable(jvalue value) Comparable_INITIALIZER
{
  setJavaJniValue(value);
}

Comparable::Comparable(jobject object) Comparable_INITIALIZER
{
  setJavaJniObject(object);
}

Comparable::Comparable(const Comparable& object) Comparable_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Comparable::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/Comparable"));
  return *result;
}

const JClass& Comparable::getJavaJniClass() const throw (::jace::JNIException)
{
  return Comparable::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::lang::Comparable >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::Comparable(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::Comparable >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Comparable >& proxy): 
    ::jace::proxy::java::lang::Comparable(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Comparable >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::Comparable(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Comparable >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::Comparable(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Comparable >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::Comparable >& object): 
    ::jace::proxy::java::lang::Comparable(object)
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

