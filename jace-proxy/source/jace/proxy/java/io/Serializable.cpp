#include "jace/proxy/java/io/Serializable.h"

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

BEGIN_NAMESPACE_4(jace, proxy, java, io)

/**
 * The Jace C++ proxy class source for java/io/Serializable.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Serializable_INITIALIZER

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Serializable::Serializable()
{}

Serializable::Serializable(jvalue value) Serializable_INITIALIZER
{
  setJavaJniValue(value);
}

Serializable::Serializable(jobject object) Serializable_INITIALIZER
{
  setJavaJniObject(object);
}

Serializable::Serializable(const Serializable& object) Serializable_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Serializable::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/io/Serializable"));
  return *result;
}

const JClass& Serializable::getJavaJniClass() const throw (::jace::JNIException)
{
  return Serializable::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, io)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::io::Serializable >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::io::Serializable(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::io::Serializable >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::io::Serializable >& proxy): 
    ::jace::proxy::java::io::Serializable(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::io::Serializable >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::io::Serializable(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::io::Serializable >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::io::Serializable(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::io::Serializable >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::io::Serializable >& object): 
    ::jace::proxy::java::io::Serializable(object)
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

