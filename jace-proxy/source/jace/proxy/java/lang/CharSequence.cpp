#include "jace/proxy/java/lang/CharSequence.h"

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
#include "jace/proxy/types/JChar.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class source for java/lang/CharSequence.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define CharSequence_INITIALIZER

::jace::proxy::types::JInt CharSequence::length()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("length").invoke(*this, arguments);
}

::jace::proxy::types::JChar CharSequence::charAt(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JChar >("charAt").invoke(*this, arguments);
}

::jace::proxy::java::lang::CharSequence CharSequence::subSequence(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::CharSequence >("subSequence").invoke(*this, arguments);
}

::jace::proxy::java::lang::String CharSequence::toString()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("toString").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
CharSequence::CharSequence()
{}

CharSequence::CharSequence(jvalue value) CharSequence_INITIALIZER
{
  setJavaJniValue(value);
}

CharSequence::CharSequence(jobject object) CharSequence_INITIALIZER
{
  setJavaJniObject(object);
}

CharSequence::CharSequence(const CharSequence& object) CharSequence_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& CharSequence::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/CharSequence"));
  return *result;
}

const JClass& CharSequence::getJavaJniClass() const throw (::jace::JNIException)
{
  return CharSequence::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::lang::CharSequence >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::CharSequence(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::CharSequence >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::CharSequence >& proxy): 
    ::jace::proxy::java::lang::CharSequence(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::lang::CharSequence >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::CharSequence(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::CharSequence >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::CharSequence(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::CharSequence >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::CharSequence >& object): 
    ::jace::proxy::java::lang::CharSequence(object)
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

