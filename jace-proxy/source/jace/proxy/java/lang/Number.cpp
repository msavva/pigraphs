#include "jace/proxy/java/lang/Number.h"

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
#include "jace/proxy/types/JByte.h"
#include "jace/proxy/types/JLong.h"
#include "jace/proxy/types/JFloat.h"
#include "jace/proxy/types/JDouble.h"
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/types/JShort.h"

BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class source for java/lang/Number.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Number_INITIALIZER

Number Number::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(Number::staticGetJavaJniClass(), arguments);
  Number result = Number(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::proxy::types::JInt Number::intValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("intValue").invoke(*this, arguments);
}

::jace::proxy::types::JLong Number::longValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JLong >("longValue").invoke(*this, arguments);
}

::jace::proxy::types::JFloat Number::floatValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JFloat >("floatValue").invoke(*this, arguments);
}

::jace::proxy::types::JDouble Number::doubleValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JDouble >("doubleValue").invoke(*this, arguments);
}

::jace::proxy::types::JByte Number::byteValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JByte >("byteValue").invoke(*this, arguments);
}

::jace::proxy::types::JShort Number::shortValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JShort >("shortValue").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Number::Number()
{}

Number::Number(jvalue value) Number_INITIALIZER
{
  setJavaJniValue(value);
}

Number::Number(jobject object) Number_INITIALIZER
{
  setJavaJniObject(object);
}

Number::Number(const Number& object) Number_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Number::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/Number"));
  return *result;
}

const JClass& Number::getJavaJniClass() const throw (::jace::JNIException)
{
  return Number::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::lang::Number >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::Number(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::Number >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Number >& proxy): 
    ::jace::proxy::java::lang::Number(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Number >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::Number(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Number >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::Number(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Number >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::Number >& object): 
    ::jace::proxy::java::lang::Number(object)
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

