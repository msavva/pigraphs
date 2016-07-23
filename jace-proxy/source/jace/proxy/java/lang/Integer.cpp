#include "jace/proxy/java/lang/Integer.h"

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
#include "jace/proxy/types/JBoolean.h"
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/java/lang/Object.h"
#include "jace/proxy/java/lang/String.h"
#include "jace/proxy/types/JShort.h"

BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class source for java/lang/Integer.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Integer_INITIALIZER : ::jace::proxy::java::lang::Number()

::jace::proxy::java::lang::String Integer::toString(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("toString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String Integer::toUnsignedString(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("toUnsignedString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String Integer::toHexString(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("toHexString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String Integer::toOctalString(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("toOctalString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String Integer::toBinaryString(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("toBinaryString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String Integer::toString(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("toString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String Integer::toUnsignedString(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("toUnsignedString").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::parseInt(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("parseInt").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::parseInt(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("parseInt").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::parseUnsignedInt(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("parseUnsignedInt").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::parseUnsignedInt(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("parseUnsignedInt").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::Integer Integer::valueOf(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::Integer >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::Integer Integer::valueOf(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Integer >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::Integer Integer::valueOf(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Integer >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

Integer Integer::Factory::create(::jace::proxy::types::JInt p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Integer::staticGetJavaJniClass(), arguments);
  Integer result = Integer(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Integer Integer::Factory::create(::jace::proxy::java::lang::String p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(Integer::staticGetJavaJniClass(), arguments);
  Integer result = Integer(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::proxy::types::JByte Integer::byteValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JByte >("byteValue").invoke(*this, arguments);
}

::jace::proxy::types::JShort Integer::shortValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JShort >("shortValue").invoke(*this, arguments);
}

::jace::proxy::types::JInt Integer::intValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("intValue").invoke(*this, arguments);
}

::jace::proxy::types::JLong Integer::longValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JLong >("longValue").invoke(*this, arguments);
}

::jace::proxy::types::JFloat Integer::floatValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JFloat >("floatValue").invoke(*this, arguments);
}

::jace::proxy::types::JDouble Integer::doubleValue()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JDouble >("doubleValue").invoke(*this, arguments);
}

::jace::proxy::java::lang::String Integer::toString()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("toString").invoke(*this, arguments);
}

::jace::proxy::types::JInt Integer::hashCode()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("hashCode").invoke(*this, arguments);
}

::jace::proxy::types::JInt Integer::hashCode(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("hashCode").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JBoolean Integer::equals(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equals").invoke(*this, arguments);
}

::jace::proxy::java::lang::Integer Integer::getInteger(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Integer >("getInteger").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::Integer Integer::getInteger(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::Integer >("getInteger").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::Integer Integer::getInteger(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::Integer p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::Integer >("getInteger").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::Integer Integer::decode(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Integer >("decode").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::compareTo(::jace::proxy::java::lang::Integer p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("compareTo").invoke(*this, arguments);
}

::jace::proxy::types::JInt Integer::compare(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("compare").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::compareUnsigned(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("compareUnsigned").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JLong Integer::toUnsignedLong(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JLong >("toUnsignedLong").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::divideUnsigned(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("divideUnsigned").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::remainderUnsigned(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("remainderUnsigned").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::highestOneBit(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("highestOneBit").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::lowestOneBit(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("lowestOneBit").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::numberOfLeadingZeros(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("numberOfLeadingZeros").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::numberOfTrailingZeros(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("numberOfTrailingZeros").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::bitCount(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("bitCount").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::rotateLeft(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("rotateLeft").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::rotateRight(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("rotateRight").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::reverse(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("reverse").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::signum(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("signum").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::reverseBytes(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("reverseBytes").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::sum(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("sum").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::max(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("max").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JInt Integer::min(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("min").invoke(staticGetJavaJniClass(), arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Integer::Integer()
{}

Integer::Integer(jvalue value) Integer_INITIALIZER
{
  setJavaJniValue(value);
}

Integer::Integer(jobject object) Integer_INITIALIZER
{
  setJavaJniObject(object);
}

Integer::Integer(const Integer& object) Integer_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * public static final MIN_VALUE
 */
::jace::JFieldProxy< ::jace::proxy::types::JInt > Integer::MIN_VALUE()
{
  return ::jace::JField< ::jace::proxy::types::JInt >("MIN_VALUE").get(staticGetJavaJniClass());
}

/**
 * public static final MAX_VALUE
 */
::jace::JFieldProxy< ::jace::proxy::types::JInt > Integer::MAX_VALUE()
{
  return ::jace::JField< ::jace::proxy::types::JInt >("MAX_VALUE").get(staticGetJavaJniClass());
}

/**
 * public static final SIZE
 */
::jace::JFieldProxy< ::jace::proxy::types::JInt > Integer::SIZE()
{
  return ::jace::JField< ::jace::proxy::types::JInt >("SIZE").get(staticGetJavaJniClass());
}

/**
 * public static final BYTES
 */
::jace::JFieldProxy< ::jace::proxy::types::JInt > Integer::BYTES()
{
  return ::jace::JField< ::jace::proxy::types::JInt >("BYTES").get(staticGetJavaJniClass());
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Integer::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/Integer"));
  return *result;
}

const JClass& Integer::getJavaJniClass() const throw (::jace::JNIException)
{
  return Integer::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::lang::Integer >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::Integer(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::Integer >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Integer >& proxy): 
    ::jace::proxy::java::lang::Integer(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Integer >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::Integer(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Integer >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::Integer(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::Integer >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::Integer >& object): 
    ::jace::proxy::java::lang::Integer(object)
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

