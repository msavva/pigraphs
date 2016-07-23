#ifndef JACE_PROXY_JAVA_LANG_INTEGER_H
#define JACE_PROXY_JAVA_LANG_INTEGER_H

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
#include "jace/proxy/java/lang/Number.h"
/**
 * The interfaces implemented by this class.
 */
#include "jace/proxy/java/lang/Comparable.h"
/**
 * Forward declarations for the classes that this class uses.
 */
BEGIN_NAMESPACE_3(jace, proxy, types)
class JByte;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JLong;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JFloat;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JDouble;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JBoolean;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JVoid;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JInt;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_4(jace, proxy, java, lang)
class Object;
END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE_4(jace, proxy, java, lang)
class String;
END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JShort;
END_NAMESPACE_3(jace, proxy, types)


BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class for java.lang.Integer.
 * Please do not edit this class, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
class Integer: public ::jace::proxy::java::lang::Number, public virtual ::jace::proxy::java::lang::Comparable
{
  public:
  class Factory
  {
  public:
    /**
     * Creates a new Integer.
     */
    JACE_PROXY_API static Integer create(::jace::proxy::types::JInt p0);
    /**
     * Creates a new Integer.
     */
    JACE_PROXY_API static Integer create(::jace::proxy::java::lang::String p0);
  };
  
  public: 
  /**
   * Creates a new null reference.
   * 
   * All subclasses of JObject should provide this constructor
   * for their own subclasses.
   */
  JACE_PROXY_API explicit Integer();
  /**
   * Copy an existing reference.
   */
  JACE_PROXY_API Integer(const Integer&);
  /**
   * toString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toString(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * toUnsignedString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toUnsignedString(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * toHexString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toHexString(::jace::proxy::types::JInt p0);
  /**
   * toOctalString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toOctalString(::jace::proxy::types::JInt p0);
  /**
   * toBinaryString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toBinaryString(::jace::proxy::types::JInt p0);
  /**
   * toString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toString(::jace::proxy::types::JInt p0);
  /**
   * toUnsignedString
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String toUnsignedString(::jace::proxy::types::JInt p0);
  /**
   * parseInt
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt parseInt(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * parseInt
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt parseInt(::jace::proxy::java::lang::String p0);
  /**
   * parseUnsignedInt
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt parseUnsignedInt(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * parseUnsignedInt
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt parseUnsignedInt(::jace::proxy::java::lang::String p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer valueOf(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer valueOf(::jace::proxy::java::lang::String p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer valueOf(::jace::proxy::types::JInt p0);
  /**
   * byteValue
   */
  JACE_PROXY_API ::jace::proxy::types::JByte byteValue();
  /**
   * shortValue
   */
  JACE_PROXY_API ::jace::proxy::types::JShort shortValue();
  /**
   * intValue
   */
  JACE_PROXY_API ::jace::proxy::types::JInt intValue();
  /**
   * longValue
   */
  JACE_PROXY_API ::jace::proxy::types::JLong longValue();
  /**
   * floatValue
   */
  JACE_PROXY_API ::jace::proxy::types::JFloat floatValue();
  /**
   * doubleValue
   */
  JACE_PROXY_API ::jace::proxy::types::JDouble doubleValue();
  /**
   * toString
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String toString();
  /**
   * hashCode
   */
  JACE_PROXY_API ::jace::proxy::types::JInt hashCode();
  /**
   * hashCode
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt hashCode(::jace::proxy::types::JInt p0);
  /**
   * equals
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean equals(::jace::proxy::java::lang::Object p0);
  /**
   * getInteger
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer getInteger(::jace::proxy::java::lang::String p0);
  /**
   * getInteger
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer getInteger(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * getInteger
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer getInteger(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::Integer p1);
  /**
   * decode
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::Integer decode(::jace::proxy::java::lang::String p0);
  /**
   * compareTo
   */
  JACE_PROXY_API ::jace::proxy::types::JInt compareTo(::jace::proxy::java::lang::Integer p0);
  /**
   * compare
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt compare(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * compareUnsigned
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt compareUnsigned(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * toUnsignedLong
   */
  JACE_PROXY_API static ::jace::proxy::types::JLong toUnsignedLong(::jace::proxy::types::JInt p0);
  /**
   * divideUnsigned
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt divideUnsigned(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * remainderUnsigned
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt remainderUnsigned(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * highestOneBit
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt highestOneBit(::jace::proxy::types::JInt p0);
  /**
   * lowestOneBit
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt lowestOneBit(::jace::proxy::types::JInt p0);
  /**
   * numberOfLeadingZeros
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt numberOfLeadingZeros(::jace::proxy::types::JInt p0);
  /**
   * numberOfTrailingZeros
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt numberOfTrailingZeros(::jace::proxy::types::JInt p0);
  /**
   * bitCount
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt bitCount(::jace::proxy::types::JInt p0);
  /**
   * rotateLeft
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt rotateLeft(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * rotateRight
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt rotateRight(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * reverse
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt reverse(::jace::proxy::types::JInt p0);
  /**
   * signum
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt signum(::jace::proxy::types::JInt p0);
  /**
   * reverseBytes
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt reverseBytes(::jace::proxy::types::JInt p0);
  /**
   * sum
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt sum(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * max
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt max(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * min
   */
  JACE_PROXY_API static ::jace::proxy::types::JInt min(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  JACE_PROXY_API virtual const JClass& getJavaJniClass() const throw (::jace::JNIException);
  JACE_PROXY_API static const JClass& staticGetJavaJniClass() throw (::jace::JNIException);
  JACE_PROXY_API explicit Integer(jvalue);
  JACE_PROXY_API explicit Integer(jobject);
  /**
   * public static final MIN_VALUE
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::types::JInt > MIN_VALUE();
  
  /**
   * public static final MAX_VALUE
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::types::JInt > MAX_VALUE();
  
  /**
   * public static final SIZE
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::types::JInt > SIZE();
  
  /**
   * public static final BYTES
   */
  JACE_PROXY_API static ::jace::JFieldProxy< ::jace::proxy::types::JInt > BYTES();

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

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> ElementProxy< ::jace::proxy::java::lang::Integer >::ElementProxy(jarray array, jvalue element, int index);
  template <> ElementProxy< ::jace::proxy::java::lang::Integer >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Integer >& proxy);
#else
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
  template <> JFieldProxy< ::jace::proxy::java::lang::Integer >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent);
  template <> JFieldProxy< ::jace::proxy::java::lang::Integer >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass);
  template <> JFieldProxy< ::jace::proxy::java::lang::Integer >::JFieldProxy(const ::jace::JFieldProxy< ::jace::proxy::java::lang::Integer >& object);
#else
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

#endif // #ifndef JACE_PROXY_JAVA_LANG_INTEGER_H

