#ifndef JACE_PROXY_JAVA_LANG_STRING_H
#define JACE_PROXY_JAVA_LANG_STRING_H

#include "jace/OsDep.h"
#include "jace/Namespace.h"
#include "jace/proxy/JObject.h"
#include "jace/JArray.h"
#include "jace/JFieldProxy.h"
#include "jace/JMethod.h"
#include "jace/JField.h"
#include "jace/JClassImpl.h"

#include <string>

/**
 * The super class for this class.
 */
#include "jace/proxy/java/lang/Object.h"
/**
 * The interfaces implemented by this class.
 */
#include "jace/proxy/java/io/Serializable.h"
#include "jace/proxy/java/lang/Comparable.h"
#include "jace/proxy/java/lang/CharSequence.h"
/**
 * Forward declarations for the classes that this class uses.
 */
BEGIN_NAMESPACE_3(jace, proxy, types)
class JByte;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JFloat;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_4(jace, proxy, java, lang)
class Iterable;
END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JVoid;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JInt;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JLong;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JChar;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JDouble;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_3(jace, proxy, types)
class JBoolean;
END_NAMESPACE_3(jace, proxy, types)


BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class for java.lang.String.
 * Please do not edit this class, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
class String: public virtual ::jace::proxy::java::lang::Object, public virtual ::jace::proxy::java::io::Serializable, public virtual ::jace::proxy::java::lang::Comparable, public virtual ::jace::proxy::java::lang::CharSequence
{
  public:
  class Factory
  {
  public:
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create();
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::proxy::java::lang::String p0);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JChar > p0);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JChar > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JInt > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2, ::jace::proxy::types::JInt p3);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2, ::jace::proxy::java::lang::String p3);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::java::lang::String p1);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2);
    /**
     * Creates a new String.
     */
    JACE_PROXY_API static String create(::jace::JArray< ::jace::proxy::types::JByte > p0);
  };
  
  public: 
  /**
   * Creates a new null reference.
   * 
   * All subclasses of JObject should provide this constructor
   * for their own subclasses.
   */
  JACE_PROXY_API explicit String();
  /**
   * Copy an existing reference.
   */
  JACE_PROXY_API String(const String&);
  /**
   * length
   */
  JACE_PROXY_API ::jace::proxy::types::JInt length();
  /**
   * isEmpty
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean isEmpty();
  /**
   * charAt
   */
  JACE_PROXY_API ::jace::proxy::types::JChar charAt(::jace::proxy::types::JInt p0);
  /**
   * codePointAt
   */
  JACE_PROXY_API ::jace::proxy::types::JInt codePointAt(::jace::proxy::types::JInt p0);
  /**
   * codePointBefore
   */
  JACE_PROXY_API ::jace::proxy::types::JInt codePointBefore(::jace::proxy::types::JInt p0);
  /**
   * codePointCount
   */
  JACE_PROXY_API ::jace::proxy::types::JInt codePointCount(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * offsetByCodePoints
   */
  JACE_PROXY_API ::jace::proxy::types::JInt offsetByCodePoints(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * getChars
   */
  JACE_PROXY_API void getChars(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1, ::jace::JArray< ::jace::proxy::types::JChar > p2, ::jace::proxy::types::JInt p3);
  /**
   * getBytes
   */
  JACE_PROXY_API void getBytes(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1, ::jace::JArray< ::jace::proxy::types::JByte > p2, ::jace::proxy::types::JInt p3);
  /**
   * getBytes
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::types::JByte > getBytes(::jace::proxy::java::lang::String p0);
  /**
   * getBytes
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::types::JByte > getBytes();
  /**
   * equals
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean equals(::jace::proxy::java::lang::Object p0);
  /**
   * contentEquals
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean contentEquals(::jace::proxy::java::lang::CharSequence p0);
  /**
   * equalsIgnoreCase
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean equalsIgnoreCase(::jace::proxy::java::lang::String p0);
  /**
   * compareTo
   */
  JACE_PROXY_API ::jace::proxy::types::JInt compareTo(::jace::proxy::java::lang::String p0);
  /**
   * compareToIgnoreCase
   */
  JACE_PROXY_API ::jace::proxy::types::JInt compareToIgnoreCase(::jace::proxy::java::lang::String p0);
  /**
   * regionMatches
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean regionMatches(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::types::JInt p2, ::jace::proxy::types::JInt p3);
  /**
   * regionMatches
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean regionMatches(::jace::proxy::types::JBoolean p0, ::jace::proxy::types::JInt p1, ::jace::proxy::java::lang::String p2, ::jace::proxy::types::JInt p3, ::jace::proxy::types::JInt p4);
  /**
   * startsWith
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean startsWith(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * startsWith
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean startsWith(::jace::proxy::java::lang::String p0);
  /**
   * endsWith
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean endsWith(::jace::proxy::java::lang::String p0);
  /**
   * hashCode
   */
  JACE_PROXY_API ::jace::proxy::types::JInt hashCode();
  /**
   * indexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt indexOf(::jace::proxy::types::JInt p0);
  /**
   * indexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt indexOf(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * lastIndexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt lastIndexOf(::jace::proxy::types::JInt p0);
  /**
   * lastIndexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt lastIndexOf(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * indexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt indexOf(::jace::proxy::java::lang::String p0);
  /**
   * indexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt indexOf(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * lastIndexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt lastIndexOf(::jace::proxy::java::lang::String p0);
  /**
   * lastIndexOf
   */
  JACE_PROXY_API ::jace::proxy::types::JInt lastIndexOf(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * substring
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String substring(::jace::proxy::types::JInt p0);
  /**
   * substring
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String substring(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * subSequence
   */
  JACE_PROXY_API ::jace::proxy::java::lang::CharSequence subSequence(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1);
  /**
   * concat
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String concat(::jace::proxy::java::lang::String p0);
  /**
   * replace
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String replace(::jace::proxy::types::JChar p0, ::jace::proxy::types::JChar p1);
  /**
   * matches
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean matches(::jace::proxy::java::lang::String p0);
  /**
   * contains
   */
  JACE_PROXY_API ::jace::proxy::types::JBoolean contains(::jace::proxy::java::lang::CharSequence p0);
  /**
   * replaceFirst
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String replaceFirst(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1);
  /**
   * replaceAll
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String replaceAll(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1);
  /**
   * replace
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String replace(::jace::proxy::java::lang::CharSequence p0, ::jace::proxy::java::lang::CharSequence p1);
  /**
   * split
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::java::lang::String > split(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1);
  /**
   * split
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::java::lang::String > split(::jace::proxy::java::lang::String p0);
  /**
   * join
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String join(::jace::proxy::java::lang::CharSequence p0, ::jace::JArray< ::jace::proxy::java::lang::CharSequence > p1);
  /**
   * join
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String join(::jace::proxy::java::lang::CharSequence p0, ::jace::proxy::java::lang::Iterable p1);
  /**
   * toLowerCase
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String toLowerCase();
  /**
   * toUpperCase
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String toUpperCase();
  /**
   * trim
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String trim();
  /**
   * toString
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String toString();
  /**
   * toCharArray
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::types::JChar > toCharArray();
  /**
   * format
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String format(::jace::proxy::java::lang::String p0, ::jace::JArray< ::jace::proxy::java::lang::Object > p1);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::java::lang::Object p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::JArray< ::jace::proxy::types::JChar > p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::JArray< ::jace::proxy::types::JChar > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2);
  /**
   * copyValueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String copyValueOf(::jace::JArray< ::jace::proxy::types::JChar > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2);
  /**
   * copyValueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String copyValueOf(::jace::JArray< ::jace::proxy::types::JChar > p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::types::JBoolean p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::types::JChar p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::types::JInt p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::types::JLong p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::types::JFloat p0);
  /**
   * valueOf
   */
  JACE_PROXY_API static ::jace::proxy::java::lang::String valueOf(::jace::proxy::types::JDouble p0);
  /**
   * intern
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String intern();
  JACE_PROXY_API virtual const JClass& getJavaJniClass() const throw (::jace::JNIException);
  JACE_PROXY_API static const JClass& staticGetJavaJniClass() throw (::jace::JNIException);
  JACE_PROXY_API explicit String(jvalue);
  JACE_PROXY_API explicit String(jobject);
  /**
   * Creates a String from a C string.
   */
  JACE_PROXY_API String(const char*);
  /**
   * Creates a new jstring from a std::string using the platform's default charset.
   */
  JACE_PROXY_API String(const std::string&);
  /**
   * Creates a String from a std::wstring.
   */
  JACE_PROXY_API String(const std::wstring&);
  /**
   * Handle assignment between two Strings.
   */
  JACE_PROXY_API String& operator=(const String& str);
  
  /**
   * Converts a String to a std::string.
   */
  JACE_PROXY_API operator std::string() const;
  
  /**
   * Converts a String to a std::wstring.
   */
  JACE_PROXY_API operator std::wstring() const;
  
  /**
   * Allows Strings to be written to ostreams.
   */
  JACE_PROXY_API friend std::ostream& operator<<(std::ostream& stream, const String& str);
  
  /**
   * Provide concatentation for Strings.
   */
  JACE_PROXY_API String operator+(String);
  
  /**
   * Provide concatenation between Strings and std::strings.
   */
  JACE_PROXY_API friend std::string operator+(const std::string&, const String&);
  
  /**
   * Provide concatenation between Strings and std::strings.
   */
  JACE_PROXY_API friend std::string operator+(const String&, const std::string&);
  
  /**
   * Provide comparison between Strings and std::strings.
   */
  JACE_PROXY_API friend bool operator==(const std::string&, const String&);
  
  /**
   * Provide comparison between Strings and std::strings.
   */
  JACE_PROXY_API friend bool operator==(const String&, const std::string&);

private:
  /**
   * The following methods are required to integrate this class
   * with the Jace framework.
   */
  /**
   * Creates a new jstring from a std::string using the platform's default charset.
   */
  jstring createString(const std::string& str);
  
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
  template <> ElementProxy< ::jace::proxy::java::lang::String >::ElementProxy(jarray array, jvalue element, int index);
  template <> ElementProxy< ::jace::proxy::java::lang::String >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::String >& proxy);
#else
  template <> inline ElementProxy< ::jace::proxy::java::lang::String >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::lang::String(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::lang::String >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::String >& proxy): 
    ::jace::proxy::java::lang::String(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif

#ifndef PUT_TSDS_IN_HEADER
  template <> JFieldProxy< ::jace::proxy::java::lang::String >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent);
  template <> JFieldProxy< ::jace::proxy::java::lang::String >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass);
  template <> JFieldProxy< ::jace::proxy::java::lang::String >::JFieldProxy(const ::jace::JFieldProxy< ::jace::proxy::java::lang::String >& object);
#else
  template <> inline JFieldProxy< ::jace::proxy::java::lang::String >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::lang::String(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::String >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::lang::String(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::lang::String >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::lang::String >& object): 
    ::jace::proxy::java::lang::String(object)
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

#endif // #ifndef JACE_PROXY_JAVA_LANG_STRING_H

