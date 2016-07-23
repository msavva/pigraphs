#ifndef JACE_PROXY_JAVA_LANG_THROWABLE_H
#define JACE_PROXY_JAVA_LANG_THROWABLE_H

#include "jace/OsDep.h"
#include "jace/Namespace.h"
#include "jace/proxy/JObject.h"
#include "jace/JEnlister.h"
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
/**
 * Forward declarations for the classes that this class uses.
 */
BEGIN_NAMESPACE_3(jace, proxy, types)
class JVoid;
END_NAMESPACE_3(jace, proxy, types)

BEGIN_NAMESPACE_4(jace, proxy, java, lang)
class String;
END_NAMESPACE_4(jace, proxy, java, lang)


BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class for java.lang.Throwable.
 * Please do not edit this class, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
class Throwable: public virtual ::jace::proxy::java::lang::Object, public std::exception, public virtual ::jace::proxy::java::io::Serializable
{
  public:
  class Factory
  {
  public:
    /**
     * Creates a new Throwable.
     */
    JACE_PROXY_API static Throwable create();
    /**
     * Creates a new Throwable.
     */
    JACE_PROXY_API static Throwable create(::jace::proxy::java::lang::String p0);
    /**
     * Creates a new Throwable.
     */
    JACE_PROXY_API static Throwable create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::Throwable p1);
    /**
     * Creates a new Throwable.
     */
    JACE_PROXY_API static Throwable create(::jace::proxy::java::lang::Throwable p0);
  };
  
  public: 
  /**
   * Creates a new null reference.
   * 
   * All subclasses of JObject should provide this constructor
   * for their own subclasses.
   */
  JACE_PROXY_API explicit Throwable();
  /**
   * Copy an existing reference.
   */
  JACE_PROXY_API Throwable(const Throwable&);
  /**
   * getMessage
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String getMessage();
  /**
   * getLocalizedMessage
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String getLocalizedMessage();
  /**
   * getCause
   */
  JACE_PROXY_API ::jace::proxy::java::lang::Throwable getCause();
  /**
   * initCause
   */
  JACE_PROXY_API ::jace::proxy::java::lang::Throwable initCause(::jace::proxy::java::lang::Throwable p0);
  /**
   * toString
   */
  JACE_PROXY_API ::jace::proxy::java::lang::String toString();
  /**
   * printStackTrace
   */
  JACE_PROXY_API void printStackTrace();
  /**
   * fillInStackTrace
   */
  JACE_PROXY_API ::jace::proxy::java::lang::Throwable fillInStackTrace();
  /**
   * addSuppressed
   */
  JACE_PROXY_API void addSuppressed(::jace::proxy::java::lang::Throwable p0);
  /**
   * getSuppressed
   */
  JACE_PROXY_API ::jace::JArray< ::jace::proxy::java::lang::Throwable > getSuppressed();
  JACE_PROXY_API virtual const JClass& getJavaJniClass() const throw (::jace::JNIException);
  JACE_PROXY_API static const JClass& staticGetJavaJniClass() throw (::jace::JNIException);
  JACE_PROXY_API explicit Throwable(jvalue);
  JACE_PROXY_API explicit Throwable(jobject);
  /**
   * Need to support a non-throwing destructor
   */
  JACE_PROXY_API ~Throwable() throw ();
  
  /**
   * Overrides std::exception::what() by returning this.toString();
   */
  JACE_PROXY_API const char* what() const throw();
  
  /**
   * The message represented by this Throwable.
   * 
   * This member variable is necessary to keep the contract
   * for exception.what().
   */
  private: 
  std::string msg;
  public: 

private:
  /**
   * The following methods are required to integrate this class
   * with the Jace framework.
   */
  static JEnlister< Throwable > enlister;
  template <typename T> friend class ::jace::JEnlister;
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
  template <> ElementProxy< ::jace::proxy::java::lang::Throwable >::ElementProxy(jarray array, jvalue element, int index);
  template <> ElementProxy< ::jace::proxy::java::lang::Throwable >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::lang::Throwable >& proxy);
#else
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
  template <> JFieldProxy< ::jace::proxy::java::lang::Throwable >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent);
  template <> JFieldProxy< ::jace::proxy::java::lang::Throwable >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass);
  template <> JFieldProxy< ::jace::proxy::java::lang::Throwable >::JFieldProxy(const ::jace::JFieldProxy< ::jace::proxy::java::lang::Throwable >& object);
#else
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

#endif // #ifndef JACE_PROXY_JAVA_LANG_THROWABLE_H

