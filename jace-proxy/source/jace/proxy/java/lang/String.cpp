#include "jace/proxy/java/lang/String.h"

/**
 * Standard Jace headers needed to implement this class.
 */
#include "jace/JArguments.h"
#include "jace/JMethod.h"
#include "jace/JField.h"
#include "jace/JClassImpl.h"
#include "jace/proxy/java/lang/Integer.h"
#include "jace/BoostWarningOff.h"
#include <boost/thread/mutex.hpp>
#include "jace/BoostWarningOn.h"

/**
 * Headers for the classes that this class uses.
 */
#include "jace/proxy/types/JByte.h"
#include "jace/proxy/types/JFloat.h"
#include "jace/proxy/java/lang/Iterable.h"
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/types/JLong.h"
#include "jace/proxy/types/JChar.h"
#include "jace/proxy/types/JDouble.h"
#include "jace/proxy/types/JBoolean.h"

BEGIN_NAMESPACE_4(jace, proxy, java, lang)

/**
 * The Jace C++ proxy class source for java/lang/String.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define String_INITIALIZER

String String::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::proxy::java::lang::String p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JChar > p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JChar > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JInt > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2, ::jace::proxy::types::JInt p3){
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1){
  JArguments arguments;
  arguments << p0 << p1;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2, ::jace::proxy::java::lang::String p3){
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::java::lang::String p1){
  JArguments arguments;
  arguments << p0 << p1;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JByte > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

String String::Factory::create(::jace::JArray< ::jace::proxy::types::JByte > p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(String::staticGetJavaJniClass(), arguments);
  String result = String(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::proxy::types::JInt String::length()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("length").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::isEmpty()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JBoolean >("isEmpty").invoke(*this, arguments);
}

::jace::proxy::types::JChar String::charAt(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JChar >("charAt").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::codePointAt(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("codePointAt").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::codePointBefore(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("codePointBefore").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::codePointCount(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("codePointCount").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::offsetByCodePoints(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("offsetByCodePoints").invoke(*this, arguments);
}

void String::getChars(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1, ::jace::JArray< ::jace::proxy::types::JChar > p2, ::jace::proxy::types::JInt p3)
{
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  JMethod< ::jace::proxy::types::JVoid >("getChars").invoke(*this, arguments);
}

void String::getBytes(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1, ::jace::JArray< ::jace::proxy::types::JByte > p2, ::jace::proxy::types::JInt p3)
{
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  JMethod< ::jace::proxy::types::JVoid >("getBytes").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::types::JByte > String::getBytes(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JByte > >("getBytes").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::types::JByte > String::getBytes()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JByte > >("getBytes").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::equals(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equals").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::contentEquals(::jace::proxy::java::lang::CharSequence p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("contentEquals").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::equalsIgnoreCase(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equalsIgnoreCase").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::compareTo(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("compareTo").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::compareToIgnoreCase(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("compareToIgnoreCase").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::regionMatches(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::types::JInt p2, ::jace::proxy::types::JInt p3)
{
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  return JMethod< ::jace::proxy::types::JBoolean >("regionMatches").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::regionMatches(::jace::proxy::types::JBoolean p0, ::jace::proxy::types::JInt p1, ::jace::proxy::java::lang::String p2, ::jace::proxy::types::JInt p3, ::jace::proxy::types::JInt p4)
{
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3 << p4;
  return JMethod< ::jace::proxy::types::JBoolean >("regionMatches").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::startsWith(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JBoolean >("startsWith").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::startsWith(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("startsWith").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::endsWith(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("endsWith").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::hashCode()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("hashCode").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::indexOf(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::indexOf(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::lastIndexOf(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::lastIndexOf(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::indexOf(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::indexOf(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::lastIndexOf(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt String::lastIndexOf(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::substring(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("substring").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::substring(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("substring").invoke(*this, arguments);
}

::jace::proxy::java::lang::CharSequence String::subSequence(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::CharSequence >("subSequence").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::concat(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("concat").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::replace(::jace::proxy::types::JChar p0, ::jace::proxy::types::JChar p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("replace").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::matches(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("matches").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean String::contains(::jace::proxy::java::lang::CharSequence p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("contains").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::replaceFirst(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("replaceFirst").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::replaceAll(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("replaceAll").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::replace(::jace::proxy::java::lang::CharSequence p0, ::jace::proxy::java::lang::CharSequence p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("replace").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::String > String::split(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::String > >("split").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::String > String::split(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::String > >("split").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::join(::jace::proxy::java::lang::CharSequence p0, ::jace::JArray< ::jace::proxy::java::lang::CharSequence > p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("join").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::join(::jace::proxy::java::lang::CharSequence p0, ::jace::proxy::java::lang::Iterable p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("join").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::toLowerCase()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("toLowerCase").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::toUpperCase()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("toUpperCase").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::trim()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("trim").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::toString()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("toString").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::types::JChar > String::toCharArray()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JChar > >("toCharArray").invoke(*this, arguments);
}

::jace::proxy::java::lang::String String::format(::jace::proxy::java::lang::String p0, ::jace::JArray< ::jace::proxy::java::lang::Object > p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::String >("format").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::JArray< ::jace::proxy::types::JChar > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::JArray< ::jace::proxy::types::JChar > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2)
{
  JArguments arguments;
  arguments << p0 << p1 << p2;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::copyValueOf(::jace::JArray< ::jace::proxy::types::JChar > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2)
{
  JArguments arguments;
  arguments << p0 << p1 << p2;
  return JMethod< ::jace::proxy::java::lang::String >("copyValueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::copyValueOf(::jace::JArray< ::jace::proxy::types::JChar > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("copyValueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::types::JBoolean p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::types::JChar p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::types::JLong p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::types::JFloat p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::valueOf(::jace::proxy::types::JDouble p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::String >("valueOf").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::java::lang::String String::intern()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::String >("intern").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
String::String()
{}

String::String(jvalue value) String_INITIALIZER
{
  setJavaJniValue(value);
}

String::String(jobject object) String_INITIALIZER
{
  setJavaJniObject(object);
}

String::String(const String& object) String_INITIALIZER
{
  setJavaJniObject(object);
}

String::String(const char* str)
{
  jstring strRef = createString(str);
  setJavaJniObject(strRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, strRef);
}

String::String(const std::string& str)
{
  jstring strRef = createString(str);
  setJavaJniObject(strRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, strRef);
}

String& String::operator=(const String& str)
{
  setJavaJniObject(str);
  return *this;
}

String::operator std::string() const
{
  JNIEnv* env = attach();
  jstring thisString = static_cast<jstring>(static_cast<jobject>(*this));
  jclass cls = getJavaJniClass().getClass();
  jmethodID getBytes = env->GetMethodID(cls, "getBytes", "()[B");
  jbyteArray array = static_cast<jbyteArray>(env->CallObjectMethod(thisString, getBytes));

  if (!array)
  {
    env->ExceptionDescribe();
    env->ExceptionClear();
    throw JNIException("String::operator std::string()- Unable to get the contents of the java String.");
  }

  int arraySize = env->GetArrayLength(array);
  jbyte* byteArray = env->GetByteArrayElements(array, 0);

  if (!byteArray)
  {
    env->ExceptionDescribe();
    env->ExceptionClear();
    throw JNIException("String::operator std::string() - Unable to get the contents of the java String.");
  }

  std::string str((char*) byteArray, (char*) byteArray + arraySize);
  env->ReleaseByteArrayElements(array, byteArray, JNI_ABORT);
  deleteLocalRef(env, array);
  return str;
}

/**
 * Creates a new jstring from a std::string using the platform's default charset.
 */
jstring String::createString(const std::string& str)
{
  JNIEnv* env = attach();
  size_t nativeLength = str.size();
  if (nativeLength > static_cast<size_t>(::jace::proxy::java::lang::Integer::MAX_VALUE()))
  {
    throw JNIException(std::string("String::String(const std::string& str) - str.size() (") +
      jace::toString(nativeLength) + ") > Integer.MAX_VALUE.");
  }
  jsize bufLen = jsize(nativeLength);
  jbyteArray jbuf = env->NewByteArray(bufLen);

  if (!jbuf)
  {
    env->ExceptionDescribe();
    env->ExceptionClear();
    throw JNIException("String::createString - Unable to allocate a new java String.");
  }

  env->SetByteArrayRegion(jbuf, 0, bufLen, (jbyte*) str.c_str());
  jclass cls = getJavaJniClass().getClass();
  jmethodID init = env->GetMethodID(cls, "<init>", "([BII)V");
  jstring jstr = static_cast<jstring>(env->NewObject(cls, init, jbuf, 0, bufLen)); 

  if (!jstr)
  {
    env->ExceptionDescribe();
    env->ExceptionClear();
    throw JNIException("String::createString - Unable to allocate a new java String.");
  }

  deleteLocalRef(env, jbuf);
  return jstr;
}

std::ostream& operator<<(std::ostream& stream, const String& str)
{
  return stream << (std::string) str;
}

std::string operator+(const std::string& stdStr, const String& jStr)
{
  return stdStr + (std::string) jStr;
}

std::string operator+(const String& jStr, const std::string& stdStr)
{
  return (std::string) jStr + stdStr;
}

String String::operator+(String str)
{
  return (std::string) *this + (std::string) str;
}

bool operator==(const std::string& stdStr, const String& str)
{
  return (std::string) str == stdStr;
}

bool operator==(const String& str, const std::string& stdStr)
{
  return (std::string) str == stdStr;
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& String::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/lang/String"));
  return *result;
}

const JClass& String::getJavaJniClass() const throw (::jace::JNIException)
{
  return String::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, lang)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
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

