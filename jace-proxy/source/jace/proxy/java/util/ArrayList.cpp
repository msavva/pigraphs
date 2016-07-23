#include "jace/proxy/java/util/ArrayList.h"

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
#include "jace/proxy/types/JBoolean.h"
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/java/util/Collection.h"
#include "jace/proxy/java/lang/Object.h"

BEGIN_NAMESPACE_4(jace, proxy, java, util)

/**
 * The Jace C++ proxy class source for java/util/ArrayList.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define ArrayList_INITIALIZER : ::jace::proxy::java::util::AbstractList()

ArrayList ArrayList::Factory::create(::jace::proxy::types::JInt p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(ArrayList::staticGetJavaJniClass(), arguments);
  ArrayList result = ArrayList(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

ArrayList ArrayList::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(ArrayList::staticGetJavaJniClass(), arguments);
  ArrayList result = ArrayList(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

ArrayList ArrayList::Factory::create(::jace::proxy::java::util::Collection p0){
  JArguments arguments;
  arguments << p0;
  jobject localRef = newObject(ArrayList::staticGetJavaJniClass(), arguments);
  ArrayList result = ArrayList(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

void ArrayList::trimToSize()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("trimToSize").invoke(*this, arguments);
}

void ArrayList::ensureCapacity(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  JMethod< ::jace::proxy::types::JVoid >("ensureCapacity").invoke(*this, arguments);
}

::jace::proxy::types::JInt ArrayList::size()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("size").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::isEmpty()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JBoolean >("isEmpty").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::contains(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("contains").invoke(*this, arguments);
}

::jace::proxy::types::JInt ArrayList::indexOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt ArrayList::lastIndexOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object ArrayList::clone()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::java::lang::Object >("clone").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > ArrayList::toArray()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > ArrayList::toArray(::jace::JArray< ::jace::proxy::java::lang::Object > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object ArrayList::get(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Object >("get").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object ArrayList::set(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::Object p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::Object >("set").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::add(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("add").invoke(*this, arguments);
}

void ArrayList::add(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::Object p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  JMethod< ::jace::proxy::types::JVoid >("add").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object ArrayList::remove(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Object >("remove").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::remove(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("remove").invoke(*this, arguments);
}

void ArrayList::clear()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("clear").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::addAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::addAll(::jace::proxy::types::JInt p0, ::jace::proxy::java::util::Collection p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::removeAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("removeAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean ArrayList::retainAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("retainAll").invoke(*this, arguments);
}

::jace::proxy::java::util::List ArrayList::subList(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::util::List >("subList").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
ArrayList::ArrayList()
{}

ArrayList::ArrayList(jvalue value) ArrayList_INITIALIZER
{
  setJavaJniValue(value);
}

ArrayList::ArrayList(jobject object) ArrayList_INITIALIZER
{
  setJavaJniObject(object);
}

ArrayList::ArrayList(const ArrayList& object) ArrayList_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& ArrayList::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/util/ArrayList"));
  return *result;
}

const JClass& ArrayList::getJavaJniClass() const throw (::jace::JNIException)
{
  return ArrayList::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, util)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::util::ArrayList >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::util::ArrayList(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::util::ArrayList >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::util::ArrayList >& proxy): 
    ::jace::proxy::java::util::ArrayList(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::util::ArrayList >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::util::ArrayList(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::ArrayList >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::util::ArrayList(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::ArrayList >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::util::ArrayList >& object): 
    ::jace::proxy::java::util::ArrayList(object)
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

