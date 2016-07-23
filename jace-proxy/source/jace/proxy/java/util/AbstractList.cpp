#include "jace/proxy/java/util/AbstractList.h"

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
 * The Jace C++ proxy class source for java/util/AbstractList.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define AbstractList_INITIALIZER : ::jace::proxy::java::util::AbstractCollection()

::jace::proxy::types::JBoolean AbstractList::add(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("add").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object AbstractList::get(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Object >("get").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object AbstractList::set(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::Object p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::Object >("set").invoke(*this, arguments);
}

void AbstractList::add(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::Object p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  JMethod< ::jace::proxy::types::JVoid >("add").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object AbstractList::remove(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Object >("remove").invoke(*this, arguments);
}

::jace::proxy::types::JInt AbstractList::indexOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt AbstractList::lastIndexOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

void AbstractList::clear()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("clear").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractList::addAll(::jace::proxy::types::JInt p0, ::jace::proxy::java::util::Collection p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::java::util::List AbstractList::subList(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::util::List >("subList").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractList::equals(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equals").invoke(*this, arguments);
}

::jace::proxy::types::JInt AbstractList::hashCode()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("hashCode").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
AbstractList::AbstractList()
{}

AbstractList::AbstractList(jvalue value) AbstractList_INITIALIZER
{
  setJavaJniValue(value);
}

AbstractList::AbstractList(jobject object) AbstractList_INITIALIZER
{
  setJavaJniObject(object);
}

AbstractList::AbstractList(const AbstractList& object) AbstractList_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& AbstractList::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/util/AbstractList"));
  return *result;
}

const JClass& AbstractList::getJavaJniClass() const throw (::jace::JNIException)
{
  return AbstractList::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, util)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::util::AbstractList >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::util::AbstractList(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::util::AbstractList >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::util::AbstractList >& proxy): 
    ::jace::proxy::java::util::AbstractList(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::util::AbstractList >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::util::AbstractList(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::AbstractList >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::util::AbstractList(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::AbstractList >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::util::AbstractList >& object): 
    ::jace::proxy::java::util::AbstractList(object)
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

