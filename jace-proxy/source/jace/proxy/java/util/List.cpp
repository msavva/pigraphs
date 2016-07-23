#include "jace/proxy/java/util/List.h"

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

BEGIN_NAMESPACE_4(jace, proxy, java, util)

/**
 * The Jace C++ proxy class source for java/util/List.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define List_INITIALIZER

::jace::proxy::types::JInt List::size()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("size").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::isEmpty()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JBoolean >("isEmpty").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::contains(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("contains").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > List::toArray()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > List::toArray(::jace::JArray< ::jace::proxy::java::lang::Object > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::add(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("add").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::remove(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("remove").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::containsAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("containsAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::addAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::addAll(::jace::proxy::types::JInt p0, ::jace::proxy::java::util::Collection p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::removeAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("removeAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::retainAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("retainAll").invoke(*this, arguments);
}

void List::clear()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("clear").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean List::equals(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equals").invoke(*this, arguments);
}

::jace::proxy::types::JInt List::hashCode()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("hashCode").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object List::get(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Object >("get").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object List::set(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::Object p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::java::lang::Object >("set").invoke(*this, arguments);
}

void List::add(::jace::proxy::types::JInt p0, ::jace::proxy::java::lang::Object p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  JMethod< ::jace::proxy::types::JVoid >("add").invoke(*this, arguments);
}

::jace::proxy::java::lang::Object List::remove(::jace::proxy::types::JInt p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::lang::Object >("remove").invoke(*this, arguments);
}

::jace::proxy::types::JInt List::indexOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("indexOf").invoke(*this, arguments);
}

::jace::proxy::types::JInt List::lastIndexOf(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JInt >("lastIndexOf").invoke(*this, arguments);
}

::jace::proxy::java::util::List List::subList(::jace::proxy::types::JInt p0, ::jace::proxy::types::JInt p1)
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
List::List()
{}

List::List(jvalue value) List_INITIALIZER
{
  setJavaJniValue(value);
}

List::List(jobject object) List_INITIALIZER
{
  setJavaJniObject(object);
}

List::List(const List& object) List_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& List::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/util/List"));
  return *result;
}

const JClass& List::getJavaJniClass() const throw (::jace::JNIException)
{
  return List::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, util)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::util::List >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::util::List(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::util::List >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::util::List >& proxy): 
    ::jace::proxy::java::util::List(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::util::List >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::util::List(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::List >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::util::List(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::List >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::util::List >& object): 
    ::jace::proxy::java::util::List(object)
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

