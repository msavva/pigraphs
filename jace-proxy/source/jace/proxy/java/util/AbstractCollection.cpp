#include "jace/proxy/java/util/AbstractCollection.h"

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
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_4(jace, proxy, java, util)

/**
 * The Jace C++ proxy class source for java/util/AbstractCollection.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define AbstractCollection_INITIALIZER

::jace::proxy::types::JInt AbstractCollection::size()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("size").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::isEmpty()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JBoolean >("isEmpty").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::contains(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("contains").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > AbstractCollection::toArray()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > AbstractCollection::toArray(::jace::JArray< ::jace::proxy::java::lang::Object > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::add(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("add").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::remove(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("remove").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::containsAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("containsAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::addAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::removeAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("removeAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean AbstractCollection::retainAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("retainAll").invoke(*this, arguments);
}

void AbstractCollection::clear()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("clear").invoke(*this, arguments);
}

::jace::proxy::java::lang::String AbstractCollection::toString()
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
AbstractCollection::AbstractCollection()
{}

AbstractCollection::AbstractCollection(jvalue value) AbstractCollection_INITIALIZER
{
  setJavaJniValue(value);
}

AbstractCollection::AbstractCollection(jobject object) AbstractCollection_INITIALIZER
{
  setJavaJniObject(object);
}

AbstractCollection::AbstractCollection(const AbstractCollection& object) AbstractCollection_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& AbstractCollection::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/util/AbstractCollection"));
  return *result;
}

const JClass& AbstractCollection::getJavaJniClass() const throw (::jace::JNIException)
{
  return AbstractCollection::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, util)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::util::AbstractCollection >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::util::AbstractCollection(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::util::AbstractCollection >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::util::AbstractCollection >& proxy): 
    ::jace::proxy::java::util::AbstractCollection(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::util::AbstractCollection >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::util::AbstractCollection(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::AbstractCollection >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::util::AbstractCollection(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::AbstractCollection >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::util::AbstractCollection >& object): 
    ::jace::proxy::java::util::AbstractCollection(object)
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

