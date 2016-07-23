#include "jace/proxy/java/util/Collection.h"

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
 * The Jace C++ proxy class source for java/util/Collection.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Collection_INITIALIZER

::jace::proxy::types::JInt Collection::size()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("size").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::isEmpty()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JBoolean >("isEmpty").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::contains(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("contains").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > Collection::toArray()
{
  JArguments arguments;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::JArray< ::jace::proxy::java::lang::Object > Collection::toArray(::jace::JArray< ::jace::proxy::java::lang::Object > p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::JArray< ::jace::proxy::java::lang::Object > >("toArray").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::add(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("add").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::remove(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("remove").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::containsAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("containsAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::addAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("addAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::removeAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("removeAll").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::retainAll(::jace::proxy::java::util::Collection p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("retainAll").invoke(*this, arguments);
}

void Collection::clear()
{
  JArguments arguments;
  JMethod< ::jace::proxy::types::JVoid >("clear").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Collection::equals(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equals").invoke(*this, arguments);
}

::jace::proxy::types::JInt Collection::hashCode()
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
Collection::Collection()
{}

Collection::Collection(jvalue value) Collection_INITIALIZER
{
  setJavaJniValue(value);
}

Collection::Collection(jobject object) Collection_INITIALIZER
{
  setJavaJniObject(object);
}

Collection::Collection(const Collection& object) Collection_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Collection::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("java/util/Collection"));
  return *result;
}

const JClass& Collection::getJavaJniClass() const throw (::jace::JNIException)
{
  return Collection::staticGetJavaJniClass();
}

END_NAMESPACE_4(jace, proxy, java, util)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::java::util::Collection >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::java::util::Collection(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::java::util::Collection >::ElementProxy(const jace::ElementProxy< ::jace::proxy::java::util::Collection >& proxy): 
    ::jace::proxy::java::util::Collection(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::java::util::Collection >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::java::util::Collection(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::Collection >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::java::util::Collection(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::java::util::Collection >::JFieldProxy(const JFieldProxy< ::jace::proxy::java::util::Collection >& object): 
    ::jace::proxy::java::util::Collection(object)
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

