#include "jace/proxy/edu/stanford/graphics/wekautils/Clustering.h"

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
#include "jace/proxy/types/JDouble.h"
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_6(jace, proxy, edu, stanford, graphics, wekautils)

/**
 * The Jace C++ proxy class source for edu/stanford/graphics/wekautils/Clustering.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Clustering_INITIALIZER

Clustering Clustering::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(Clustering::staticGetJavaJniClass(), arguments);
  Clustering result = Clustering(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::JArray< ::jace::proxy::types::JDouble > Clustering::getKmeansCentroids(::jace::JArray< ::jace::proxy::types::JDouble > p0, ::jace::proxy::types::JInt p1, ::jace::proxy::types::JInt p2, ::jace::proxy::types::JInt p3)
{
  JArguments arguments;
  arguments << p0 << p1 << p2 << p3;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JDouble > >("getKmeansCentroids").invoke(staticGetJavaJniClass(), arguments);
}

::jace::JArray< ::jace::proxy::types::JDouble > Clustering::getKmeansCentroids(::jace::proxy::java::lang::String p0, ::jace::proxy::types::JInt p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::JArray< ::jace::proxy::types::JDouble > >("getKmeansCentroids").invoke(staticGetJavaJniClass(), arguments);
}

::jace::proxy::types::JDouble Clustering::saveKmeansCentroids(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::types::JInt p2)
{
  JArguments arguments;
  arguments << p0 << p1 << p2;
  return JMethod< ::jace::proxy::types::JDouble >("saveKmeansCentroids").invoke(staticGetJavaJniClass(), arguments);
}

void Clustering::main(::jace::JArray< ::jace::proxy::java::lang::String > p0)
{
  JArguments arguments;
  arguments << p0;
  JMethod< ::jace::proxy::types::JVoid >("main").invoke(staticGetJavaJniClass(), arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
Clustering::Clustering()
{}

Clustering::Clustering(jvalue value) Clustering_INITIALIZER
{
  setJavaJniValue(value);
}

Clustering::Clustering(jobject object) Clustering_INITIALIZER
{
  setJavaJniObject(object);
}

Clustering::Clustering(const Clustering& object) Clustering_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Clustering::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("edu/stanford/graphics/wekautils/Clustering"));
  return *result;
}

const JClass& Clustering::getJavaJniClass() const throw (::jace::JNIException)
{
  return Clustering::staticGetJavaJniClass();
}

END_NAMESPACE_6(jace, proxy, edu, stanford, graphics, wekautils)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Clustering(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >::ElementProxy(const jace::ElementProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >& proxy): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Clustering(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Clustering(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Clustering(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >::JFieldProxy(const JFieldProxy< ::jace::proxy::edu::stanford::graphics::wekautils::Clustering >& object): 
    ::jace::proxy::edu::stanford::graphics::wekautils::Clustering(object)
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

