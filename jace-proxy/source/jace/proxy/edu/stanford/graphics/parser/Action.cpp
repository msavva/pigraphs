#include "jace/proxy/edu/stanford/graphics/parser/Action.h"

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
#include "jace/proxy/edu/stanford/graphics/parser/Mention.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_6(jace, proxy, edu, stanford, graphics, parser)

/**
 * The Jace C++ proxy class source for edu/stanford/graphics/parser/Action.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define Action_INITIALIZER

Action Action::Factory::create(::jace::proxy::edu::stanford::graphics::parser::Mention p0, ::jace::proxy::edu::stanford::graphics::parser::Mention p1, ::jace::proxy::edu::stanford::graphics::parser::Mention p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(Action::staticGetJavaJniClass(), arguments);
  Action result = Action(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

Action Action::Factory::create(::jace::proxy::java::lang::String p0, ::jace::proxy::java::lang::String p1, ::jace::proxy::java::lang::String p2){
  JArguments arguments;
  arguments << p0 << p1 << p2;
  jobject localRef = newObject(Action::staticGetJavaJniClass(), arguments);
  Action result = Action(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::proxy::edu::stanford::graphics::parser::Mention Action::getAgent()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::edu::stanford::graphics::parser::Mention >("getAgent").invoke(*this, arguments);
}

::jace::proxy::edu::stanford::graphics::parser::Mention Action::getVerb()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::edu::stanford::graphics::parser::Mention >("getVerb").invoke(*this, arguments);
}

::jace::proxy::edu::stanford::graphics::parser::Mention Action::getObject()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::edu::stanford::graphics::parser::Mention >("getObject").invoke(*this, arguments);
}

::jace::proxy::types::JBoolean Action::equals(::jace::proxy::java::lang::Object p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::types::JBoolean >("equals").invoke(*this, arguments);
}

::jace::proxy::types::JInt Action::hashCode()
{
  JArguments arguments;
  return JMethod< ::jace::proxy::types::JInt >("hashCode").invoke(*this, arguments);
}

::jace::proxy::java::lang::String Action::toString()
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
Action::Action()
{}

Action::Action(jvalue value) Action_INITIALIZER
{
  setJavaJniValue(value);
}

Action::Action(jobject object) Action_INITIALIZER
{
  setJavaJniObject(object);
}

Action::Action(const Action& object) Action_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * public final agent
 */
::jace::JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Mention > Action::agent()
{
  return ::jace::JField< ::jace::proxy::edu::stanford::graphics::parser::Mention >("agent").get(*this);
}

/**
 * public final verb
 */
::jace::JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Mention > Action::verb()
{
  return ::jace::JField< ::jace::proxy::edu::stanford::graphics::parser::Mention >("verb").get(*this);
}

/**
 * public final object
 */
::jace::JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Mention > Action::object()
{
  return ::jace::JField< ::jace::proxy::edu::stanford::graphics::parser::Mention >("object").get(*this);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& Action::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("edu/stanford/graphics/parser/Action"));
  return *result;
}

const JClass& Action::getJavaJniClass() const throw (::jace::JNIException)
{
  return Action::staticGetJavaJniClass();
}

END_NAMESPACE_6(jace, proxy, edu, stanford, graphics, parser)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::edu::stanford::graphics::parser::Action(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >::ElementProxy(const jace::ElementProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >& proxy): 
    ::jace::proxy::edu::stanford::graphics::parser::Action(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::edu::stanford::graphics::parser::Action(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::edu::stanford::graphics::parser::Action(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >::JFieldProxy(const JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::Action >& object): 
    ::jace::proxy::edu::stanford::graphics::parser::Action(object)
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

