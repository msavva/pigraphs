#include "jace/proxy/edu/stanford/graphics/parser/ActionParser.h"

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
#include "jace/proxy/types/JVoid.h"
#include "jace/proxy/types/JInt.h"
#include "jace/proxy/java/util/List.h"
#include "jace/proxy/java/lang/String.h"

BEGIN_NAMESPACE_6(jace, proxy, edu, stanford, graphics, parser)

/**
 * The Jace C++ proxy class source for edu/stanford/graphics/parser/ActionParser.
 * Please do not edit this source, as any changes you make will be overwritten.
 * For more information, please refer to the Jace Developer's Guide.
 */
#define ActionParser_INITIALIZER

ActionParser ActionParser::Factory::create(){
  JArguments arguments;
  jobject localRef = newObject(ActionParser::staticGetJavaJniClass(), arguments);
  ActionParser result = ActionParser(localRef);
  JNIEnv* env = attach();
  deleteLocalRef(env, localRef);
  return result;
}

::jace::proxy::types::JInt ActionParser::addActions(::jace::proxy::java::lang::String p0, ::jace::proxy::java::util::List p1)
{
  JArguments arguments;
  arguments << p0 << p1;
  return JMethod< ::jace::proxy::types::JInt >("addActions").invoke(*this, arguments);
}

::jace::proxy::java::util::List ActionParser::parse(::jace::proxy::java::lang::String p0)
{
  JArguments arguments;
  arguments << p0;
  return JMethod< ::jace::proxy::java::util::List >("parse").invoke(*this, arguments);
}

/**
 * Creates a new null reference.
 * 
 * All subclasses of JObject should provide this constructor
 * for their own subclasses.
 */
ActionParser::ActionParser()
{}

ActionParser::ActionParser(jvalue value) ActionParser_INITIALIZER
{
  setJavaJniValue(value);
}

ActionParser::ActionParser(jobject object) ActionParser_INITIALIZER
{
  setJavaJniObject(object);
}

ActionParser::ActionParser(const ActionParser& object) ActionParser_INITIALIZER
{
  setJavaJniObject(object);
}

/**
 * The following methods are required to integrate this class
 * with the Jace framework.
 */
static boost::mutex javaClassMutex;
const JClass& ActionParser::staticGetJavaJniClass() throw (::jace::JNIException)
{
  static boost::shared_ptr<JClassImpl> result;
  boost::mutex::scoped_lock lock(javaClassMutex);
  if (result == 0)
    result = boost::shared_ptr<JClassImpl>(new JClassImpl("edu/stanford/graphics/parser/ActionParser"));
  return *result;
}

const JClass& ActionParser::getJavaJniClass() const throw (::jace::JNIException)
{
  return ActionParser::staticGetJavaJniClass();
}

END_NAMESPACE_6(jace, proxy, edu, stanford, graphics, parser)

BEGIN_NAMESPACE(jace)

#ifndef PUT_TSDS_IN_HEADER
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >::ElementProxy(jarray array, jvalue element, int _index): 
    ::jace::proxy::edu::stanford::graphics::parser::ActionParser(element), index(_index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, array));
  }
  template <> inline ElementProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >::ElementProxy(const jace::ElementProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >& proxy): 
    ::jace::proxy::edu::stanford::graphics::parser::ActionParser(proxy), index(proxy.index)
  {
    JNIEnv* env = attach();
    parent = static_cast<jarray>(newGlobalRef(env, proxy.parent));
  }
#endif
#ifndef PUT_TSDS_IN_HEADER
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >::JFieldProxy(jfieldID _fieldID, jvalue value, jobject _parent): 
    ::jace::proxy::edu::stanford::graphics::parser::ActionParser(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    if (_parent)
      parent = newGlobalRef(env, _parent);
    else
      parent = _parent;

    parentClass = 0;
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >::JFieldProxy(jfieldID _fieldID, jvalue value, jclass _parentClass): 
    ::jace::proxy::edu::stanford::graphics::parser::ActionParser(value), fieldID(_fieldID)
  {
    JNIEnv* env = attach();

    parent = 0;
    parentClass = static_cast<jclass>(newGlobalRef(env, _parentClass));
  }
  template <> inline JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >::JFieldProxy(const JFieldProxy< ::jace::proxy::edu::stanford::graphics::parser::ActionParser >& object): 
    ::jace::proxy::edu::stanford::graphics::parser::ActionParser(object)
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

