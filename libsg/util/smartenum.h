#pragma once

// Smart enum from http://saltares.com/blog/computing/smart-enums-in-c-or-what-is-this-madness/
#include <cstring>

#define SMARTENUM_VALUE(typeName, value) k##typeName##_##value, 
#define SMARTENUM_STRING(typeName, value) #value, 

#define SMARTENUM_DEFINE_ENUM(typeName, values)                 \
  enum typeName { values(SMARTENUM_VALUE, typeName) k##typeName##_Count, };

#define SMARTENUM_DECLARE_NAMES(typeName, values)               \
  const char* typeName##Names [];
#define SMARTENUM_DEFINE_NAMES(typeName, values)                \
  const char* typeName##Names [] = { values(SMARTENUM_STRING, typeName) };

#define SMARTENUM_DECLARE_GET_VALUE_FROM_STRING(typeName)       \
  typeName get##typeName##FromString(const char* str);

#define SMARTENUM_DEFINE_GET_VALUE_FROM_STRING(typeName)       \
  typeName get##typeName##FromString(const char* str)           \
  {                                                             \
    for (int i = 0; i < k##typeName##_Count; ++i)               \
      if (!strcmp(##typeName##Names[i], str))                   \
        return (##typeName##)i;                                 \
    return k##typeName##_Count;                                 \
  }

#define getStringFromEnumValue(typeName, value) typeName##Names[##value]
#define getEnumValueFromString(typeName, name) get##typeName##FromString(##name)

// Variant of smart enum using enum class
#define SMARTENUMCLASS_VALUE(typeName, value) k##value, 
#define SMARTENUMCLASS_DEFINE_ENUM(typeName, values)            \
  enum class typeName { values(SMARTENUMCLASS_VALUE, typeName) kCount, };

#define SMARTENUMCLASS_DECLARE_NAMES(typeName, values)          \
  const char* typeName##Names [];
#define SMARTENUMCLASS_DEFINE_NAMES(typeName, values)           \
  const char* typeName##Names [] = { values(SMARTENUM_STRING, typeName) };

#define SMARTENUMCLASS_DECLARE_GET_VALUE_FROM_STRING(typeName)  \
  typeName get##typeName##FromString(const char* str);

#define SMARTENUMCLASS_DEFINE_GET_VALUE_FROM_STRING(typeName,kc)\
  typeName get##typeName##FromString(const char* str)           \
  {                                                             \
    int k = (int) kc;                                           \
    for (int i = 0; i < k; ++i)                                 \
      if (!strcmp(##typeName##Names[i], str))                   \
        return (##typeName##)i;                                 \
    return (##typeName) k;                                      \
  }
