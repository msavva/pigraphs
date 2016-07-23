#pragma once

namespace sg {
namespace ui {

typedef int ErrorCode;

struct ErrorCodes {
  static const ErrorCode E_OK = 0;
  static const ErrorCode E_GENERAL_ERROR = -1;
  static const ErrorCode E_UNKNOWN_CMD = -2;
  static const ErrorCode E_INVALID_ARGS = -3;
  static const ErrorCode E_UNSUPPORTED = -4;
};

}  // namespace ui
}  // namespace sg


