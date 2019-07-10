#ifndef FUN_CODE_GENERATION_H_INCLUDED
#define FUN_CODE_GENERATION_H_INCLUDED

// #include <casadi/casadi.hpp>
#include "../utils/common.hpp"

namespace mecali
{
  void generate_code(casadi::Function& func, std::string name);
  void generate_code(casadi::Function& func, std::string name, Dictionary& opts);
}

#endif // FUN_CODE_GENERATION_H_INCLUDED
