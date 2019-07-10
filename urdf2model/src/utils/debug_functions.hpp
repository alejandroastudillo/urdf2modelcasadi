/*
TODO: Check that vec_size is at least 2, or handle vec_size = 1
*/

#ifndef DEBUG_FUNCTIONS_H_INCLUDED
#define DEBUG_FUNCTIONS_H_INCLUDED

#include <stdexcept>
#include <string>
#include <iostream>
#include <iomanip>
// #include <Eigen/Core>


template <typename T>
void print_indent(std::string var_name, T var_value,               int indent)
{
      std::cout << std::setprecision(3) << std::left << std::setw(indent) << var_name << std::setw(8) << var_value << std::endl;
}
void print_indent(std::string var_name, Eigen::VectorXd var_value, int indent)
{
      int vec_size = var_value.size();
      std::stringstream ss;

      for(int i=0; i<vec_size-1; i++)
      {
        ss << std::setprecision(3) << std::left << std::setw(10) << var_value[i]; // ss << var_value[i] << "  ";
      }
      ss << std::setprecision(3) << std::left << var_value[vec_size-1];

      std::cout << std::left << std::setw(indent) << var_name << std::setw(indent) << ss.str() << std::endl;
}
void print_indent(std::string var_name, Eigen::Vector3d var_value, int indent)
{
      int vec_size = var_value.size();
      std::stringstream ss;

      for(int i=0; i<vec_size-1; i++)
      {
        ss << std::setprecision(3) << std::left << std::setw(10) << var_value[i]; // ss << var_value[i] << "  ";
      }
      ss << std::setprecision(3) << std::left << var_value[vec_size-1];

      std::cout << std::left << std::setw(indent) << var_name << std::setw(indent) << ss.str() << std::endl;
}

void custom_assert(bool expr, std::string msg)
{
  if (expr ==  false)
  {
    throw std::invalid_argument(msg);
  }
}


#endif // DEBUG_FUNCTIONS_H_INCLUDED
