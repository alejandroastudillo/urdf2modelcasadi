#include <string>
#include <iostream>
#include <iomanip>
#include <Eigen/Core>

template <typename T>
void print_indent(std::string var_name, T var_value, int indent)
{
      std::cout << std::left << std::setw(indent) << var_name << std::setw(indent) << var_value << std::endl;
}
void print_indent(std::string var_name, Eigen::VectorXd var_value, int indent)
{
      int vec_size = var_value.size();
      std::stringstream ss;

      ss << "[ ";
      for(int i=0; i<vec_size-1; i++)
      {
        ss << var_value[i] << "  ";
      }
      ss << var_value[vec_size-1] << " ]";

      std::cout << std::left << std::setw(indent) << var_name << std::setw(indent) << ss.str() << std::endl;
}
