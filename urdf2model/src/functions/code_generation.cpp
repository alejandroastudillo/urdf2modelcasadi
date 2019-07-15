#include "code_generation.hpp"

namespace mecali
{
  void generate_code(casadi::Function& func, std::string name)
  {
    func.generate(name+".c");
  }

  void generate_code(casadi::Function& func, std::string name, Dictionary& opts)
  {
    #ifdef DEBUG
      if (opts.find("c") != opts.end() || opts.find("python") != opts.end() || opts.find("matlab") != opts.end())
      {
        std::cout << "** Code generating function \"" << func.name() << "\" in: " << std::endl;
      }
    #endif

    if (opts.find("c") != opts.end())  // If there exists a key "c" inside opts
    {
      if (opts["c"])      // If key "c" was set to true
      {
          #ifdef DEBUG
            std::cout << "\t C ... \t\t" << name << ".c" << std::endl;
          #endif
          func.generate(name+".c");
      }
    }
    if (opts.find("python") != opts.end())  // If there exists a key "python" inside opts
    {
      if (opts["python"])
      {
          // #ifdef DEBUG
          //   std::cout << "\t Python ... \t" << name << ".py" << std::endl;
          // #endif
          // func.generate(name+".py");
          std::cout << "\n\t ########### PYTHON CODE-GENERATION NOT YET SUPPORTED ###########" << std::endl;
      }
    }
    if (opts.find("matlab") != opts.end())  // If there exists a key "matlab" inside opts
    {
      if (opts["matlab"])
      {
          // #ifdef DEBUG
          //   std::cout << "\t MATLAB ... \t" << name << ".m" << std::endl;
          // #endif
          // func.generate(name+".m");
          std::cout << "\n\t ########### MATLAB CODE-GENERATION NOT YET SUPPORTED ###########" << std::endl;
      }
    }


  }
}
