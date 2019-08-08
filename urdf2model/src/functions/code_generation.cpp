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
      if (opts.find("c") != opts.end() || opts.find("save") != opts.end())
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
    if (opts.find("save") != opts.end())  // If there exists a key "save" inside opts
    {
      if (opts["save"])
      {
          #ifdef DEBUG
            std::cout << "\t CasADi save ... \t" << name << ".m" << std::endl;
          #endif
          func.save(name+".casadi");
      }
    }


  }
}
