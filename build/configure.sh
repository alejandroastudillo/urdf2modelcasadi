# Declare TPUT dictionary for readability: BLACK=$(tput setaf 0), RED=$(tput setaf 1), GREEN=$(tput setaf 2), YELLOW=$(tput setaf 3), LIME_YELLOW=$(tput setaf 190), POWDER_BLUE=$(tput setaf 153), BLUE=$(tput setaf 4), MAGENTA=$(tput setaf 5), CYAN=$(tput setaf 6), WHITE=$(tput setaf 7), BOLD=$(tput bold), NORMAL=$(tput sgr0), BLINK=$(tput blink), REVERSE=$(tput smso), UNDERLINE=$(tput smul)
  declare -A TPUT=( ["CYAN"]=$(tput setaf 6) ["NORMAL"]=$(tput sgr0) ["BOLD"]=$(tput bold))
  print_title () {
      printf "\n%40s\n" "${TPUT[CYAN]}${TPUT[BOLD]} $1 ${TPUT[NORMAL]}"
  }

# Set variables used by CMake
  export CASADI_DIRECTORY="/home/alejandro/phd_software/casadi_source/build/install_matlab"
  export PINOCCHIO_INCLUDE="/opt/openrobots/include/"
  export EIGEN_INCLUDE="/usr/include/eigen3"
  export INSTALL_FOLDER="install_folder"

# If the CMakeCache.txt file exists, delete it.
  if test -f "CMakeCache.txt"; then
      print_title "########## Removing CMakeCache.txt ##########"
      rm CMakeCache.txt
      # ls | grep -v configure.sh | xargs rm -r # deletes everything, except configure.sh
      #rm -rf CMakeFiles
      #rm *.so Makefile urdf2model_casadi *.cmake
  fi

# Execute the cmake command, assigning the variable reference values (CASADI_DIR, PINOCCHIO_INC, EIGEN_INC).
  print_title "########## Executing CMake ##########"
  # cmake ../urdf2model -DCASADI_DIR=$CASADI_DIRECTORY -DPINOCCHIO_INC=$PINOCCHIO_INCLUDE -DEIGEN_INC=$EIGEN_INCLUDE
  # cmake ../urdf2model -DCASADI_DIR=$CASADI_DIRECTORY -DPINOCCHIO_INC=$PINOCCHIO_INCLUDE -DEIGEN_INC=$EIGEN_INCLUDE -DBUILD_UNIT_TESTS=ON -DDEBUG_MODE=ON
  cmake ../urdf2model -DCASADI_DIR=$CASADI_DIRECTORY -DPINOCCHIO_INC=$PINOCCHIO_INCLUDE -DEIGEN_INC=$EIGEN_INCLUDE -DCMAKE_INSTALL_PREFIX=$INSTALL_FOLDER

# Execute the make command
  print_title "########## Executing make ##########"
  make -j3

# Execute unit tests
  print_title "########## Executing unit tests ##########"
  # export BOOST_TEST_LOG_LEVEL="message"
  # # ctest -V
  make test ARGS="-j3" # -V

# Execute the make install command
  print_title "########## Installing the library ##########"
  make install
