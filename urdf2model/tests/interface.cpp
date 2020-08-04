
#define BOOST_TEST_MODULE INTERFACE_TESTS
#include <boost/test/unit_test.hpp>

#include "model_interface.hpp"

#define MAKE_STR(x) _MAKE_STR(x)
#define _MAKE_STR(x) #x

#ifdef MODELS_DIR
    #define Rob_models_dir MAKE_STR(MODELS_DIR)
#else
    #define Rob_models_dir "../../urdf2model/models"
#endif

std::string filename = Rob_models_dir"/kortex_description/urdf/JACO3_URDF_V11.urdf";

BOOST_AUTO_TEST_CASE(Reduced_model)
{
  // Instantiate model and data objects
    mecali::Model         model;                                        // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1ModelTpl.html
    mecali::Data          data = pinocchio::Data(model);                // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/structpinocchio_1_1DataTpl.html

    pinocchio::urdf::buildModel(filename,model);    // https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio_1_1urdf.html
  // Set the gravity applied to the model
    model.gravity.linear(pinocchio::Model::gravity981);     // options: model.gravity.setZero(), model.gravity.linear( Eigen::Vector3d(0,0,-9.81));
  // initialize the data structure for the model
    data = pinocchio::Data(model);

    std::vector<std::string> list_of_joints_to_lock_by_name;
    list_of_joints_to_lock_by_name.push_back("Actuator2");
    list_of_joints_to_lock_by_name.push_back("Actuator4"); // It can be in the wrong order
    list_of_joints_to_lock_by_name.push_back("Actuator5");
    // list_of_joints_to_lock_by_name.push_back("blabla"); // Joint not in the model

    // Print the list of joints to remove + retrieve the joint id
    std::vector<mecali::Index> list_of_joints_to_lock_by_id;
    for(std::vector<std::string>::const_iterator it = list_of_joints_to_lock_by_name.begin();
        it != list_of_joints_to_lock_by_name.end(); ++it)
    {
      const std::string & joint_name = *it;
      if(model.existJointName(joint_name)) // do not consider joint that are not in the model
        list_of_joints_to_lock_by_id.push_back(model.getJointId(joint_name));
    }

    Eigen::VectorXd q_rand = pinocchio::randomConfiguration(model);

    mecali::Serial_Robot reduced_robot_model;
    reduced_robot_model.import_reduced_model(filename, list_of_joints_to_lock_by_name,q_rand);

    mecali::Model reduced_model = pinocchio::buildReducedModel(model,list_of_joints_to_lock_by_id,q_rand);

    BOOST_CHECK(reduced_robot_model.n_joints == reduced_model.njoints);
    BOOST_CHECK(reduced_robot_model.gravity.isApprox(reduced_model.gravity.linear_impl()));


}
