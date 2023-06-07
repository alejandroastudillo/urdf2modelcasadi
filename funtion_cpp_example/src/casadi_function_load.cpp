#include <stdio.h>
#include <dlfcn.h>
#include <iostream>
#include <chrono>


using namespace std;
using namespace chrono;

typedef long long int casadi_int;
typedef int (*eval_t)(const double**, double**, casadi_int*, double*, int);

int indy7_G()
{
// Load the shared library
    void* handle = dlopen("indy7_G.so", RTLD_LAZY);
    if (handle == 0) {
        printf("Cannot open indy7_G.so, error: %s\n", dlerror());
        return 1;
    }

    // Reset error
    dlerror();

    // Function evaluation
    eval_t eval = (eval_t)dlsym(handle, "generalized_gravity");
    if (dlerror()) {
        printf("Failed to retrieve \"generalized_gravity\" function.\n");
        return 1;
    }

    // Allocate input/output buffers and work vectors dlrj
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[6];
    double* res[6];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[] = {0.0, 0.7, 0.0, 0.0, 0.0, 0.0};
    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[6];
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    system_clock::time_point start = system_clock::now();
    if (eval(arg, res, iw, w, mem)) {
        printf("Function evaluation failed.\n");
        return 1;
    }
    system_clock::time_point end = system_clock::now();

    // Print the result
    printf("Result:\n");
    for (casadi_int i = 0; i < sz_res; ++i) {
        printf("%g ", output_values[i]);
    }
    printf("\n");

    nanoseconds nano = end - start;
    cout<<"computation time for \"G\": "<< nano.count()/1000.0<<"[us]"<<endl;

    // Free the handle
    dlclose(handle);

    return 0;
}

int indy7_M()
{
// Load the shared library
    void* handle = dlopen("indy7_M.so", RTLD_LAZY);
    if (handle == 0) {
        printf("Cannot open indy7_M.so, error: %s\n", dlerror());
        return 1;
    }

    // Reset error
    dlerror();

    // Function evaluation
    eval_t eval = (eval_t)dlsym(handle, "M");
    if (dlerror()) {
        printf("Failed to retrieve \"M\" function.\n");
        return 1;
    }

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[6];
    double* res[6];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[36]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    system_clock::time_point start = system_clock::now();
    if (eval(arg, res, iw, w, mem)) {
        printf("Function evaluation failed.\n");
        return 1;
    }
    system_clock::time_point end = system_clock::now();
    
    
    // Print the result
    printf("Result:\n");
    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {
            printf("%g ", output_values[i * sz_res + j]);
        }
        printf("\n");
    }
    nanoseconds nano = end - start;
    cout<<"computation time for \"M\": "<< nano.count()/1000.0<<"[us]"<<endl;
    // Free the handle
    dlclose(handle);

    return 0;
}

int indy7_C()
{
    time_t tstart, tend;
// Load the shared library
    void* handle = dlopen("indy7_C.so", RTLD_LAZY);
    if (handle == 0) {
        printf("Cannot open indy7_C.so, error: %s\n", dlerror());
        return 1;
    }

    // Reset error
    dlerror();

    // Function evaluation
    eval_t eval = (eval_t)dlsym(handle, "coriolis");
    if (dlerror()) {
        printf("Failed to retrieve \"C\" function.\n");
        return 1;
    }

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[6];
    double* res[6];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_pos[] = {0.0, 0.7, 0.0, 0.0, 0.0, 0.0};
    double input_vel[] = {0.0, 0.7, 0.0, 0.0, 0.0, 0.0};

    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i] = &input_pos[i];
    }
    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i+6] = &input_vel[i];
    }

    // Set output buffers
    double output_values[36]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    system_clock::time_point start = system_clock::now();
    if (eval(arg, res, iw, w, mem)) {
        printf("Function evaluation failed.\n");
        return 1;
    }
    system_clock::time_point end = system_clock::now();

    // Print the result
    printf("Result:\n");
    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {
            printf("%g ", output_values[i * sz_res + j]);
        }
        printf("\n");
    }

    nanoseconds nano = end - start;
    cout<<"computation time for \"C\": "<< nano.count()/1000.0<<"[us]"<<endl;

    // Free the handle
    dlclose(handle);

    return 0;
}
/*
int indy7_FK()
{
// Load the shared library
    void* handle = dlopen("indy7_fk.so", RTLD_LAZY);
    if (handle == 0) {
        printf("Cannot open indy7_fk.so, error: %s\n", dlerror());
        return 1;
    }

    // Reset error
    dlerror();

    // Function evaluation
    eval_t eval = (eval_t)dlsym(handle, "M");
    if (dlerror()) {
        printf("Failed to retrieve \"M\" function.\n");
        return 1;
    }

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[6];
    double* res[6];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[36]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    system_clock::time_point start = system_clock::now();
    if (eval(arg, res, iw, w, mem)) {
        printf("Function evaluation failed.\n");
        return 1;
    }
    system_clock::time_point end = system_clock::now();
    
    
    // Print the result
    printf("Result:\n");
    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {
            printf("%g ", output_values[i * sz_res + j]);
        }
        printf("\n");
    }
    nanoseconds nano = end - start;
    cout<<"computation time for \"M\": "<< nano.count()/1000.0<<"[us]"<<endl;
    // Free the handle
    dlclose(handle);

    return 0;
}
*/
int indy7_J_b()
{
// Load the shared library
    void* handle = dlopen("indy7_J_b.so", RTLD_LAZY);
    if (handle == 0) {
        printf("Cannot open indy7_J_b.so, error: %s\n", dlerror());
        return 1;
    }

    // Reset error
    dlerror();

    // Function evaluation
    eval_t eval = (eval_t)dlsym(handle, "J_b");
    if (dlerror()) {
        printf("Failed to retrieve \"J_b\" function.\n");
        return 1;
    }

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[6];
    double* res[6];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[36]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    system_clock::time_point start = system_clock::now();
    if (eval(arg, res, iw, w, mem)) {
        printf("Function evaluation failed.\n");
        return 1;
    }
    system_clock::time_point end = system_clock::now();
    
    
    // Print the result
    printf("Result:\n");
    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {
            printf("%g ", output_values[i * sz_res + j]);
        }
        printf("\n");
    }
    nanoseconds nano = end - start;
    cout<<"computation time for \"J_b\": "<< nano.count()/1000.0<<"[us]"<<endl;
    // Free the handle
    dlclose(handle);

    return 0;
}
int indy7_J_s()
{
// Load the shared library
    void* handle = dlopen("indy7_J_s.so", RTLD_LAZY);
    if (handle == 0) {
        printf("Cannot open indy7_J_s.so, error: %s\n", dlerror());
        return 1;
    }

    // Reset error
    dlerror();

    // Function evaluation
    eval_t eval = (eval_t)dlsym(handle, "J_s");
    if (dlerror()) {
        printf("Failed to retrieve \"J_s\" function.\n");
        return 1;
    }

    // Allocate input/output buffers and work vectors
    casadi_int sz_arg = 6;
    casadi_int sz_res = 6;
    casadi_int sz_iw = 0;
    casadi_int sz_w = 0;

    const double* arg[6];
    double* res[6];
    casadi_int iw[sz_iw];
    double w[sz_w];

    // Set input values
    double input_values[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (casadi_int i = 0; i < sz_arg; ++i) {
        arg[i] = &input_values[i];
    }

    // Set output buffers
    double output_values[36]; // 6x6 matrix
    for (casadi_int i = 0; i < sz_res; ++i) {
        res[i] = &output_values[i];
    }

    // Evaluate the function
    int mem = 0;  // No thread-local memory management
    
    system_clock::time_point start = system_clock::now();
    if (eval(arg, res, iw, w, mem)) {
        printf("Function evaluation failed.\n");
        return 1;
    }
    system_clock::time_point end = system_clock::now();
    
    
    // Print the result
    printf("Result:\n");
    for (casadi_int i = 0; i < sz_res; ++i) {
        for (casadi_int j = 0; j < sz_res; ++j) {
            printf("%g ", output_values[i * sz_res + j]);
        }
        printf("\n");
    }
    nanoseconds nano = end - start;
    cout<<"computation time for \"J_s\": "<< nano.count()/1000.0<<"[us]"<<endl;
    // Free the handle
    dlclose(handle);

    return 0;
}
int main() {
    
    indy7_M();
    indy7_C();
    indy7_G();
    indy7_J_b();
    indy7_J_s();
    return 0;
}
