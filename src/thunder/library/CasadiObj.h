#ifndef CASADIOBJ_H
#define CASADIOBJ_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>

namespace regrob{
    
    /* Class CasadiObj */
    class CasadiObj {
            
        protected:

            /* Initialize variables */
            virtual void init() = 0;
            /* Initialize and resize variables and function (in particular attributes of class)*/
            virtual void initVarsFuns() = 0;
            /* Update result of casadi function */
            virtual void compute() = 0;
            /* Create casadi function */
            virtual void init_casadi_functions() = 0;
            /* Set arguments to update the result */
            virtual void setArguments(const Eigen::VectorXd& var) = 0;
            /* Generate code */
            virtual void generate_code(std::string& path) = 0;
            /* Get function name used to generate code */
            virtual std::vector<std::string> getFunctionsName() = 0;
            /* Get casadi function */
            virtual std::vector<casadi::Function> getCasadiFunctions() = 0;

            /* Empty costructor */
            CasadiObj(){};
            /* Destructor */
            ~CasadiObj(){};

            /* Function to transform casadi element to double */
            static double mapFunction(const casadi::SXElem& elem) {return static_cast<double>(casadi::SXElem(elem));};

        public:

            /* Generate code with merged casadi functions */
            void generate_mergeCode(std::vector<casadi::Function>& functions, const std::string& savePath, const std::string& name_file){
                
                int dim_vec = functions.size();

                // Options for c-code auto generation
                casadi::Dict opts = casadi::Dict();
                opts["cpp"] = true;
                opts["with_header"] = true;
                
                // generate functions in c code
                casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(name_file, opts);

                for (int i=0; i<dim_vec; i++) {
                    myCodeGen.add(functions[i]);
                }
                myCodeGen.generate(savePath);
            };
    
    };
}

#endif