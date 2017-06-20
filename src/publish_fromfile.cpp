/**
 * publish_fromfile.cpp
 *
 * Wraps the pff_run_function code with a main() entry point so that the code
 * can be started as a rosnode without involving the unit tests
 *
 */

#include "pff_run_functions.hpp"

int main(int argc, char **argv)
{
  return runner(argc,argv);
}
