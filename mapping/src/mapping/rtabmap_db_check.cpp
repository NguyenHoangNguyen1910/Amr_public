#include <iostream>
#include <stdexcept>
#include <string>

#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: rtabmap_db_check DATABASE\n";
    return 2;
  }

  try {
    rtabmap::ParametersMap parameters;
    parameters[rtabmap::Parameters::kMemIncrementalMemory()] = "true";

    rtabmap::Rtabmap rtabmap;
    rtabmap.init(parameters, argv[1], false);
    rtabmap.close(false);
    std::cout << "rtabmap_open=ok database=" << argv[1] << "\n";
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "rtabmap_open=error " << exception.what() << "\n";
    return 1;
  }
}
