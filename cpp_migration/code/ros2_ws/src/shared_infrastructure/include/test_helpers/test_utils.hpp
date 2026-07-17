#include <memory>
#include <boost/process.hpp>

namespace bp = boost::process;

class TestUtils {
public:
    static std::unique_ptr<bp::child> createDensityRegressorProcess();
};