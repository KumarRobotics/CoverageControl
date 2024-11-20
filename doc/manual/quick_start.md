\page quick_start Quick Start
\tableofcontents

# Parameters
The library uses `toml` files to specify the configuration parameters for the environment and the robot models.  
The class `CoverageControl::Parameters` is used to load the configuration file and access the parameters.

```python
params = CoverageControl.Parameters("params/coverage_control_params.toml")
```
See \ref params/coverage_control_params.toml for an example configuration file.

# Python Interface

Import the `coverage_control` library and the `ClairvoyantCVT` algorithm.
```python
import coverage_control as cc
from coverage_control import ClairvoyantCVT as CoverageAlgorithm
```

You can choose one of the following algorithms instead of `ClairvoyantCVT`:
- `ClairvoyantCVT`
- `CentralizedCVT`
- `DecentralizedCVT`
- `NearOptimalCVT`

Create a `CoverageControl::Parameters` object and load the configuration file:
```python
params = cc.Parameters() # for default parameters
```

Create a simulation environment:
```python
env = cc.CoverageSystem(params)
```

Plot the initial environment (needs `gnuplot` installed):
```python
env.PlotInitMap("init_map");
```

Print the initial coverage cost:
```python
init_cost = env.GetObjectiveValue()
print(f"Initial Coverage cost: {init_cost:.2e}")
```

Create a controller using the `CoverageAlgorithm` and the environment:
```python
controller = CoverageAlgorithm(params, env)
```

Execute the algorithm:
```python
for i in range(0, params.pEpisodeSteps):
    # Compute actions to be taken by the robots
    controller.ComputeActions();
    # Get actions from the controller
    actions = controller.GetActions()

    # Send actions to the environment
    if env.StepActions(actions):
        print(f"Error in step {i}")
        break

    if controller.IsConverged():
        print(f"Converged in step {i}")
        break

```

Print improvement in cost:
```python
current_cost = env.GetObjectiveValue()
print(f"Improvement %: {100 * (init_cost - current_cost)/init_cost:.2f}")
```

Plot the final state of the environment:
```cpp
env.PlotSystemMap("final_map");
```


See \ref python/system_env/coverage_simple.py and \ref python/system_env/coverage_class.py for complete examples.

---

# C++ Interface

\note The C++ interface is not available in the pip package. You need to build the library from source. See the [Installation](https://kumarrobotics.github.io/CoverageControl/installation.html#installation-from-source) guide for instructions.


Include the `CoverageControl` library, algorithms, and other necessary headers:
```cpp
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/coverage_system.h>

#include <CoverageControl/algorithms/clairvoyant_cvt.h>

#include <iostream>
#include <memory>
#include <string>
```

Typedefs and using statements for easier access to the library classes:
```cpp
typedef CoverageControl::ClairvoyantCVT CoverageAlgorithm;

using CoverageControl::Point2;
using CoverageControl::PointVector;
using CoverageControl::Parameters;
using CoverageControl::WorldIDF;
using CoverageControl::CoverageSystem;
```

Inside the `main` function, create a `Parameters` object:
```cpp
Parameters params;
```

Create coverage system environment:
```cpp
CoverageSystem env(params);
auto init_objective = env.GetObjectiveValue();
std::cout << "Initial objective: " << init_objective << std::endl;
```

Plot the initial environment:
```cpp
env.PlotInitMap("init_map"); // Creates "init_map.png"
```

Create a controller using the `CoverageAlgorithm` and the environment:
```cpp
CoverageAlgorithm algorithm(params, env);
```

Execute the algorithm:
```cpp
for (int i = 0; i < params.pEpisodeSteps; ++i) {
    algorithm.ComputeActions();
    auto actions = algorithm.GetActions();
    if (env.StepActions(actions)) {
        std::cout << "Invalid action" << std::endl;
        break;
    }
    if (algorithm.IsConverged()) {
        break;
    }
}
```

Print improvement in cost:
```cpp
auto final_objective = env.GetObjectiveValue();
std::cout << "Improvement %: "
          << (init_objective - final_objective) / init_objective * 100
          << std::endl;
```

Plot the final state of the environment:
```cpp
env.PlotSystemMap("final_map"); // Creates "final_map.png"
```

See \ref cppsrc/main/coverage_algorithm.cpp for a complete example.

## Compile and Run

Create a `CMakeLists.txt` file with the following content:
```python
cmake_minimum_required(VERSION 3.16)
project(coveragecontrol)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(CoverageControl REQUIRED)

add_executable(coveragecontrol coveragecontrol.cpp) # Replace with your source file
target_link_libraries(coveragecontrol PRIVATE CoverageControl::CoverageControl)
install(TARGETS coveragecontrol DESTINATION ${CMAKE_INSTALL_BINDIR})
```

Build the program:
```bash
mkdir build
cd build
cmake ..
make
```

Run the program:
```bash
./coveragecontrol
```

