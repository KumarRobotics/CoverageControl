\page quick_start Quick Start
\tableofcontents

The library uses `toml` files to specify the configuration parameters for the environment and the robot models.
See \ref params/coverage_control_params.toml for an example configuration file.

## Python Interface

Import the `CoverageControl` library and the `ClairvoyantCVT` algorithm.
```python
import sys
import CoverageControl # Main library
from CoverageControl import ClairvoyantCVT as CoverageAlgorithm
```

You can choose one of the following algorithms instead of `ClairvoyantCVT`:
- `ClairvoyantCVT`
- `CentralizedCVT`
- `DecentralizedCVT`
- `NearOptimalCVT`

Create a `CoverageControl::Parameters` object and load the configuration file:
```python
params = CoverageControl.Parameters() # for default parameters
params.load("params/coverage_control_params.toml") # load the configuration file
```

Create a simulation environment:
```python
env = CoverageControl.Environment(params)
```

Print the initial coverage cost:
```python
init_cost = env.GetObjectiveValue()
print("Initial Coverage cost: " + str('{:.2e}'.format(init_cost)))
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
        print("Error in step " + str(i))
        break

    if controller.IsConverged():
        print("Converged in step " + str(i))
        break
```

Print improvement in cost:
```python
current_cost = env.GetObjectiveValue()
print("Improvement %: " + str('{:.2f}'.format(100 * (init_cost - current_cost)/init_cost)))
```

See \ref python/tests/coverage_simple.py and \ref python/tests/coverage_class.py for complete examples.

---

## C++ Interface

Include the `CoverageControl` library, algorithms, and other necessary headers:
```cpp
#include <CoverageControl/parameters.h>
#include <CoverageControl/typedefs.h>
#include <CoverageControl/world_idf.h>
#include <CoverageControl/coverage_system.h>

#include <CoverageControl/algorithms/near_optimal_cvt.h>

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
auto init_objective = env->GetObjectiveValue();
std::cout << "Initial objective: " << init_objective << std::endl;
```

To plot the initial environment use the function `PlotInitMap(dir_name, init_map)` :
```cpp
env.PlotInitMap("./", "init_map");
```

Create a controller using the `CoverageAlgorithm` and the environment:
```cpp
CoverageAlgorithm algorithm(params, *env);
```

Execute the algorithm:
```cpp
for (int ii = 0; ii < params.pEpisodeSteps; ++ii) {
    algorithm.ComputeActions();
    auto actions = algorithm.GetActions();
    if (env->StepActions(actions)) {
        std::cout << "Invalid action" << std::endl;
        break;
    }
    if (algorithm.IsConverged() {
        break;
    }
}
```

Print improvement in cost:
```cpp
auto current_objective = env->GetObjectiveValue();
std::cout << "Improvement %: "
          << (init_objective - final_objective) / init_objective * 100
          << std::endl;
```

See \ref cppsrc/main/coverage_algorithm.cpp for a complete example.








