See [Installation From Source](https://KumarRobotics.github.io/CoverageControl/installation.html) for instructions on how to install from source.
The core of the project is written in `C++` and the Python bindings are generated using `pybind11`. 
The `Pytorch` neural network is purely written in `Python` and uses the core `C++` code for the coverage control simulation and testing.

The repository is partially oraganized using the [Scientific Python Developer Guide][spc-dev-intro].
[spc-dev-intro]: https://learn.scientific-python.org/development/

## License

Note that the repository is licensed under the GPL-3.0 License.
This is primarily because the project uses [CGAL](https://www.cgal.org/) which is licensed under the GPL-3.0 License.
Thus, any contributions to the project must also be licensed under the GPL-3.0 License.

## Setting up a development environment manually

You can set up a development environment by running:

```bash
python3 -m venv .venv
source ./.venv/bin/activate
pip install -v -e .
```

## Building docs

The repository uses `doxygen` to build the documentation.
Run the following command to build the documentation from the base directory of the repository:

```bash
doxygen doc/Doxyfile
```

This creates a static website in the `doc/html` directory.

```bash
cd doc/html
firefox index.html
```
