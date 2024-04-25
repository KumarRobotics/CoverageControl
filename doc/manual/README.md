# Home
\tableofcontents
\htmlonly
<img style="float: right;width: 20vw; min-width: 330px;border:1px solid #000000; margin-left: 0.5em; " src="LPAC.gif"/>
\endhtmlonly

Coverage control is the problem of navigating a robot swarm to collaboratively monitor features or a phenomenon of interest not known _a priori_.
The relative importance of the features is modeled as an underlying scalar field known as the importance density field (IDF).
Coverage control with unknown features of interest has applications in various domains, including search and rescue, surveillance, and target tracking.

The library provides a simulation environment, algorithms, and GNN-based architectures for the coverage control problem.  

**Key features:**  
- The core library is written in `C++` and `CUDA` to handle large-scale simulations
- There are `python` bindings that interface with the core library
- Several Centroidal Voronoi Tessellation (CVT)-based algorithms (aka Lloyd's algorithms)
- Learnable Perception-Action-Communication (LPAC) architecture for the coverage control problem is implemented in `PyTorch` and `PyTorch Geometric`

---

## Getting Started
The library is available as a `pip` package. To install the package, run the following command:
```bash
pip install coverage_control
```

See [Installation](https://kumarrobotics.github.io/CoverageControl/installation.html) for more details on installation.

See [Quick Start](https://kumarrobotics.github.io/CoverageControl/quick_start.html) guide for a quick introduction to the library.

---

## Citation
```
@article{agarwal2024lpac,
      title         =   {LPAC: Learnable Perception-Action-Communication Loops with
                            Applications to Coverage Control}, 
      author        =   {Saurav Agarwal and Ramya Muthukrishnan and 
                            Walker Gosrich and Vijay Kumar and Alejandro Ribeiro},
      year          =   {2024},
      eprint        =   {2401.04855},
      archivePrefix =   {arXiv},
      primaryClass  =   {cs.RO}
}
```
@htmlonly
<br>
@endhtmlonly

> [LPAC: Learnable Perception-Action-Communication Loops with Applications to Coverage Control.](https://doi.org/10.48550/arXiv.2401.04855)  
> Saurav Agarwal, Ramya Muthukrishnan, Walker Gosrich, Vijay Kumar, and Alejandro Ribeiro.  
> arXiv preprint arXiv:2401.04855 (2024).


## Acknowledgements
- [PyTorch](https://pytorch.org/)
- [PyTorch Geometric](https://pytorch-geometric.readthedocs.io/en/latest/)
- [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)
- [pybind11](https://pybind11.readthedocs.io/en/stable/)
- [CGAL](https://www.cgal.org/)
- [JSON for Modern C++](https://github.com/nlohmann/json)
- [CUDA Samples](https://github.com/NVIDIA/cuda-samples)
- [gnuplot-iostream](http://stahlke.org/dan/gnuplot-iostream/)
- [hungarian-algorithm-cpp](https://github.com/mcximing/hungarian-algorithm-cpp)
- [toml++](https://marzer.github.io/tomlplusplus/index.html)


## Support and Funding
The work was performed at the [GRASP Laboratory](https://www.grasp.upenn.edu/) and the [Alelab](https://alelab.seas.upenn.edu/), University of Pennsylvania, USA.

This work was supported in part by grants ARL DCIST CRA W911NF-17-2-0181 and ONR N00014-20-1-2822.


## Contributors
- [Saurav Agarwal](https://www.saurav.fyi/)
- Ramya Muthukrishnan


## License
The library is licensed under the [GPL-3.0 License](https://www.gnu.org/licenses/gpl-3.0.html).
The documentation is not under the GPL-3.0 License and is licensed under the [CC BY-NC-SA 4.0 License](https://creativecommons.org/licenses/by-nc-sa/4.0/).
