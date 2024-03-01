\page algorithms CVT Baseline Algorithms
\tableofcontents

## Centroidal Voronoi Tessellation
We discuss an iterative gradient descent algorithm that uses the centroidal Voronoi tessellation (CVT) to generate a robot configuration that provides good coverage of the environment.
The algorithm, also referred to as Lloyd's algorithm, is widely used to solve the coverage control problem.
It relies on computing the Voronoi partition (or tessellation) of the region with respect to the locations of the robots.
The Voronoi partition can be computed in \f$\mathcal O(N\log N)\f$ time using sweep line algorithm, and efficient implementations are available.
The Voronoi partition~\f$\mathcal P\f$ can be defined as:

\f{eqnarray*}{
		\mathcal P &=& \{P_i \mid i \in \mathcal V\},\\
		P_i &=& \{\mathbf q \in \mathcal W \mid \|\mathbf p_i - \mathbf q\| \le \|\mathbf p_j - \mathbf q\|, \forall j \in \mathcal V\}.
\f}

\note Vornoi partitioning is computed in the class \ref CoverageControl::Voronoi and uses the structure CoverageControl::VoronoiCell to represent the cells. The computations are performed using the CGAL library.

---
## Control Law
Given an importance density field (IDF)  \f$\Phi: \mathcal W \mapsto \mathbf{R}\f$ and a Voronoi cell \f$P_i\f$ for robot \f$i\f$, we can compute the generalized mass, centroid, and the polar moment of inertia as:
\f{eqnarray*}{
    m_i &=& \int_{P_i} \Phi(\mathbf q) \, d\mathbf q,\\
    \mathbf c_i &=& \frac{1}{m_i} \int_{P_i} \mathbf q \Phi(\mathbf q) \, d\mathbf q,\\
    I_i &=& \int_{P_i} \|\mathbf q - \mathbf c_i\|^2 \Phi(\mathbf q) \, d\mathbf q.
\f}

The objective function for the coverage control problem, with \f$f(\|\mathbf p_i - \mathbf q\|) = \|\mathbf p_i - \mathbf q\|^2\f$, can be rewritten as:
\f{eqnarray*}{
		\mathcal J(\mathcal P) &=& \sum_{i \in \mathcal V} \int_{P_i} \|\mathbf p_i - \mathbf q\|^2 \Phi(\mathbf q) \, d\mathbf q\\
		&=& \sum_{i \in \mathcal V} I_i + \sum_{i \in \mathcal V} m_i \|\mathbf p_i - \mathbf c_i\|^2.
\f}

Taking the partial derivative of the objective function with respect to the location of the robot \f$i\f$, we get:
\f{eqnarray*}{
		\frac{\partial \mathcal J(\mathcal P)}{\partial \mathbf p_i} = 2m_i (\mathbf p_i - \mathbf c_i)
\f}

The partial derivates vanish at the centroid of the Voronoi cell, i.e., \f$\mathbf p_i = \mathbf c_i\f$; thus, the centroid of the Voronoi cell is the local minimum of the objective function.
Hence, we can write a control law that drives the robot towards the centroid of the Voronoi cell as:
\f{eqnarray*}{
        \mathbf u_i = \dot{\mathbf p}_i = -k (\mathbf p_i - \mathbf c_i).
\f}
Here, \f$k\f$ is a positive gain for the control law.
The control law in has nice convergence properties; it is guaranteed to converge to a local minimum of the objective function.

\note An abstract class \ref CoverageControl::AbstractController is provided, which can be used to implement different control algorithms. The class defines `GetActions()` and `ComputeActions()` methods as pure virtual functions.

---
## Baseline Algorithms
The algorithm can now be expressed as an iteration of the following steps until convergence or for a maximum number of iterations:
1. Compute the Voronoi partition \f$\mathcal P\f$ of the region with respect to the locations of the robots.
2. Compute the mass centroids \f$\mathbf c_i\f$ for each Voronoi cell \f$P_i\f$.
3. Move each robot towards the centroid of the Voronoi cell using the control law.

We can define different variants of the above algorithm with the same control law, depending on the information available to the robots for computing the Voronoi partition and the centroids.
In this paper, we refer to the algorithms as variants of CVT.

**Clairvoyant:** The clairvoyant is a centralized algorithm based on CVT.
It has perfect knowledge of the positions of all the robots at all times.
Thus, it can compute the exact Voronoi partition.
It also has complete knowledge of the IDF for the centroid computation for each Voronoi cell.
Although the algorithm is not optimal, it generally computes solutions that are very close to the optimal solution.  
See: CoverageControl::ClairvoyantCVT

**Centralized CVT (C-CVT):** The C-CVT is also a centralized algorithm based on CVT.
Similar to the clairvoyant algorithm, it knows the positions of all the robots and computes the exact Voronoi partition.
However, unlike the clairvoyant algorithm, the C-CVT can access a limited IDF.
It operates on the cumulative knowledge of the IDF of all the robots, sensed up to the current time step.
Thus, the amount of information available to the C-CVT is dependent on the sensor radius of the robots and the trajectory taken by the robots.  
See: CoverageControl::CentralizedCVT

**Decentralized CVT (D-CVT):** The D-CVT is the decentralized version of the C-CVT algorithm.
Here, each robot uses the positions of neighboring robots, i.e., the robots within its communication range, to compute the Voronoi partition.
Furthermore, each robot has restricted access to the IDF sensed by itself up to the current time.
Thus, the D-CVT algorithm uses only the local information available to each robot to take control actions.  
See: CoverageControl::DecentralizedCVT

As one can expect from the amount of information available to each algorithm, the clairvoyant algorithm is the best-performing algorithm, followed by the C-CVT and the D-CVT algorithms.
The clairvoyant algorithm is used to generate the dataset for training, and the C-CVT and D-CVT baseline algorithms are used to evaluate the performance of the LPAC architecture.

---
## References
S. Lloyd, "Least squares quantization in PCM," IEEE Transactions on Information Theory, vol. 28, no. 2, pp. 129–137, 1982.

J. Cortés, S. Martı̀nez, T. Karataş, and F. Bullo, "Coverage control for mobile sensing networks," IEEE Transactions on Robotics and Automation, vol. 20, no. 2, pp. 243–255, 2004.

M. Karavelas, "2D Voronoi diagram adaptor," in CGAL User and Reference Manual, 5th ed. CGAL Editorial Board, 2023. [Online]. Available: https://doc.cgal.org/5.6/Manual/packages.html#PkgVoronoiDiagram2

M. de Berg, O. Cheong, M. van Kreveld, and M. Overmars, Computational Geometry: Algorithms and Applications, 3rd ed. Berlin: Springer-Verlag, 2008.
