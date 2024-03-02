\page coverage-control-problem Problem Statement
\tableofcontents

## Introduction
Coverage control is the problem of navigating a robot swarm to collaboratively monitor features or a phenomenon of interest not known _a priori_.
The goal is to provide sensor coverage based on the importance of information at each point in the environment.
An importance density field (IDF) \f$\Phi:\mathcal{W} \mapsto {R}_{\geq 0}\f$ is defined over a 2-D environment \f$\mathcal W\subset  R^2\f$.
The IDF represents a non-negative measure of importance at each point in the environment.
With the state of a robot \f$i\f$ given by its position \f$\mathbf p_i\in \mathcal W\f$ in the environment, the control actions given by the velocity \f$\dot{\mathbf p}_i(t)\f$, and \f$\Delta t\f$ as the time step, we use the following model for the state evolution:
\f{equation}{
\mathbf p_i(t+\Delta t) = \mathbf p_i(t) + \dot{\mathbf p}_i(t) \Delta t.
\f}

## Cost Function
The cost function for the coverage control problem is defined as:
\f{equation}{
	\mathcal J\left(\mathbf X\right) = \int_{\mathbf q \in \mathcal W} \min_{i\in\mathcal V}  f(\|\mathbf p_i - \mathbf q\|) \Phi(\mathbf q)\,d\mathbf q.
	\label{eq:coverage_gen}
\f}
Here, \f$f\f$ is a non-decreasing function, and a common choice is \f$f(x) = x^2\f$ as it is a smooth function and is easy to optimize.


Assuming that no two robots can occupy the same point in the environment, the Voronoi partition can be used to assign each robot a distinct portion of the environment to cover.
The Voronoi partition \f$\mathcal P\f$ is defined as:
\f{eqnarray*}{
    \mathcal P &=& \{P_i \mid i \in \mathcal V\},\\
                  P_i &=& \{\mathbf q \in \mathcal W \mid \|\mathbf p_i - \mathbf q\| \le \|\mathbf p_j - \mathbf q\|, \forall j \in \mathcal V\}.
                  \label{eq:voronoi}
\f}
All points in \f$P_i\f$ are closer to robot \f$i\f$ than any other robot.
The cost function can now be expressed in terms of the Voronoi partition as:
\f{equation}{
	\mathcal J\left(\mathbf X\right) = \sum_{i=1}^N \int_{\mathbf q \in P_i} f(\|\mathbf p_i - \mathbf q\|) \Phi(\mathbf q)\,d\mathbf q.
	\label{eq:coverage_voronoi}
\f}
This new cost function is a sum of integrals over disjoint regions and is much easier to compute and optimize than the original function.
Furthermore, if the Voronoi partition is known, the cost function can be computed in a decentralized manner, as each robot only needs to compute the integral over its own region \f$P_i\f$.

---
## The Problem
We can now define the coverage control problem in the context of the navigation control problem:
Find a decentralized control policy \f$\Pi\f$ that minimizes the expected cost \f$\mathcal J(\mathbf X)\f$.
The policy \f$\Pi\f$ is defined over a space of all possible velocities, and each robot independently executes the same policy.

\htmlonly
<img class="center" style="width: 20vw; min-width: 330px; margin-left: auto; margin-right: auto;" src="coveragecontrol_global.png"/>
<figcaption>A near-optimal solution, along with Voronoi partition, to an instance of the coverage control problem with 32 robots and 32 features in a 1024 x 1024 environment.</figcaption>
\endhtmlonly

---

## Decentralized Coverage Control
The library aims at providing simulations and algorithms for the decentralized coverage control problem with the following restrictions:
1. The IDF \f$\Phi\f$ is static and is not known _a priori_.
2. At any time, each robot can make localized observations of the IDF within a sensor field of view (FoV).
3. Each robot can maintain its own localized observations of the IDF aggregated over time.
4. The robots can only communicate with other robots that are within a limited communication radius.

In such a setting, a coverage control algorithm needs to provide the following based on the state of robot \f$i\f$ and the information received from its neighbors \f$\mathcal N(i)\f$:
1. A function \f$\mathcal I\f$ that computes the information, in the form of messages, to be communicated by robot \f$i\f$ to its neighbors, and 
2. A common policy \f$\Pi\f$ that computes the control action \f$\mathbf u_i = \dot{\mathbf p}_i\f$ for any robot \f$i \in \mathcal V\f$.

Designing such decentralized algorithms is challenging and can be intractable for complex systems.
This motivates us to use a learning-based approach to design a decentralized coverage control algorithm.
The \ref lpac with GNN addresses the above challenges and provides a scalable and robust solution to the problem.
