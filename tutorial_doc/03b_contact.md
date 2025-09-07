# Contact modelling
Quick intro to contaxct modelling and its important, also where is it important?

The rigid approximation of contact leads to fundamental problems in the mathematics and ill-conditioned numerics in software. Real physical systems are compliant, even if stiff.
Numerical techniques to deal with ill conditioning essentially boil down to introducing compliance at some level deep within the solver.

It is for these reasons that Drake embrace compliance, to provide accurate modeling of the physics of contact but also as a means to mitigate numerical problems associated with rigid approximations of contact. Within this framework, rigid contact can be approximated with very stiff compliance. As shown in [Castro et al., 2023](https://arxiv.org/abs/2312.03908) this technique is very effective even at very high stiffness values, beyond what it is really needed to model hard materials such as steel. In this way the modeling of contact is transparent to users, with physical parameters to control compliance and no need for hidden parameters in our solvers.

In this tutorial we will cover:

- Contact models in drake

## Compliant contact models
https://drake.mit.edu/doxygen_cxx/group__compliant__contact.html

### Point contact
The compliant point contact model only reports a single contact between bodies. More particularly, the contact is characterized by a single point. In some cases (e.g., a steel ball on a table surface), that is perfectly fine. For objects that contact across a large surface area (such as a box on the table), this characterization has two negative implications:

- the contact point will not be guaranteed to be at the center of pressure, possibly inducing unrealistic torque, and
- not be temporally coherent. This will lead to instability artifacts which can only be addressed through smaller time steps.

Both of these issues can be addressed by changing the geometry that represent the body's contact surface. For some shapes (e.g., boxes), we can introduce two sets of collision elements: discrete "points" at the corners, and a box capturing the volume (see block_for_pick_and_place.urdf and inclined_plane_with_body.cc in Drake's examples). With this strategy, the contact "points" are actually small-radius spheres. The volume-capturing box should actually be inset from those spheres such that when the box is lying on a plane (such that the logical contact manifold would be a face), only the contact points make contact, providing reliable points of contact. However, for arbitrary configurations contact with the box will provide more general contact.


### Hydroelastic Contact
Hydroelastic contact was created to address some of the short-comings in point contact. In fact, one might argue that many of the strategies used to mitigate the shortcomings of point contact (such as using lots of points) push it closer and closer to hydroelastic contact.

## Accessing contact forces.


-------------------------------------------------------
-------------------------------------------------------
-------------------------------------------------------


================
<!-- # Simulation of Multibody Systems with Frictional Contact
The `MultibodyPlant` offers two different modalities to model mechanical systems in time. These are:

- Discrete Models for Simulation (preferred for robustness and speed in most cases as advised by Drake)
- Continuous Models for Simulation

**Note:** As of May 2024, for multibody problems involving contact with friction, discrete solvers in Drake perform significantly better than error-controlled integration methods. 

 In the following sections we'll only provide a very limited distinction between a continuous and a discrete system. A complete discussion of the subject, including the modeling of hybrid systems is beyond the scope of this section and thus interested readers are referred to the documentation for Simulator. 

In this tutorial, we'll explore:
- Simulation and integration schemes for continuous systems.
- Contact solvers and mechanics for discrete systems.

## Simulation of continuous models
When the time_step is set to zero during MultibodyPlant construction, the system is modeled as continuous. 
```Python

```
This means it follows the dynamics of the form:
$$\dot{x} = f(t,x,u)$$

where $x$ is the plant's state, $t$ is time, and $u$ is the set of externally applied inputs (like actuation or forces). Continuous models allow you to use Drake's integrators to advance the system forward in time.

### Types of integrators
Integrators can be broadly categorized as one of:
- Explicit/Implicit integrators.
- Fixed-step/error-controlled integrators.

Fixed-step integrators often run faster, but they may fail to capture critical transitions, such as slip-stick behavior, when stiction tolerances are very small. On the other hand, error-controlled integrators, like `RungeKutta3Integrator`, adjust the time step to maintain stability and accuracy at the cost of simulation speed.

Implicit integrators are promising for handling stiff systems with larger time steps, but our experience with Drake's `ImplicitEulerIntegrator` has not demonstrated clear advantages for multibody systems using Stribeck approximations.

### Choosing an integrator
You can select an integrator for the simulation as shown below:
```Python
```

### Adjusting solver parameters
To fine-tune solver parameters for better performance, you can adjust various integrator settings:
```Python
```

Note that the integrators are designed also to be used standalone (without a Simulator driving them), examples can be found [here](). 

## Simulation of discerete Models
For multibody systems with contact and friction, discrete modeling is preferred due to its computational efficiency and robustness. Here, the system advances using fixed-length time steps, defined during `MultibodyPlant` construction:

```Python
```
Drake provides two primary solvers for discrete contact:

- **TAMSI Solver**, which formulates non-linear compliant contact with regularized friction using a Newton-Raphson solver with a custom "transition aware" line search, see [Castro et al., 2019].
- **Semi-Analytic Primal (“SAP”) contact solver**, a convex formulation originally developed in [[Castro et al., 2022]](https://arxiv.org/abs/2110.10107) as an extension to the work in [[Anitescu, 2006]](https://link.springer.com/article/10.1007/s10107-005-0590-7) and [[Todorov, 2014]](https://ieeexplore.ieee.org/document/6907751) to resolve physical compliance, providing a performant implementation in primal coordinates.

To select a different contact approximation, you can use:
```Python
plant.set_discrete_contact_solver(DiscreteContactApproximation.kSAP)
```

SAP's convex formulation makes it the preferred solver for frictional contact in many scenarios. It also supports more advanced models like Hunt-Crossley for compliant contact [Hunt and Crossley 1975], making it suitable for accurately capturing physical interactions.

**Note:** In discrete models, only dynamic friction is considered, as static friction cannot be fully resolved due to the velocity-level formulation.

<!-- 
## Simulation of Hybrid Models
A Drake system is a continuous/discrete/hybrid dynamic system where the continuous part is a DAE, that is, it is expected to consist of a set of differential equations and bilateral algebraic constraints. The set of active constraints may change as a result of particular events, such as contact.

The continuous parts of the trajectory are advanced using a numerical integrator. Different integrators have different properties; you can choose the one that is most appropriate for your application or use the default which is adequate for most systems.

## Numerical approximations of dry friction
Drake uses Coulomb's law of friction along with the maximum dissipation principle (MDP) to model friction in both continuous and discrete systems. These physical laws govern how friction forces are applied in response to contact, whether modeled as forces or stresses.

Coulomb's law of friction can be stated in terms of forces for point contact as:

 $$  ‖f_t‖ \le \mu_s f_n $$
where $\mu_s$ is the "static" coefficient of friction, and $f_t$ and $f_n$ are the tangential (friction) and normal components of the contact force, respectively.

For hydroelastic contact the law is defined in terms of stresses:

$$ ‖T_t‖ \le \mu_s p_n, $$
where $T_t$ and $p_n$ are the tangential (friction) and normal stresses, respectively.

The friction force fₜ (or stress Tₜ) is perpendicular to the contact normal. It's direction within a plane perpendicular to the contact normal, is determined by MDP. The MDP states that the direction of the friction force is such that it maximizes the amount of dissipation due to friction. Therefore, for a sliding contact with a non-zero slip velocity vector vₜ, the friction force directly opposes vₜ and its magnitude is μₖfₙ (or μₖpₙ), where μₖ is the "dynamic" (or kinetic) coefficient of friction, with μₖ ≤ μₛ. This can be written as:

 
 $$ fₜ = -μₖ vₜ/‖vₜ‖ fₙ. $$ 
 ---


Drake can model multibody systems as being continuous or discrete. Each of these represent different numerical approximations of the same underlying physics.

In a nutshell, continuous models formulate the multibody physics at the acceleration level and use a regularized model of Coulomb friction. This model is a continuous function of state that can be integrated in time using standard error-controlled numerical methods.

Discrete models on the other hand, formulate the multibody physics at the velocity level, and incorporate friction as constraints. Now, these constraints can be regularized similarly to continuous models, to improve numerics and robustness. Drake's discrete non-convex model with regularized friction is presented in [Castro et al., 2019], and novel convex approximations are presented [Castro et al., 2022] and [Castro et al., 2023].
-->
