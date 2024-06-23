{{< figure src="/images/MSL_PS.gif" >}}

Evolving Soft Robots to walk via Genetic Programming is a current project exploring the process of synthetic evolution with the hopes of finding innovative locomotion techniques. 

I created a custom physics simulator using C++, Swift, and Apple's Metal Shading Language to perform graphics rendering and calculations. This is a current project however, and I am currently in the process of transitioning the genetic program rendering from Matplotlib to Apple Metal GPU. The simulator uses a spring-mass system to simulate the motion of soft-bodies. Each mass object is connected to a variety of other masses through springs. The springs simulate the forces applied to the mass using Hooke's Law, which allows for simple calculation of compressive and tensile forces.

{{< figure src="/images/cube_bounce.gif" >}}

Additional forces such as gravity, friction, and dampening forces are applied to the simulation to uphold kinetic laws and energy consumption. Thus, the goal for an evolved soft robot is to be efficient and effective at combating these force restrictions.

There are two types of "materials" the genetic program can use when creating the morphology of the soft robot: bone and muscle. The muscle structure is a soft body cube that has the ability to expand and contract based on an actuating function that determines the lengths of the springs present, allowing the muscle to move based on time. The bone structure is therefore a foundation piece that does not have this ability to expand and contract, instead serving as a base.

{{< figure src="/images/cube_breathe.gif" >}}

The evolution process uses Genetic Programming, a variant of Evolutionary Algorithms that use the pruning and expansion of tree data structures to create solutions to a problem. This provides the possibility to create novel designs to solve problems that maximize "fitness" or the criteria in which you are testing. In this case, this criteria is the movement capability in the simulated environment.

This is an active project, last update Dec. 20, 2023.

- Updates being made:
  - Adding ambient lighting for better graphics
  - Incorporating Metal GPU into calculations in addition to its use in graphics rendering
  - Finishing transition of Genetic Program into Metal environment from C++ and Python
  - Improving computational ability to support more complex robots

Here are my past renditions of robotic movement using Genetic Programming, this will be updated when rendering in Metal is complete.

{{< figure src="/images/robot_movement.gif" >}}

{{< figure src="/images/robot_bounce.gif" >}}
