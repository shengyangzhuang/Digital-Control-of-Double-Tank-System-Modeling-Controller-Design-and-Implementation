# Digital-Control-of-Double-Tank-System-Modeling-Controller-Design-and-Implementation
The objective of this project is to understand the basics of digital control including modeling, controller design, and implementation.

In the first part, we mainly construct the mathematicle models of the coupled tank. Then we perform the continuous control design by calculating the transfer function, using pole-placement to calculate PID parameters and using Simulink to simulate the performances under different setting parameters.

In the second part, we mainly design the digital controller, consisting of process, controller, sampler, ZOH, A/D and D/A converters. For the sake of simplicity, we here negelect the A/D and D/A converters. By changing the sampling time of ZOH, we compare the differences in control performance compared to continuous case. Then we discretize the controller into state space form by using c2d function and compare the simulation results with previous case. We find that when discretizing the continuous controller, if the sampling time is too large, the step response became oscillating. Then we determine the maximum sampling time without affecting control performance by experiments.

In the third part, we design our discrete controller based on previous work. First we determine the continuous-time linearized system and identify the matrices and change them to discrete time system. Then we analyze the properties of our discrete time system, such as observability and reachability. Next we define an augumented system which represents the colsed-loop dynamics of the water tank system. Due to separation principle, we design the parameters independently when palcing the poles of the colsed-loop system. By simulink, we verify that our discrete designed controller can performed better than discretized controller with the same sampling time.

Finally, we investigate the effect of quantization.
