## TeamCode Module

Under the TeamCode Module, all our code is stored (within the legacy folder). Within it, there are 5 packages: 

* a library for base operations: [lib](legacy/lib)
  * Control (Pure Pursuit, MPC, LQR, and other control systems)
  * Drivers (Motor Controllers)
  * Motion (Motion Profile Generators)
  * Utilities & Other Operations
* a folder containing opmodes: [programs](legacy/programs)
  * Autonomous/TeleOp Programs in the respective subfolders
* a package with constants and other sources: [src](legacy/src)
  * Robot Class
  * Constants
  * Game Time Tracker
* a module for storing the finite state machines: [states](legacy/states)
* a collection of the robot's system-specific code: [subsystems](legacy/subsystems)
  * Drive Subsystem
  * Inertal Measurement Unit Subsystem
  * Vuforia Vision Subsystem
  * TensorFlow Vision Subsystem

