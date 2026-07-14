# Pursuiter 
Robolions - 1261's easy to use Choreo Integrated take on a Pure Pursuit Controller.

## Structure Overview
This library is a mostly standard implementation of a Pure Pursuit Controller for Swerve Drive. Pure Pursuit controllers are generally best for FRC as they can dynamically react to obstructions or undesired events which obstruct the path. A Pure Pursuit controller utilizes a "look-ahead" point along the path. The robot drives towards this "look-ahead" point and updates this "look-ahead" as it progresses along the path. 

### PursuitAutoFactory
PursuitAutoFactory should be used to initialize Choreo's .traj files you have created in the Choreo App. In a brief overview the auto factory can:
- Link Commands to the Event Markers you create in the Choreo App with 'addEvent'. The distinction here being that these Event Markers are triggered based on path progression. Events cannot be triggered solely based on proximity, which means there is no need to ensure Event Markers do not occur at path overlaps.
- Flip paths on both the X and Y axies about a centerpoint using 'followPath'.
- Log the current path trajectory for aura points (enabled via 'PursuitProfile'). You are encouraged to tinker around with the logging of this library as you see fit.
- Accept a pose supplier, reset odometry function, and a 'PursuitProfile'. More info on the 'PursuitProfile' class below.
- Includes an SendableChooser if you wish to register autos with this factory.

### PursuitProfile
PursuitProfile is used to inform the PursuitAutoFactory of constraints and PIDs. You can:
- Set the path's endpoint tolerances (both distance and rotation).
- The Pure Pursuit controller's look-ahead distance to inform its next look-ahead point.
- PIDControllers to guide rotation and translational correction mid path.
- A seperate PIDController only used at the end of the path to help the path finish within tolerance. Profiled Pure Pursuit controllers can struggle with ending at the specified endpoint accurately because of profiled speeds at the end of the path being much lower than in the middle path. This PID Controller is meant to help with that.
- A toggle for the default logging included in the auto path.

## Quick Start
### 1. Define your Profile
'''java
PursuitProfile highPrecisionProfile = new PursuitProfile(
    0.05,                         // 5 cm translation tolerance
    2.0,                          // 2 degrees rotation tolerance
    Meters.of(0.5),               // 0.5 meter look-ahead distance
    new PIDController(0.5, 0, 0), // Mid-path translation PID
    new PIDController(3.0, 0, 0), // Endpoint anchoring PID
    new PIDController(1.5, 0, 0), // Heading PID
    true                          // Enable telemetry logging
);
'''
