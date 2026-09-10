# Robolions - 1261's 2026 FRC Codebase

## Structure Overview
We utilize a unique implementation of State-machines for purely teleoperated and use a strict command system for autonomous.
+ A somewhat standard implementation of Choreo in [AutoSubsystem.java](src/main/java/frc/robot/lib/auto/AutoSubsystem.java) with all custom commands created in [AutoCommands.java](src/main/java/frc/robot/lib/auto/AutoCommands.java).
+ All statemachine skeleton located at [statemachine](src/main/java/frc/robot/lib/statemachine), with [Statemachines](https://github.com/RoboLions/frc-2026/blob/main/src/main/java/frc/robot/lib/statemachine/StateMachine.java) containing [States](https://github.com/RoboLions/frc-2026/blob/main/src/main/java/frc/robot/lib/statemachine/State.java), governed by [Transitions](https://github.com/RoboLions/frc-2026/blob/main/src/main/java/frc/robot/lib/statemachine/Transition.java).

hello, i'm sun