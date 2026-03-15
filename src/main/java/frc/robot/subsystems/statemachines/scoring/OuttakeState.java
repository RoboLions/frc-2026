// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Intake;

/** Add your docs here. */
public class OuttakeState extends State {

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getBButton();
            },
            ScoringStateMachine.idleState));
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getRightBumper();
            },
            ScoringStateMachine.intakeState));
  }


  @Override
  public void init(State prevState) {
    Intake.intakeDown();
    Intake.outtake();
    RobotMap.driverController.setRumble(GenericHID.RumbleType.kLeftRumble, 0.05);
  }

  @Override
  public void execute() {

  }

  @Override
  public void exit(State nextState) {

  }
}