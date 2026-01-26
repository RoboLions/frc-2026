// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachines.scoring;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.swerve.Swerve;

/** Add your docs here. */
public class CycleState extends State {

  Translation2d TURRET_TARGET = new Translation2d();

  @Override
  public void build() {
    addTransition(
        new Transition(
            () -> {
              return RobotMap.driverController.getBButton();
            },
            ScoringStateMachine.idleState));
  }

  @Override
  public void init(State prevState) {

  }

  @Override
  public void execute() {
        double xdiff = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 4 - Swerve.getPose().getX() : Swerve.getPose().getX() - 12.5;
        double ydiff = 4 - Swerve.getPose().getY();

        if (xdiff >= 0) {
            TURRET_TARGET = Constants.Hood.HUB_POSE;
        } else if (ydiff > 0) {
            TURRET_TARGET = Constants.Hood.TOP_PASS;
        } else {
            TURRET_TARGET = Constants.Hood.BOT_PASS;
        }
        
        Turret.turretTrack();
  }

  @Override
  public void exit(State nextState) {

  }
}