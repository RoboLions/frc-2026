package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.auto.AutoSubsystem;
import frc.robot.subsystems.auto.commands.AlignCommand;
import frc.robot.subsystems.auto.commands.ArmDown;
import frc.robot.subsystems.auto.commands.AutonTestCommand;
import frc.robot.subsystems.auto.commands.FShootCommand;
import frc.robot.subsystems.auto.commands.IdleCommand;
import frc.robot.subsystems.auto.commands.IntakeCommand;
import frc.robot.subsystems.auto.commands.LLenable;
import frc.robot.subsystems.auto.commands.LongRangeAlign;
import frc.robot.subsystems.auto.commands.ShootCommand;
import frc.robot.subsystems.auto.commands.SpeakerAlign;
import frc.robot.subsystems.auto.commands.SpinAllCommand;
import frc.robot.subsystems.auto.commands.SpinThroughCommand;
import frc.robot.subsystems.auto.commands.SpinupCommand;
import frc.robot.subsystems.auto.commands.StopCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Climber;
import frc.robot.subsystems.interfaces.Feeder;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Limelight;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Swerve;
import frc.robot.subsystems.statemachines.Climb.ClimbStateMachine;
import frc.robot.subsystems.statemachines.drive.DrivetrainStateMachine;
import frc.robot.subsystems.statemachines.scoring.MasterStateMachine;

public class RobotMap {

  private static SendableChooser<Command> autoChooser;
  /* state machine instances */
  public static final DrivetrainStateMachine drivetrainStateMachine = new DrivetrainStateMachine();
  public static final MasterStateMachine masterStateMachine = new MasterStateMachine();
  public static final ClimbStateMachine climbStateMachine = new ClimbStateMachine();

  /* Xbox controllers */
  public static final XboxController manipulatorController = new XboxController(1);
  public static final XboxController driverController = new XboxController(0);

  /* Auto objects */
  public static final AutoSubsystem autoSubsystem = new AutoSubsystem();

  public static final AutonTestCommand autonTestCommand = new AutonTestCommand();
  public static final IntakeCommand intakeCommand = new IntakeCommand();
  public static final SpinupCommand spinupCommand = new SpinupCommand();
  public static final ShootCommand shootCommand = new ShootCommand();
  public static final IdleCommand idleCommand = new IdleCommand();
  public static final AlignCommand alignCommand = new AlignCommand();
  public static final SpinAllCommand spinAllCommand = new SpinAllCommand();
  public static final SpinThroughCommand spinThroughCommand = new SpinThroughCommand();
  public static final FShootCommand fShootCommand = new FShootCommand();
  public static final StopCommand stopCommand = new StopCommand();
  public static final ArmDown armDown = new ArmDown();
  public static final LongRangeAlign longRangeAlign = new LongRangeAlign();
  public static final LLenable llEnable = new LLenable();
  public static final SpeakerAlign speakerAlign = new SpeakerAlign();

  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void init() {
    Swerve.init();
    Intake.init();
    Shooter.init();
    Feeder.init();
    Arm.init();
    Limelight.init();
    Climber.init();

    NamedCommands.registerCommand("Intake", intakeCommand);
    NamedCommands.registerCommand("SpinUp", spinupCommand);
    NamedCommands.registerCommand("Shoot", shootCommand);
    NamedCommands.registerCommand("Idle", idleCommand);
    NamedCommands.registerCommand("Align", alignCommand);
    NamedCommands.registerCommand("SpinAll", spinAllCommand);
    NamedCommands.registerCommand("SpinThrough", spinThroughCommand);
    NamedCommands.registerCommand("FShoot", fShootCommand);
    NamedCommands.registerCommand("Stop", stopCommand);
    NamedCommands.registerCommand("ArmDown", armDown);
    NamedCommands.registerCommand("LRALign", longRangeAlign);
    NamedCommands.registerCommand("LLenable", llEnable);
    NamedCommands.registerCommand("SpeakerAlign", speakerAlign);

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    drivetrainStateMachine.init();
    masterStateMachine.init();
    climbStateMachine.init();
  }
}
// initiate bomb sequence
