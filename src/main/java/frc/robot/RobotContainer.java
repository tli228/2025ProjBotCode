package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import frc.robot.subsystems.vision.PoseEstimatorVision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.commands.ClimberCommands.ManualClimbCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.IntakeCommands.RunIntakeCommand;
import frc.robot.commands.OperatorCommands.AmpPosCommand;
import frc.robot.commands.OperatorCommands.EjectNoteCommand;
import frc.robot.commands.OperatorCommands.ResetShooterCommand;
import frc.robot.commands.OperatorCommands.ShootNoteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer 
{
  public static final PowerDistribution pdh = new PowerDistribution();
  private final Vision cam = new Vision(VisionConstants.kCameraName, VisionConstants.camTransform);

  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem driveTrain = new DriveSubsystem();
  private final PoseEstimatorVision poseEstimator = new PoseEstimatorVision(cam, driveTrain);
  private final ConveyorSubsystem conveyor = new ConveyorSubsystem();
  private final FlyWheelSubsystem flyWheel = new FlyWheelSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final TelescopeSubsystem telescope = new TelescopeSubsystem();
  private final TiltSubsystem tilt = new TiltSubsystem();

  private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final static CommandXboxController operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the trigger bindings
    configureBindings();

    NamedCommands.registerCommand("IntakeNote", new RunIntakeCommand(conveyor));
    NamedCommands.registerCommand("ShootNote", new ShootNoteCommand(conveyor, flyWheel));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("NOTHING!!!", new InstantCommand());
    SmartDashboard.putData(autoChooser);

    
    
    driveTrain.setDefaultCommand(new RunCommand(
      //left joystick controls translation
      //right joystick controls rotation of the robot
      () -> driveTrain.drive(
        -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriverDeadband), 
        -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriverDeadband), 
        -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriverDeadband), 
        true), 
      driveTrain));
    
    telescope.setDefaultCommand(new RunCommand(
      () -> telescope.setTelescopeMotor(-MathUtil.applyDeadband(operatorController.getRightY(), OIConstants.kDriverDeadband)*0.5), 
      telescope));
    
     
    tilt.setDefaultCommand(new RunCommand(
      ()-> tilt.setTiltMotor(-MathUtil.applyDeadband(operatorController.getLeftY(), OIConstants.kDriverDeadband)), 
      tilt));
  }

  private void configureBindings() 
  {
    operatorController.x().toggleOnTrue(new RunIntakeCommand(conveyor)); //intake

    operatorController.y().toggleOnTrue(new ShootNoteCommand(conveyor, flyWheel));

    //operatorController.a().toggleOnTrue(Commands.deadline(new FlywheelHoldCommand(flyWheel), new FeedShooterCommand(conveyor)));
    
    //amp setting
    operatorController.leftBumper().toggleOnTrue(new AmpPosCommand(telescope, tilt, conveyor, flyWheel));
    operatorController.leftBumper().toggleOnFalse(new ResetShooterCommand(telescope, tilt));

    //eject note
    operatorController.rightBumper().whileTrue(new EjectNoteCommand(conveyor, flyWheel));

    //climb controls
    operatorController.leftTrigger().whileTrue(new ManualClimbCommand(climber, -1.0));

    //up
    operatorController.rightTrigger().whileTrue(new ManualClimbCommand(climber, 1.0));
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    return autoChooser.getSelected();
  }
}
