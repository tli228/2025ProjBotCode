// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.TestSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;




import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.elevator.*;
import frc.robot.subsystems.endeffector.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.commands.ClimberCommands.*;
import frc.robot.commands.ElevatorCommands.*;
import frc.robot.commands.EndEffectorCommands.*;
import frc.robot.commands.SwervedriveCommands.auto.*;
import frc.robot.commands.SwervedriveCommands.drivebase.*;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.drive.DriveSubsystem;


//https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html

 /* This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  public static PowerDistribution pdh;

  public static SwerveSubsystem driveTrain;
  public static ClimberSubsystem climber;
  public static ElevatorSubsystem elevator;
  public static EndEffectorSubsystem endEffector;

  public static CommandXboxController driverController;
  public static CommandXboxController operatorController;

  private final SendableChooser<Command> autoChooser;


  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  //adding a operator controller and trying to get it to work
  final       CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem();
  private final TestSubsystem         testmotor = new TestSubsystem();
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    pdh = new PowerDistribution();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    
    driveTrain = new SwerveSubsystem();
    climber = new ClimberSubsystem();
    elevator = new ElevatorSubsystem();
    endEffector = new EndEffectorSubsystem();

    driverController = new CommandXboxController(OperatorConstants.DRIVER_PORT);
    operatorController = new CommandXboxController(OperatorConstants.OPERATOR_PORT);

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("NOTHING!!!", new InstantCommand());
 
    driveTrain.setDefaultCommand(new RunCommand(
      //left joystick controls translation
      //right joystick controls rotation of the robot
      () -> driveTrain.driveCommand(-driverController.getLeftY(), driverController.getLeftX(), driverController.getRightX()),
      driveTrain));


    climber.setDefaultCommand(new RunCommand(() -> {
    }));

    elevator.setDefaultCommand(new RunCommand(() -> {
      elevator.setSpeed(-operatorController.getLeftY()*.1); //Multiply by .1 for testing
    }));

    endEffector.setDefaultCommand(new RunCommand(() -> {
      endEffector.setSpeedEndEffectorTilt(-operatorController.getRightY()*.1);  //We are using this to test, the .1 is to make it go slow
    }));


  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
     
    //To do: Set Buttons, Set Speeds, Verify Directions

    // Operator Controls
    operatorController.rightTrigger().onChange(new ShootCommand(-.1));
    operatorController.leftTrigger().onChange(new ShootCommand(.1));

    operatorController.rightBumper().onChange(new IntakeCommand(-.1));
    operatorController.leftBumper().onChange(new IntakeCommand(.1));


 
    // Driver Controls
    driverController.y().onTrue(new InstantCommand(() -> {climber.LatchServo();}));
    driverController.b().onTrue(new InstantCommand(() -> {climber.UnlatchServo();}));

    driverController.rightTrigger().onChange(new ExtendWinchCommand(-.1));
    driverController.leftTrigger().onChange(new ExtendWinchCommand(.1));

    driverController.rightTrigger().onChange(new GrabCageCommand(.1));
    driverController.leftTrigger().onChange(new GrabCageCommand(-1.));
     
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand()
  // {
  //   // An example command will be run in autonomous
  //   return drivebase.getAutonomousCommand("New Auto");
  // }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Testpath");
  }
  //code stolen from example from github
}

