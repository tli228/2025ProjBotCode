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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  //Define Subsystems
  public static SwerveSubsystem driveTrain = new SwerveSubsystem();
  public static ClimberSubsystem climber = new ClimberSubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  public static Intake intake = new Intake();

  //Define Controllers
  public static CommandXboxController driverController = new CommandXboxController(0);
  public static CommandXboxController operatorController = new CommandXboxController(1);

  //Auto Chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveTrain.getSwerveDrive(),
                                                                () -> driverController.getLeftY() * -1,
                                                                () -> driverController.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverController::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    autoChooser = AutoBuilder.buildAutoChooser("Pass The Line Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);

    driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);


    climber.setDefaultCommand(new RunCommand(() -> {
         // Driver Controls
    driverController.y().onTrue(new InstantCommand(() -> {climber.LatchServo();}));
    driverController.b().onTrue(new InstantCommand(() -> {climber.UnlatchServo();}));

    
   if(driverController.rightTrigger().getAsBoolean()){
    if(climber.lowerLimitReachedWinch()){
      climber.setSpeed(0);
      climber.resetEncoder();
    }
    climber.setSpeed(3);
   }
   else if(driverController.leftTrigger().getAsBoolean()){
    climber.setSpeed(-3);
   }
   else{
    climber.setSpeed(0);
   }
    driverController.rightBumper().whileTrue(new GrabCageCommand(1.0));
    driverController.leftBumper().whileTrue(new GrabCageCommand(-1.0));

    }, climber));


    intake.setDefaultCommand(new RunCommand(() -> {}, intake));

    elevator.setDefaultCommand(new RunCommand(() -> {
      //if the lower limit's been reached don't allow them to go down but allow them to go up. 
      if(elevator.lowerLimitReached() == true && -operatorController.getLeftY() <= 0)  {
         //Multiply by .1 for testing
        elevator.resetEncoder();
        elevator.setSpeed(0);
      }
      else{
        // if(endEffector.endEffectorTilt.getEncoder().getPosition() > 5.6)
        // {
           elevator.setSpeed(-operatorController.getLeftY()*3);
        // }
        // else{
        //   endEffector.goToTilt(5.7);
        // }
      }
    
    }, elevator));

    endEffector.setDefaultCommand(new RunCommand(() -> {
      if(elevator.lowerLimitReached() == true /*&& -operatorController.getLeftY() <= 0*/) {        
        //if((endEffector.endEffectorTilt.getEncoder().getPosition() <= -.8)||(endEffector.endEffectorTilt.getEncoder().getPosition() >= 0 )){
          //if(endEffector.atSetPoint() == false){
            endEffector.goToTilt(-.4);
          //}else{
           // endEffector.setSpeedEndEffectorTilt(0);
          //}
        //}
      }else if((elevator.elevator1.getEncoder().getPosition() <= 155)) {
        endEffector.goToTilt(5.7);
      }else{
        endEffector.setSpeedEndEffectorTilt(-operatorController.getRightY()*.1); //We are using this to test, the .1 is to make it go slow //this is the value to make sure the end effector clears the funnel
      }
    }, endEffector));
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
    Command heightL1PresetCommand = new ElevatorPreset(Constants.ElevatorConstants.L1Height,elevator);
    Command heightL2PresetCommand = new ElevatorPreset(Constants.ElevatorConstants.L2Height,elevator);
    Command heightL3PresetCommand = new ElevatorPreset(Constants.ElevatorConstants.L3Height,elevator);
    Command heightL4PresetCommand = new ElevatorPreset(Constants.ElevatorConstants.L4Height,elevator);
    //Command heightA1PresetCommand = new ElevatorPreset(Constants.ElevatorConstants.A1Height,elevator);
    //Command heightA2PresetCommand = new ElevatorPreset(Constants.ElevatorConstants.A2Height,elevator);

    Command tiltL1PresetCommand = new TiltPreset(Constants.tiltConstants.tiltL1, endEffector);
    Command tiltL2PresetCommand = new TiltPreset(Constants.tiltConstants.tiltL2, endEffector);
    Command tiltL3PresetCommand = new TiltPreset(Constants.tiltConstants.tiltL3, endEffector);
    Command tiltL4PresetCommand = new TiltPreset(Constants.tiltConstants.tiltL4, endEffector);
    //Command tiltA1PresetCommand = new TiltPreset(Constants.tiltConstants.tiltA1, endEffector);
    //Command tiltA2PresetCommand = new TiltPreset(Constants.tiltConstants.tiltA2, endEffector);
    
    SequentialCommandGroup goToPresetL1 = new SequentialCommandGroup(heightL1PresetCommand, tiltL1PresetCommand);
    SequentialCommandGroup goToPresetL2 = new SequentialCommandGroup(heightL2PresetCommand, tiltL2PresetCommand);
    SequentialCommandGroup goToPresetL3 = new SequentialCommandGroup(heightL3PresetCommand, tiltL3PresetCommand);
    SequentialCommandGroup goToPresetL4 = new SequentialCommandGroup(heightL4PresetCommand, tiltL4PresetCommand);
    //SequentialCommandGroup goToPresetA1 = new SequentialCommandGroup(tiltA1PresetCommand, heightA1PresetCommand);
    //SequentialCommandGroup goToPresetA2 = new SequentialCommandGroup(tiltA2PresetCommand, heightA2PresetCommand);

    operatorController.rightTrigger().whileTrue(new ShootCommand(-2.5));
    operatorController.leftTrigger().whileTrue(new ShootCommand(2.5));

    operatorController.rightBumper().whileTrue(new IntakeCommand(-.2));
    operatorController.leftBumper().whileTrue(new IntakeCommand(.2));
    operatorController.x().whileTrue(goToPresetL1);
    operatorController.y().whileTrue(goToPresetL2);
    operatorController.b().whileTrue(goToPresetL3);
    operatorController.a().whileTrue(goToPresetL4);
    //operatorController.povDown().whileTrue(goToPresetA1);
    //operatorController.povUp().whileTrue(goToPresetA2);

 
 
     
  }
  public void setMotorBrake(boolean brake)
  {
    driveTrain.setMotorBrake(brake);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Pass the auto line for points
    return autoChooser.getSelected();
    // return driveTrain.getAutonomousCommand("Pass The Line Auto");
  }
}

