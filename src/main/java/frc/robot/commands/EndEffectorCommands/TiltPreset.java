// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TiltPreset extends Command {
  private EndEffectorSubsystem endEffector; 
  private double tilt;  

  /** Creates a new TiltPreset. */
  public TiltPreset(double tilt, EndEffectorSubsystem endEffector) {
    this.tilt = tilt; 
    this.endEffector = endEffector; 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setSpeedEndEffectorTilt(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.goToTilt(tilt);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setSpeedEndEffectorTilt(0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endEffector.atSetPoint();
  }
}
