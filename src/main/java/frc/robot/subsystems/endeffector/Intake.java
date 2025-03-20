// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConfig;

public class Intake extends SubsystemBase {
  private final SparkMax endEffectorIntake = new SparkMax(CANConfig.END_EFFECTOR_INTAKE, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intake(double speed) {
    endEffectorIntake.set(speed);
  }
}
