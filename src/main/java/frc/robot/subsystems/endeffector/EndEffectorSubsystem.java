package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConfig;

public class EndEffectorSubsystem extends SubsystemBase
{
    private final SparkMax endEffectorMotor;
    private final SparkMax endEffectorTilt;
    private final SparkMax endEffectorIntake;

    public EndEffectorSubsystem()
    {
        endEffectorMotor = new SparkMax(CANConfig.END_EFFECTOR_MOTOR, MotorType.kBrushless);
        endEffectorTilt = new SparkMax(CANConfig.END_EFFECTOR_TILT, MotorType.kBrushless);
        endEffectorIntake = new SparkMax(CANConfig.END_EFFECTOR_INTAKE, MotorType.kBrushless);
    }
    

    public void setSpeedEndEffectorMotor(double speed) 
    {
        endEffectorMotor.set(speed);
    }

    public void setSpeedEndEffectorTilt(double speed) 
    {
        endEffectorTilt.set(speed);
    }
    
    public void setSpeedEndEffectorIntake(double speed){
        endEffectorIntake.set(speed);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("End Effector Encoder", endEffectorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("End Effector Tilt", endEffectorTilt.getEncoder().getPosition());
        SmartDashboard.putNumber("End Effector Tilt Absolute Encoder", endEffectorTilt.getAlternateEncoder().getPosition());
        SmartDashboard.putNumber("End Effector Intake", endEffectorIntake.getEncoder().getPosition());
    }
}