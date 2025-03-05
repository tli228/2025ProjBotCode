package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConfig;

public class EndEffectorSubsystem extends SubsystemBase
{
    private final SparkMax endEffectorAlgae;
    private final SparkMax endEffectorFinger;
    private final SparkMax endEffectorMotor;
    private final SparkMax endEffectorTilt;

    public EndEffectorSubsystem()
    {
        endEffectorAlgae = new SparkMax(CANConfig.END_EFFECTOR_ALGAE, MotorType.kBrushless);
        endEffectorFinger = new SparkMax(CANConfig.END_EFFECTOR_FINGER, MotorType.kBrushless);
        endEffectorMotor = new SparkMax(CANConfig.END_EFFECTOR_MOTOR, MotorType.kBrushless);
        endEffectorTilt = new SparkMax(CANConfig.END_EFFECTOR_TILT, MotorType.kBrushless);
    }

    public void setSpeedEndEffectorAlgae(double speed) 
    {
        endEffectorAlgae.set(speed);
    }
    
    public void setSpeedEndEffectorFinger(double speed) 
    {
        endEffectorFinger.set(speed);
    }

    public void setSpeedEndEffectorMotor(double speed) 
    {
        endEffectorMotor.set(speed);
    }

    public void setSpeedEndEffectorTilt(double speed) 
    {
        endEffectorTilt.set(speed);
    }

    @Override
    public void periodic()
    {
        
    }
}