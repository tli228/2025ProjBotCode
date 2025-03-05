package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConfig;

public class ElevatorSubsystem extends SubsystemBase
{
    private final SparkMax elevator1;
    private final SparkMax elevator2;

    public ElevatorSubsystem()
    {
        elevator1 = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
        elevator2 = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);
    }

    public void setSpeedElevator1(double speed) 
    {
        elevator1.set(speed);
    }
    
    public void setSpeedElevator2(double speed) 
    {
        elevator2.set(speed);
    }

    @Override
    public void periodic()
    {
        
    }
}