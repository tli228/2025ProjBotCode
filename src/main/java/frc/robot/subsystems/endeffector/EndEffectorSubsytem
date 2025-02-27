package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase
{
    private final SparkMax elevator1;
    private final SparkMax elevator2;

    public ElevatorSubsystem()
    {
        elevator1 = new SparkMax(ElevatorConstants.kElevator1CANId, MotorType.kBrushless);
        elevator2 = new SparkMax(ElevatorConstants.kElevator2CANId, MotorType.kBrushless);
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