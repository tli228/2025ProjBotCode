package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase
{
    private final SparkMax algaeRoller;
    private final SparkMax algaeTilt;

    public AlgaeIntakeSubsystem()
    {
        algaeRoller = new SparkMax(AlgaeIntakeConstants.kAlgaeRollerCANId, MotorType.kBrushless);
        algaeTilt = new SparkMax(AlgaeIntakeConstants.kAlgaeTiltCANId, MotorType.kBrushless);
    }

    public void setSpeedRoller(double speed) 
    {
        algaeRoller.set(speed);
    }
    
    public void setSpeedArmTilt(double speed) 
    {
        algaeTilt.set(speed);
    }

    @Override
    public void periodic()
    {
        
    }
}