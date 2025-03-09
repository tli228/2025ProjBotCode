package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConfig;

public class ElevatorSubsystem extends SubsystemBase
{
    private final SparkMax elevator1 = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
    private final SparkMax elevator2 = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);;

    private SparkMaxConfig elevator1Config = new SparkMaxConfig();
    private SparkMaxConfig elevator2Config = new SparkMaxConfig();
    
    public ElevatorSubsystem()
    {
        elevator1Config.inverted(true);
        elevator2Config.inverted(false);
        elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeedElevator1(double speed) 
    {
        elevator1.set(speed);
    }
    
    public void setSpeedElevator2(double speed) 
    {
        elevator2.set(speed);
    }

    public void setSpeed(double speed){
        elevator1.set(speed);
        elevator2.set(speed);
    }

    @Override
    public void periodic()
    {
        
    }
}