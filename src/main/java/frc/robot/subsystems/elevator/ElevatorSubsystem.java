package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConfig;

public class ElevatorSubsystem extends SubsystemBase
{
    private final SparkMax elevator1 = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
    private final SparkMax elevator2 = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);
    private final SparkMax endEffectorTilt = new SparkMax(CANConfig.END_EFFECTOR_TILT, MotorType.kBrushless);

    private SparkMaxConfig elevator1Config = new SparkMaxConfig();
    private SparkMaxConfig elevator2Config = new SparkMaxConfig();

    private double startingHeight = 15.25;
    private double endingHeight = 59.25;
    private double startingEncoder = 0;
    private double endingEncoder = 110;

    private double inchesPerEncoder = (endingHeight - startingHeight)/(endingEncoder - startingEncoder);

    private PIDController elevatorPID = new PIDController(.5,0,0);
    
    public ElevatorSubsystem()
    {
        elevator1Config.inverted(true);
        elevator2Config.inverted(false);
        elevator1Config.idleMode(IdleMode.kBrake);
        elevator2Config.idleMode(IdleMode.kBrake);
        elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPID.setTolerance(1);
    }

    public void goToHeight(double height){
        height = height - 15.25; //15.25 is starting height
        double voltage = elevatorPID.calculate(this.getHeight(), height);
        if(Math.abs(voltage) > 3){
            voltage = (voltage)/Math.abs(voltage)*3;
        }
        SmartDashboard.putNumber("elevator PID Voltage", voltage);
        elevator1.setVoltage(voltage);
        elevator2.setVoltage(voltage);

    }
    public double getHeight(){
        return elevator1.getEncoder().getPosition()*inchesPerEncoder;
    }


    public void setSpeedElevator1(double speed) 
    {
        elevator1.set(speed);
    }
    
    public void setSpeedElevator2(double speed) 
    {
        elevator2.set(speed);
    }

    public void setSpeedEndEffectorTilt(double speed){
        endEffectorTilt.set(speed);
    }

    public void setSpeed(double speed){
        elevator1.set(speed);
        elevator2.set(speed);
    }

    //elevator positions
    

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Motor1 Position", elevator1.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Motor2 Position", elevator2.getEncoder().getPosition());
    }
}