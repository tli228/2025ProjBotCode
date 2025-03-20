package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CANConfig;

public class ElevatorSubsystem extends SubsystemBase
{
    public DigitalInput elevatorLowerSwitch;

    public final SparkMax elevator1 = new SparkMax(CANConfig.ELEVATOR_LEFT, MotorType.kBrushless);
    public final SparkMax elevator2 = new SparkMax(CANConfig.ELEVATOR_RIGHT, MotorType.kBrushless);

    private SparkMaxConfig elevator1Config = new SparkMaxConfig();
    private SparkMaxConfig elevator2Config = new SparkMaxConfig();

    private double startingHeight = 15.25;
    private double endingHeight = 59.25;
    private double startingEncoder = 0;
    private double endingEncoder = 110;

    private double inchesPerEncoder = (endingHeight - startingHeight)/(endingEncoder - startingEncoder);

   private PIDController elevatorPID = new PIDController(.8,0,0);
    
    public ElevatorSubsystem()
    {
        this.elevatorLowerSwitch = new DigitalInput(2);

        elevator1Config.inverted(true);
        elevator2Config.inverted(false);
        elevator1Config.idleMode(IdleMode.kBrake);
        elevator2Config.idleMode(IdleMode.kBrake);
        elevator1.configure(elevator1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevator2.configure(elevator2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorPID.setTolerance(1);
    }

    public void goToHeight(double height){
        double voltage = elevatorPID.calculate(elevator1.getEncoder().getPosition(), height);
        if(Math.abs(voltage) > 12){
            voltage = (voltage)/Math.abs(voltage)*12;
        }
        SmartDashboard.putNumber("elevator PID Voltage", voltage);
        elevator1.setVoltage(voltage);
        elevator2.setVoltage(voltage);

    }
    public void resetEncoder(){
        elevator1.getEncoder().setPosition(0);
        elevator2.getEncoder().setPosition(0);
    }
    public double getHeight(){
        return elevator1.getEncoder().getPosition()*inchesPerEncoder;
    }
    public void setVoltage(double voltage) {
        elevator1.setVoltage(voltage);
        elevator2.setVoltage(voltage);
    }

    public boolean atSetpoint(){
        return elevatorPID.atSetpoint();
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


    public Boolean lowerLimitReached() {
        if(elevatorLowerSwitch.get())
            return false;
            else{
            return true;
            }
    }
    

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Elevator Motor1 Position", elevator1.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Motor2 Position", elevator2.getEncoder().getPosition());
        SmartDashboard.putBoolean("Elevator Limit Switch", lowerLimitReached());
    }
}