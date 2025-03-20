package frc.robot.subsystems.endeffector;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.tiltConstants;

public class EndEffectorSubsystem extends SubsystemBase
{
    public final SparkMax endEffectorMotor;
    public final SparkMax endEffectorTilt;

    private PIDController tiltPIDController = new PIDController(.06, 0.0, 0.0);

    public EndEffectorSubsystem()
    {
        endEffectorMotor = new SparkMax(CANConfig.END_EFFECTOR_MOTOR, MotorType.kBrushless);
        endEffectorTilt = new SparkMax(CANConfig.END_EFFECTOR_TILT, MotorType.kBrushless);
        tiltPIDController.setTolerance(.5);
    }
    

    public void setSpeedEndEffectorMotor(double speed) 
    {
        endEffectorMotor.set(speed);
    }

    public void setSpeedEndEffectorTilt(double speed) 
    {
        endEffectorTilt.set(speed);
    }

    public void setPIDGains(double P, double I, double D){
        tiltPIDController.setPID(P, I, D); 
    }

    public void goToTilt(double encoders){ 
        double speed = tiltPIDController.calculate(endEffectorTilt.getEncoder().getPosition(), encoders); 
        if(speed > .85){
            speed = (speed/Math.abs(speed))*.85;
        }
        endEffectorTilt.set(speed);
    }
    /*
     *     public void goToHeight(double height){
        double voltage = elevatorPID.calculate(elevator1.getEncoder().getPosition(), height);
        if(Math.abs(voltage) > 3){
            voltage = (voltage)/Math.abs(voltage)*3;
        }
        SmartDashboard.putNumber("elevator PID Voltage", voltage);
        elevator1.setVoltage(voltage);
        elevator2.setVoltage(voltage);

    }
     */

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("End Effector Encoder", endEffectorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("End Effector Tilt", endEffectorTilt.getEncoder().getPosition());
        SmartDashboard.putNumber("End Effector Tilt Absolute Encoder", endEffectorTilt.getAlternateEncoder().getPosition());
    }

    public boolean atSetPoint() {
        return tiltPIDController.atSetpoint(); 
    }
}