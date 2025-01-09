package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule 
{
    private final CANSparkMax drivingMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkPIDController drivingPIDController;
    private final SparkPIDController turningPIDController;

    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private SwerveModuleState optimizedState = new SwerveModuleState(0.0,new Rotation2d());

    public SwerveModule(int driveMotorId, int turnMotorId, double p_chassisAngularOffset)
    {
        drivingMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turnMotorId, MotorType.kBrushless);

        //factory reset motor controllers to get them to a know state
        drivingMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

        //Apply conversion factors to encoders to be used with the wpilib api
        drivingEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor);
        drivingEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);
        turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);

        //invert turning encoder since shaft rotates opposite direction of turning motor
        turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted);

        //PID controller setup
        drivingPIDController = drivingMotor.getPIDController();
        turningPIDController = turningMotor.getPIDController();
        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        //set PID constants and min/max output
        drivingPIDController.setP(SwerveModuleConstants.kDrivingP);
        drivingPIDController.setI(SwerveModuleConstants.kDrivingI);
        drivingPIDController.setD(SwerveModuleConstants.kDrivingD);
        drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF);
        drivingPIDController.setOutputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput);

        turningPIDController.setP(SwerveModuleConstants.kTurningP);
        turningPIDController.setI(SwerveModuleConstants.kTurningI);
        turningPIDController.setD(SwerveModuleConstants.kTurningD);
        turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput);
        //enable PID wrapping for turning encoder
        turningPIDController.setPositionPIDWrappingEnabled(SwerveModuleConstants.kEnablePIDWrapping);
        turningPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
        turningPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        drivingMotor.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode);
        turningMotor.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
        drivingMotor.setSmartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
        turningMotor.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

        //save motor settings
        drivingMotor.burnFlash();
        turningMotor.burnFlash();

        chassisAngularOffset = p_chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        optimizedState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState p_desiredState)
    {
        //make new desired state with angular offset
        SwerveModuleState correctDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, 
            desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset)));

        //make optimize desired state to make sure the wheel never turns more than 90 degrees
        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctDesiredState, 
            new Rotation2d(turningEncoder.getPosition()));

        //Command driving and turning motors to their respective setpoints
        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

        desiredState = p_desiredState;
        optimizedState = optimizedDesiredState;
    }

    public void setTurnMotor(double percent)
    {
        turningMotor.set(percent);
    }

    public void setDriveMotor(double percent)
    {
        drivingMotor.set(percent);
    }

    public SwerveModuleState getDesiredState()
    {
        return desiredState;
    }

    public SwerveModuleState getOptimizedState()
    {
        return optimizedState;
    }

    public SwerveModuleState getModuleState()
    {
        return new SwerveModuleState(drivingEncoder.getVelocity(), 
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
    }

    public SparkPIDController getPIDController()
    {
        return drivingPIDController;
    }

    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset)
        );
    }

    public void resetEncoder()
    {
        drivingEncoder.setPosition(0);
    }
}