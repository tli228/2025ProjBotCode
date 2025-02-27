package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule 
{
    private final SparkMax drivingMotor;
    private final SparkMax turningMotor;

    private SparkMaxConfig motorConfig;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingPIDController;
    private final SparkClosedLoopController turningPIDController;

    private final double chassisAngularOffset;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());
    private SwerveModuleState optimizedState = new SwerveModuleState(0.0,new Rotation2d());

    public SwerveModule(int driveMotorId, int turnMotorId, double p_chassisAngularOffset)
    {
        drivingMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turnMotorId, MotorType.kBrushless);

        //factory reset motor controllers to get them to a know state
        //drivingMotor.restoreFactoryDefaults(); // Deprecated
        //turningMotor.restoreFactoryDefaults();

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        motorConfig = new SparkMaxConfig();

        motorConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

        //Apply conversion factors to encoders to be used with the wpilib api
        //drivingEncoder.setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor); // Deprecated
        //drivingEncoder.setVelocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor);
        //turningEncoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor);
        //turningEncoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor);

        //invert turning encoder since shaft rotates opposite direction of turning motor
        //turningEncoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted); // Deprecated

        //PID controller setup
        drivingPIDController = drivingMotor.getClosedLoopController();
        turningPIDController = turningMotor.getClosedLoopController();
        //drivingPIDController.setFeedbackDevice(drivingEncoder); // Deprecated
        //turningPIDController.setFeedbackDevice(turningEncoder);

        //set PID constants and min/max output
        motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        .p(SwerveModuleConstants.kTurningP)
        .i(SwerveModuleConstants.kTurningI)
        .d(SwerveModuleConstants.kTurningD)
        .outputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput)
        // Set PID values for velocity control in slot 1
        .p(SwerveModuleConstants.kDrivingP, ClosedLoopSlot.kSlot1)
        .i(SwerveModuleConstants.kDrivingI, ClosedLoopSlot.kSlot1)
        .d(SwerveModuleConstants.kDrivingD, ClosedLoopSlot.kSlot1)
        .velocityFF(SwerveModuleConstants.kDrivingFF, ClosedLoopSlot.kSlot1)
        .outputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput, ClosedLoopSlot.kSlot1);

        //drivingPIDController.setP(SwerveModuleConstants.kDrivingP); // Deprecated
        //drivingPIDController.setI(SwerveModuleConstants.kDrivingI);
        //drivingPIDController.setD(SwerveModuleConstants.kDrivingD);
        //drivingPIDController.setFF(SwerveModuleConstants.kDrivingFF);
        //drivingPIDController.setOutputRange(SwerveModuleConstants.kDrivingMinOutput, SwerveModuleConstants.kDrivingMaxOutput);

        //turningPIDController.setP(SwerveModuleConstants.kTurningP); // Deprecated
        //turningPIDController.setI(SwerveModuleConstants.kTurningI);
        //turningPIDController.setD(SwerveModuleConstants.kTurningD);
        //turningPIDController.setOutputRange(SwerveModuleConstants.kTurningMinOutput, SwerveModuleConstants.kTurningMaxOutput);
        
        drivingMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        turningMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        //enable PID wrapping for turning encoder
        //turningPIDController.setPositionPIDWrappingEnabled(SwerveModuleConstants.kEnablePIDWrapping); // Deprecated
        //turningPIDController.setPositionPIDWrappingMinInput(SwerveModuleConstants.kTurningEncoderPositionPIDMinInput);
        //turningPIDController.setPositionPIDWrappingMaxInput(SwerveModuleConstants.kTurningEncoderPositionPIDMaxInput);

        //drivingMotor.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode); // Deprecated
        //turningMotor.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode);
        //drivingMotor.setSmartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit);
        //turningMotor.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit);

        //save motor settings
        //drivingMotor.burnFlash();
        //turningMotor.burnFlash();

        drivingPIDController.setReference(SwerveModuleConstants.kDrivingMotorCurrentLimit, ControlType.kCurrent, ClosedLoopSlot.kSlot1);
        turningPIDController.setReference(SwerveModuleConstants.kTurningMotorCurrentLimit, ControlType.kCurrent, ClosedLoopSlot.kSlot0);

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
        drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

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

    public SparkClosedLoopController getPIDController()
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