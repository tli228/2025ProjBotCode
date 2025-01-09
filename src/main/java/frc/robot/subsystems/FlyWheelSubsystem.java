package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlyWheelConstants;

public class FlyWheelSubsystem extends SubsystemBase
{
    private WPI_VictorSPX flyWheelMotor23 = new WPI_VictorSPX(FlyWheelConstants.kFlywheel1CANId);
    private WPI_VictorSPX flyWheelMotor24 = new WPI_VictorSPX(FlyWheelConstants.kFlywheel2CANId);

     
    private Encoder flyWheelEncoder23 = new Encoder(
        FlyWheelConstants.kFlyWheel23RelativeEncoderDIOChannelA, 
        FlyWheelConstants.kFlyWheel23RelativeEncoderDIOChannelB,
        true,
        EncodingType.k1X);

    private Encoder flyWheelEncoder24 = new Encoder(
        FlyWheelConstants.kFlyWheel24RelativeEncoderDIOChannelA,
        FlyWheelConstants.kFlyWheel24RelativeEncoderDIOChannelB,
        false,
        EncodingType.k1X
    );

    private AnalogInput proximitySensor = new AnalogInput(FlyWheelConstants.kFlyWheelProximitySensorDIOId);
    
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        FlyWheelConstants.kFlyWheelkS, 
        FlyWheelConstants.kFlyWheelkV, 
        FlyWheelConstants.kFlyWheelkA);

    //network tables
    private final NetworkTableInstance instance;

    //motors
    private final DoublePublisher flyWheel23PrecentPublisher;
    private final DoublePublisher flyWheel24PrecentPublisher;

    //encoder 23 table
    private final BooleanPublisher encoder23DirectionPublisher;
    private final DoublePublisher encoder23DistancePublisher;
    private final DoublePublisher encoder23DistancePerPulsePublisher;
    private final DoublePublisher encoder23RatePublisher;
    private final DoublePublisher encoder23RawPublisher;

    //encoder 24 table
    private final BooleanPublisher encoder24DirectionPublisher;
    private final DoublePublisher encoder24DistancePublisher;
    private final DoublePublisher encoder24DistancePerPulsePublisher;
    private final DoublePublisher encoder24RatePublisher;
    private final DoublePublisher encoder24RawPublisher;

    //proximity switch table
    private final DoublePublisher proximityBitPublisher;
    private final DoublePublisher proximityAverageBitPublisher;
    private final DoublePublisher proximityVoltPublisher;

    public FlyWheelSubsystem()
    {
        flyWheelMotor23.setInverted(InvertType.InvertMotorOutput);

        flyWheelMotor23.setNeutralMode(NeutralMode.Coast);
        flyWheelMotor24.setNeutralMode(NeutralMode.Coast);

        flyWheelEncoder24.setSamplesToAverage(10);
        flyWheelEncoder23.setSamplesToAverage(10);


        flyWheelEncoder24.setDistancePerPulse(FlyWheelConstants.kFlyWheelEncoderVelocityFactor);
        flyWheelEncoder23.setDistancePerPulse(FlyWheelConstants.kFlyWheelEncoderVelocityFactor);

        instance = NetworkTableInstance.getDefault();
        flyWheel23PrecentPublisher = instance.getDoubleTopic("/FlyWheelSubsystem/MotorsInfo/FlyWheel23/Precent").publish();
        flyWheel24PrecentPublisher = instance.getDoubleTopic("/FlyWheelSubsystem/MotorsInfo/FlyWheel24/Precent").publish();

        encoder23DirectionPublisher = instance.getBooleanTopic("FlyWheelSubsystem/Encoders/Encoder23/Direction").publish();
        encoder23DistancePublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder23/Distance").publish();
        encoder23DistancePerPulsePublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder23/DistancePerPulse").publish();
        encoder23RatePublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder23/Rate").publish();
        encoder23RawPublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder23/Raw").publish();

        encoder24DirectionPublisher = instance.getBooleanTopic("FlyWheelSubsystem/Encoders/Encoder24/Direction").publish();
        encoder24DistancePublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder24/Distance").publish();
        encoder24DistancePerPulsePublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder24/DistancePerPulse").publish();
        encoder24RatePublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder24/Rate").publish();
        encoder24RawPublisher = instance.getDoubleTopic("FlyWheelSubsystem/Encoders/Encoder24/Raw").publish();

        proximityBitPublisher = instance.getDoubleTopic("/FlyWheelSubsystem/Sensors/ProximitySensor/Bits").publish();
        proximityAverageBitPublisher = instance.getDoubleTopic("/FlyWheelSubsystem/Sensors/ProximitySensor/Bits").publish();
        proximityVoltPublisher = instance.getDoubleTopic("/FlyWheelSubsystem/Sensors/ProximitySensor/Volts").publish();
    }

    /**
     * Method to set fly wheel motor power
     *  
     * @param desiredSpeed desired speed of the motors
     */
    public void setFlyWheelMotors(double desiredPower)
    {
        flyWheelMotor23.set(ControlMode.PercentOutput, desiredPower);
        flyWheelMotor24.set(ControlMode.PercentOutput, desiredPower);
    }

    /**
     * Method to set fly wheel motor velocity in meters per second
     * 
     * @param desiredVelocity desired velocity of the motors in meters per second
     */
    public void setFlyWheelVelocity(double desiredVelocity)
    {
        double feedForwardVal = feedforward.calculate(desiredVelocity);

        flyWheelMotor23.setVoltage(feedForwardVal);
        flyWheelMotor24.setVoltage(feedForwardVal);
    }

    public double[] getEncoderDistance()
    {
        return new double[] {flyWheelEncoder23.getDistance(), flyWheelEncoder24.getDistance()};
    }
    
    public boolean getProximitySensor()
    {
        if(proximitySensor.getVoltage() < 1)
        {
            return true;
        }
        return false;
    }

    public void setFlyWheelMotorsCoast()
    {
        flyWheelMotor23.setNeutralMode(NeutralMode.Coast);
        flyWheelMotor24.setNeutralMode(NeutralMode.Coast);
    }

    public void setFlyWheelMotorsBrake()
    {
        flyWheelMotor23.setNeutralMode(NeutralMode.Brake);
        flyWheelMotor24.setNeutralMode(NeutralMode.Brake);
    }


    @Override
    public void periodic()
    {
        //motorinfo
        flyWheel23PrecentPublisher.set(flyWheelMotor23.get());
        flyWheel24PrecentPublisher.set(flyWheelMotor24.get());

        //encoder 23
        encoder23DirectionPublisher.set(flyWheelEncoder23.getDirection());
        encoder23DistancePublisher.set(flyWheelEncoder23.getDistance());
        encoder23DistancePerPulsePublisher.set(flyWheelEncoder23.getDistancePerPulse());
        encoder23RatePublisher.set(flyWheelEncoder23.getRate());
        encoder23RawPublisher.set(flyWheelEncoder23.getRaw());

        //encoder 24
        encoder24DirectionPublisher.set(flyWheelEncoder24.getDirection());
        encoder24DistancePublisher.set(flyWheelEncoder24.getDistance());
        encoder24DistancePerPulsePublisher.set(flyWheelEncoder24.getDistancePerPulse());
        encoder24RatePublisher.set(flyWheelEncoder24.getRate());
        encoder24RawPublisher.set(flyWheelEncoder24.getRaw());

        //proximity sensor
        proximityBitPublisher.set(proximitySensor.getValue());
        proximityAverageBitPublisher.set(proximitySensor.getAverageBits());
        proximityVoltPublisher.set(proximitySensor.getVoltage());
    }
}
