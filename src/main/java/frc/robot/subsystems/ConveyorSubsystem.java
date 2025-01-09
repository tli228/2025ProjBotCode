package frc.robot.subsystems;
import frc.robot.Constants.ConveyorConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ConveyorSubsystem extends SubsystemBase
{

    private VictorSPX conveyorMotor1 = new VictorSPX(ConveyorConstants.conveyorCANID19);
    private VictorSPX conveyorMotor2 = new VictorSPX(ConveyorConstants.conveyorCANID20);
    private CANSparkMax intakeMotor = new CANSparkMax(ConveyorConstants.intakeCAN, MotorType.kBrushless);
    private AnalogInput proximitySensor = new AnalogInput(ConveyorConstants.kConveyorProximitySensorDIOId);

    private final NetworkTableInstance instance;
    private final DoublePublisher conveyorMotor1PrecentPublisher;
    private final DoublePublisher conveyorMotor2PrecentPublisher;
    private final DoublePublisher intakeMotorPrecentPublisher;
    private final DoublePublisher proximityBitPublisher;
    private final DoublePublisher proximityAverageBitPublisher;
    private final DoublePublisher proximityVoltPublisher;

    public ConveyorSubsystem() 
    { 
        conveyorMotor1.setNeutralMode(NeutralMode.Brake);
        conveyorMotor2.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        conveyorMotor1.setInverted(InvertType.InvertMotorOutput);

        instance = NetworkTableInstance.getDefault();
        conveyorMotor1PrecentPublisher = instance.getDoubleTopic("/ConveyorSubsystem/MotorInfo/ConveyorMotor1/Precent").publish();
        conveyorMotor2PrecentPublisher = instance.getDoubleTopic("/ConveyorSubsystem/MotorInfo/ConveyorMotor2/Precent").publish();
        intakeMotorPrecentPublisher = instance.getDoubleTopic("/ConveyorSubsystem/MotorInfo/IntakeMotor/Precent").publish();

        proximityBitPublisher = instance.getDoubleTopic("/ConveyorSubsystem/Sensors/ProximitySensor/Bits").publish();
        proximityAverageBitPublisher = instance.getDoubleTopic("/ConveyorSubsystem/Sensors/ProximitySensor/AverageBits").publish();
        proximityVoltPublisher = instance.getDoubleTopic("/ConveyorSubsystem/Sensors/ProximitySensor/Volts").publish();
    }

    public void setConveyorMotors(double output)
    {
        conveyorMotor1.set(ControlMode.PercentOutput, output);
        conveyorMotor2.set(ControlMode.PercentOutput, output);
    }

    public void setIntakeMotor(double output)
    {
        intakeMotor.set(output);
    }

    public boolean getProximitySensor()
    {
        if(proximitySensor.getVoltage() < 1)
        {
            return true;
        }
        return false;
    }
    
    @Override
    public void periodic() 
    { 
        conveyorMotor1PrecentPublisher.set(conveyorMotor1.getMotorOutputPercent());
        conveyorMotor2PrecentPublisher.set(conveyorMotor2.getMotorOutputPercent());
        intakeMotorPrecentPublisher.set(intakeMotor.get());

        proximityBitPublisher.set(proximitySensor.getValue());
        proximityAverageBitPublisher.set(proximitySensor.getAverageValue());
        proximityVoltPublisher.set(proximitySensor.getVoltage());
    }
}
