package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase
{
    private WPI_VictorSPX climber1 = new WPI_VictorSPX(ClimberConstants.kClimber1CANId);
    private WPI_VictorSPX climber2 = new WPI_VictorSPX(ClimberConstants.kClimber2CANId);
    private PowerDistribution PDH;
    private final NetworkTableInstance instance;
    private final DoublePublisher climber1CurrentPublisher;
    private final DoublePublisher climber2CurrentPublisher;
    private final DoublePublisher climber1PrecentPublisher;
    private final DoublePublisher climber2PrecentPublisher;

    
    private final DigitalInput limitSwitchRight = new DigitalInput(0);
    private final DigitalInput limitSwitchLeft = new DigitalInput(1);


    public ClimberSubsystem()
    {
        PDH = RobotContainer.pdh;

        climber1.setNeutralMode(NeutralMode.Brake);
        climber2.setNeutralMode(NeutralMode.Brake);

        instance = NetworkTableInstance.getDefault();
        climber1CurrentPublisher = instance.getDoubleTopic("/ClimberSubsystem/MotorsInfo/Climber1/Current").publish();
        climber2CurrentPublisher = instance.getDoubleTopic("/ClimberSubsystem/MotorsInfo/Climber2/Current").publish();
        climber1PrecentPublisher = instance.getDoubleTopic("/ClimberSubsystem/MotorsInfo/Climber1/Precent").publish();
        climber2PrecentPublisher = instance.getDoubleTopic("/ClimberSubsystem/MotorsInfo/Climber2/Precent").publish();
    }

    public boolean getLimitSwitchRight()
    {
        return limitSwitchRight.get();
    }

    public boolean getLimitSwitchLeft()
    {
        return limitSwitchLeft.get();
    }

    public double getCurrent(int num) 
    {
        return PDH.getCurrent(num);
    }

    public void setClimberSpeeds(double num)     
    {
        climber1.set(ControlMode.PercentOutput, num);
        climber2.set(ControlMode.PercentOutput, num);
    }

    @Override
    public void periodic()
    {
        climber1CurrentPublisher.set(getCurrent(ClimberConstants.kClimber1PDH));
        climber2CurrentPublisher.set(getCurrent(ClimberConstants.kClimber2PDH));
        climber1PrecentPublisher.set(climber1.get());
        climber2PrecentPublisher.set(climber2.get());

        SmartDashboard.putBoolean("limitSwitchRight", limitSwitchRight.get());
        SmartDashboard.putBoolean("limitSwitchLeft", limitSwitchLeft.get());

    }
}