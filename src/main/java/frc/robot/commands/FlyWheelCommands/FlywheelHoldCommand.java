package frc.robot.commands.FlyWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;

//Command to take in and hold note to score in the amp
public class FlywheelHoldCommand extends Command
{
    private final FlyWheelSubsystem subsystem;
    private boolean isTriggered = false;

    public FlywheelHoldCommand(FlyWheelSubsystem flyWheelSubsystem)
    {
        subsystem = flyWheelSubsystem;
        subsystem.setFlyWheelMotorsBrake();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() 
    { 
        isTriggered = false;
    }

    @Override
    public void execute()
    {
        subsystem.setFlyWheelMotors(0.2);

        if(subsystem.getProximitySensor())
        {
            isTriggered = true;
        }

    }

    //ends when proximity sensor is active
    @Override
    public boolean isFinished()
    {
        if(!subsystem.getProximitySensor() && isTriggered)
        {
            return true;
        }

        return false;
    }

    @Override 
    public void end(boolean interrupted)
    {
        subsystem.setFlyWheelMotors(0);
        isTriggered = false;
    }
}
