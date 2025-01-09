package frc.robot.commands.FlyWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;

public class ShootInAmpCommand extends Command
{
    private final FlyWheelSubsystem subsystem;
    private boolean isTriggered = false;

    public ShootInAmpCommand(FlyWheelSubsystem flyWheelSubsystem)
    {
        subsystem = flyWheelSubsystem;
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
        subsystem.setFlyWheelMotors(1);

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