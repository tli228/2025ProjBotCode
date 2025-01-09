package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunIntakeCommand extends Command 
{
    private final ConveyorSubsystem subsystem;
    private boolean isTriggered = false;

    public RunIntakeCommand(ConveyorSubsystem conveyorSubsystem)
    {
        addRequirements(conveyorSubsystem);
        subsystem = conveyorSubsystem;
    }

    @Override
    public void initialize() 
    { 
        isTriggered = false;
    }

    @Override 
    public void execute()
    {
        subsystem.setConveyorMotors(0.4);
        subsystem.setIntakeMotor(0.3);

        if(subsystem.getProximitySensor())
        {
            isTriggered = true;
        }
    }

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
        isTriggered = false;
        subsystem.setIntakeMotor(0);
        subsystem.setConveyorMotors(0);
    }

}
