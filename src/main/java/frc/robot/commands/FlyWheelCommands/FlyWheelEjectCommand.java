package frc.robot.commands.FlyWheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;

public class FlyWheelEjectCommand extends Command
{
    private FlyWheelSubsystem subsystem;

    public FlyWheelEjectCommand(FlyWheelSubsystem TelescopeSubsystem)
    {
        subsystem = TelescopeSubsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() 
    {
        subsystem.setFlyWheelMotors(-0.5);
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.setFlyWheelMotors(0);
    }
    
}
