package frc.robot.commands.TelescopeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopeCommand extends Command
{
    private final TelescopeSubsystem subsystem;
    private double desiredDistance;

    public TelescopeCommand(TelescopeSubsystem telescopeSubsystem, double _desiredDistance)
    {
        subsystem = telescopeSubsystem;
        desiredDistance = _desiredDistance;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        subsystem.setPidSetpoint(desiredDistance);
    }

    @Override
    public void execute()
    {
        subsystem.setTelescopeDistance(desiredDistance);
    }

    @Override
    public boolean isFinished()
    {
        return subsystem.getPidAtSetpoint();
    }

    @Override 
    public void end(boolean interrupted)
    {
        subsystem.setTelescopeMotor(0);
    }
}
