package frc.robot.commands.TiltCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TiltSubsystem;

public class TiltCommand extends Command
{
    private final TiltSubsystem subsystem;
    private double desiredAngle;

    public TiltCommand(TiltSubsystem telescopeSubsystem, double _desiredAngle)
    {
        subsystem = telescopeSubsystem;
        desiredAngle = _desiredAngle;
        addRequirements(subsystem);
    }

    @Override
    public void initialize()
    {
        subsystem.setPidSetpoint(desiredAngle);
    }

    @Override
    public void execute()
    {
        subsystem.setTiltPosition(desiredAngle);
    }

    @Override
    public boolean isFinished()
    {
        return subsystem.getPidAtSetpoint();
    }

    @Override 
    public void end(boolean interrupted)
    {
        subsystem.setTiltMotor(0);
    }
}