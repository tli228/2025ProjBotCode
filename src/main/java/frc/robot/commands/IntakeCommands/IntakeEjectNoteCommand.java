package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class IntakeEjectNoteCommand extends Command 
{
    private final ConveyorSubsystem subsystem;

    public IntakeEjectNoteCommand(ConveyorSubsystem conveyorSubsystem)
    {
        addRequirements(conveyorSubsystem);
        subsystem = conveyorSubsystem;
    }

    @Override
    public void initialize() { }

    @Override 
    public void execute()
    {
        subsystem.setConveyorMotors(-0.2);
        subsystem.setIntakeMotor(-0.5);
    }

    @Override 
    public void end(boolean interrupted)
    {
        subsystem.setConveyorMotors(0);
        subsystem.setIntakeMotor(0);
    }

}
