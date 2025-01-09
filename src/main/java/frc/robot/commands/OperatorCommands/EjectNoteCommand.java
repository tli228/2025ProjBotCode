package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FlyWheelCommands.FlyWheelEjectCommand;
import frc.robot.commands.IntakeCommands.IntakeEjectNoteCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

public class EjectNoteCommand extends ParallelCommandGroup
{
    private ConveyorSubsystem conveyor;
    private FlyWheelSubsystem flyWheel;

    public EjectNoteCommand(ConveyorSubsystem _conveyor, FlyWheelSubsystem _flyWheel)
    {
        conveyor = _conveyor;
        flyWheel = _flyWheel;

        addCommands(
            new IntakeEjectNoteCommand(conveyor),
            new FlyWheelEjectCommand(flyWheel)
        );
    }
}
