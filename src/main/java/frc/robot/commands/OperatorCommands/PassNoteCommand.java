package frc.robot.commands.OperatorCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FlyWheelCommands.FlywheelHoldCommand;
import frc.robot.commands.IntakeCommands.FeedShooterCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;

public class PassNoteCommand extends SequentialCommandGroup
{
    public PassNoteCommand(ConveyorSubsystem conveyor, FlyWheelSubsystem flyWheel)
    {
        addCommands(Commands.deadline(new FlywheelHoldCommand(flyWheel), new FeedShooterCommand(conveyor)));
    }
}
