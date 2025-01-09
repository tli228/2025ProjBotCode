package frc.robot.commands.OperatorCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TelescopeCommands.TelescopeCommand;
import frc.robot.commands.TiltCommands.TiltCommand;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TiltSubsystem;

public class ResetShooterCommand extends SequentialCommandGroup
{
    private TelescopeSubsystem telescope;
    private TiltSubsystem tilt;

    public ResetShooterCommand(TelescopeSubsystem _telescope, TiltSubsystem _tilt)
    {
        telescope = _telescope;
        tilt = _tilt;

        addCommands(
            Commands.parallel(
                new TelescopeCommand(telescope, 0),
                new TiltCommand(tilt, Units.degreesToRadians(15))
            ),
            new TiltCommand(tilt, 0)
        );
    }
}
