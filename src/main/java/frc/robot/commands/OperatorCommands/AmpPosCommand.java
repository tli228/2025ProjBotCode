package frc.robot.commands.OperatorCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FlyWheelCommands.FlywheelHoldCommand;
import frc.robot.commands.IntakeCommands.FeedShooterCommand;
import frc.robot.commands.TelescopeCommands.TelescopeCommand;
import frc.robot.commands.TiltCommands.TiltCommand;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.FlyWheelSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TiltSubsystem;

public class AmpPosCommand extends SequentialCommandGroup
{


    public AmpPosCommand(TelescopeSubsystem telescope, TiltSubsystem tilt, ConveyorSubsystem conveyor, FlyWheelSubsystem flyWheel) 
    {


        addCommands(
            Commands.deadline(new FlywheelHoldCommand(flyWheel), new FeedShooterCommand(conveyor)),
            Commands.parallel(
                new TelescopeCommand(telescope, Units.inchesToMeters(8)),
                new TiltCommand(tilt, Units.degreesToRadians(90))
            ),
            new TiltCommand(tilt, Units.degreesToRadians(130)) //change
        );
    }
}
