package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ExtendWinchCommand extends Command 
{
    private final double speed;

    public ExtendWinchCommand(Double _speed)
    {
        addRequirements(RobotContainer.climber);
        speed = _speed;
    }

    @Override
    public void initialize() {
        RobotContainer.climber.Winch(0);
    }

    @Override
    public void execute() {
        RobotContainer.climber.Winch(speed);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.climber.Winch(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}