package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class GrabCageCommand extends Command 
{
    private final double speed;

    public GrabCageCommand(Double _speed)
    {
        addRequirements(RobotContainer.climber);
        speed = _speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.climber.Grab(speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}