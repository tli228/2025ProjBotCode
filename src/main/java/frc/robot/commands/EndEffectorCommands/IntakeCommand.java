package frc.robot.commands.EndEffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class IntakeCommand extends Command 
{
    private final double speed;

    public IntakeCommand(Double _speed)
    {
        addRequirements(RobotContainer.intake);
        speed = _speed;
    }

    @Override
    public void initialize() {
        RobotContainer.intake.intake(0);
    }

    @Override
    public void execute() 
    {
        RobotContainer.intake.intake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.intake.intake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}