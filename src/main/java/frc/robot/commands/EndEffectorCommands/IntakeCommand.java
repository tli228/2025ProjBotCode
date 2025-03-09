package frc.robot.commands.EndEffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class IntakeCommand extends Command 
{
    private final double speed;

    public IntakeCommand(Double _speed)
    {
        addRequirements(RobotContainer.endEffector);
        speed = _speed;
    }

    @Override
    public void initialize() {
        RobotContainer.endEffector.setSpeedEndEffectorIntake(0);
    }

    @Override
    public void execute() 
    {
        RobotContainer.endEffector.setSpeedEndEffectorIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.endEffector.setSpeedEndEffectorIntake(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}