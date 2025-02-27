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
    public void initialize() {}

    @Override
    public void execute() 
    {
        RobotContainer.endEffector.setSpeedEndEffectorAlgae(speed);
        RobotContainer.endEffector.setSpeedEndEffectorFinger(speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}