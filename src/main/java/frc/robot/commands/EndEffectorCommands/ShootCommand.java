package frc.robot.commands.EndEffectorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootCommand extends Command 
{
    private final double speed;

    public ShootCommand(Double _speed)
    {
        addRequirements(RobotContainer.endEffector);
        speed = _speed;
    }

    @Override
    public void initialize() {
        RobotContainer.endEffector.setSpeedEndEffectorMotor(0);
    }

    @Override
    public void execute() 
    {
        RobotContainer.endEffector.setSpeedEndEffectorMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        RobotContainer.endEffector.setSpeedEndEffectorMotor(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}