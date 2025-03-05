package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ElevatorUpCommand extends Command 
{
    private final double speed;

    public ElevatorUpCommand(Double _speed)
    {
        addRequirements(RobotContainer.elevator);
        speed = _speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() 
    {
        RobotContainer.elevator.setSpeedElevator1(speed);
        RobotContainer.elevator.setSpeedElevator2(speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}