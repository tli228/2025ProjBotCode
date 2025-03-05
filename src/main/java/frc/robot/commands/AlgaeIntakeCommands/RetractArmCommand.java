package frc.robot.commands.AlgaeIntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RetractArmCommand extends Command
{
    private final double speed;

    public RetractArmCommand(Double _speed)
    {
        addRequirements(RobotContainer.algaeIntake);
        speed = _speed;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        RobotContainer.algaeIntake.setSpeedArmTilt(-speed);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}