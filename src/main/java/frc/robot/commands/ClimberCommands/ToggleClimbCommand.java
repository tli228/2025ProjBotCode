package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggleClimbCommand extends Command
{
    private final ClimberSubsystem climber;
    
    private int num;

    public ToggleClimbCommand(ClimberSubsystem subsystem)
    {
        addRequirements(subsystem);
        climber = subsystem;
        num = 0;
    }

    @Override
    public void initialize() {
        num++;
    }

    @Override
    public void execute() {
        if(num%2 == 1) {
            climber.setClimberSpeeds(0.2);
        }
        else {
            climber.setClimberSpeeds(-0.2);
        }
    }

    @Override
    public boolean isFinished()
    {
        if(climber.getCurrent(ClimberConstants.kClimber1PDH) > ClimberConstants.currentMax || climber.getCurrent(ClimberConstants.kClimber2PDH) > ClimberConstants.currentMax) { 
            return true;
        }
        return false;
    }

    @Override 
    public void end(boolean interrupted)
    {
        climber.setClimberSpeeds(0);
    }
}
