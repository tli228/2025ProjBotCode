package frc.robot.commands.FlyWheelCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelSubsystem;

public class FlyWheelShootCommand extends Command
{
    private FlyWheelSubsystem subsystem;
    private int counter;

    public FlyWheelShootCommand(FlyWheelSubsystem flyWheelSubsystem)
    {
        subsystem = flyWheelSubsystem;
        subsystem.setFlyWheelMotorsCoast();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() 
    {
        counter = 0;
    }

    @Override 
    public void execute()
    {
        if(subsystem.getProximitySensor()) 
        {
            if(counter == 0) //When the top of the note is sensed 
            {
                counter++;
            }
            else if(counter == 2) // when the bottom of the note is sensed
            {
                counter++;
            }
        }
        else if(counter == 1) //When the sensor is in the hole of the note
        {
            counter++;
        }
        else if(counter == 3) // when the note passes the sensor
        {
            counter++;
        }

        subsystem.setFlyWheelMotors(1);
    }

    @Override
    public boolean isFinished()
    {        
        if(counter == 4)
        {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        subsystem.setFlyWheelMotors(0); //go back to zero when finished
        counter = 0;
    }
}