package frc.utils;

public class JoystickUtils 
{
    public JoystickUtils(){ }
    
    /**Method that will dampen the joystick inputs
     * 
     * @param value the joystick input
     * @param deadband Range around zero.
     * @param sensitivity how much sensativity will be applied. This is a number between [0,1]
     * @return returns new value of joystick input
     */
    public static double applySensitivity(double value, double deadband, double sensitivity)
    {
        //if joystick value is below the deadband return 0
        if(Math.abs(value) <= deadband)
        {
            return 0; 
        }
        double newValue = sensitivity * Math.pow(value, 3);
        newValue += (1-sensitivity) * value;

        return newValue;
    }
}
