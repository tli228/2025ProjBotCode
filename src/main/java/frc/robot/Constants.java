package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
  public static class DrivetrainConstants
  {
    //CAN Id's for the driving and turning motors
    public static final int kFrontLeftDrivingCANId = 16;
    public static final int kFrontRightDrivingCANId = 17;
    public static final int kBackLeftDrivingCANId = 15;
    public static final int kBackRightDrivingCANId = 13;

    public static final int kFrontLeftTurningCANId = 11;
    public static final int kFrontRightTurningCANId = 10;
    public static final int kBackLeftTurningCANId = 12;
    public static final int kBackRightTurningCANId = 14;
   // updated the CAN id's hopefully

    
  }

}