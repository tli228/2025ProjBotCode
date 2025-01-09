package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DriveSubsystem extends SubsystemBase
{
    
    private final SwerveModule frontLeft = new SwerveModule(
        DrivetrainConstants.kFrontLeftDrivingCANId, 
        DrivetrainConstants.kFrontLeftTurningCANId, 
        DrivetrainConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule frontRight = new SwerveModule(
        DrivetrainConstants.kFrontRightDrivingCANId, 
        DrivetrainConstants.kFrontRightTurningCANId, 
        DrivetrainConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule backLeft = new SwerveModule(
        DrivetrainConstants.kBackLeftDrivingCANId, 
        DrivetrainConstants.kBackLeftTurningCANId, 
        DrivetrainConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule backRight = new SwerveModule(
        DrivetrainConstants.kBackRightDrivingCANId, 
        DrivetrainConstants.kBackRightTurningCANId, 
        DrivetrainConstants.kBackRightChassisAngularOffset);

    private AHRS imu = new AHRS(SPI.Port.kMXP);

    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DrivetrainConstants.kDriveKinematics, getHeading(), getModulePositions());

    private Field2d field = new Field2d();

    private final StructArrayPublisher<SwerveModuleState> measuredSwerveStatePublisher;
    private final StructArrayPublisher<SwerveModuleState> setpointSwerveStatePublisher;

    
    public DriveSubsystem() 
    { 
        /* 
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getSpeeds, 
            this::driveWithChassisSpeeds, 
            DrivetrainConstants.kPathFollowerConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();

                if(alliance.isPresent())
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            this);

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);*/

        measuredSwerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveModuleState/Measured", SwerveModuleState.struct).publish();
        
        setpointSwerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/SwerveModuleState/Setpoint", SwerveModuleState.struct).publish();
    }

    /**
     * Method to drive drivetrain with joysticks
     * 
     * @param xSpeed speed of robot in x direction (forward)
     * @param ySpeed speed of robot in y direction (sideways)
     * @param rotSpeed angular rate of the robot 
     * @param fieldRelative set if x and y speeds are relative to the field
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) 
    { 
        xSpeed *= DrivetrainConstants.kMaxSpeedMetersPerSecond;
        ySpeed *= DrivetrainConstants.kMaxSpeedMetersPerSecond;
        rotSpeed *= DrivetrainConstants.kMaxAngularSpeed;


        var swerveModuleStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative ? //if fieldRelative
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getHeading())
                :new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));


        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }


    public SwerveDriveKinematics getKinematics()
    {
        return DrivetrainConstants.kDriveKinematics;
    }

    /**
     * A temperary method for testing the swerve modules. This will be used to tune PIDs
     * 
     * @param speed velocity of the wheels in meters per second
     * @param angle angle of the wheels in radians
     */
    public void testSwerve(double speed, double angle)
    {
        SwerveModuleState testState = new SwerveModuleState(speed, new Rotation2d(angle));

        frontLeft.setDesiredState(testState);
        frontRight.setDesiredState(testState);
        backLeft.setDesiredState(testState);
        backRight.setDesiredState(testState);
    }

    public void setTurningMotors(double percent)
    {
        frontLeft.setTurnMotor(percent);
        frontRight.setTurnMotor(percent);
        backLeft.setTurnMotor(percent);
        backRight.setTurnMotor(percent);           
    }

    public void setDrivingMotors(double percent)
    {
        frontLeft.setDriveMotor(percent);
        frontRight.setDriveMotor(percent);
        backLeft.setDriveMotor(percent);
        backRight.setDriveMotor(percent);
    }

    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] targetStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, DrivetrainConstants.kMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(targetStates[0]);
        frontRight.setDesiredState(targetStates[1]);
        backLeft.setDesiredState(targetStates[2]);
        backRight.setDesiredState(targetStates[3]);
    }

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose)
    {
        odometry.resetPosition(
            getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getSpeeds()
    {
        return DrivetrainConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getHeading()
    {
        return Rotation2d.fromDegrees(-imu.getAngle());
    }

    public SwerveModuleState[] getModuleStates()
    {
        return new SwerveModuleState[]{
            frontLeft.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            backRight.getModuleState()};
    }

    public SwerveModulePosition[] getModulePositions()
    {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(), frontRight.getPosition(),
            backLeft.getPosition(), backRight.getPosition()};
    }

    @Override
    public void periodic()
    {
        //update odometry
        odometry.update(getHeading(), getModulePositions());

        //publish to networktable for advantagescope
        measuredSwerveStatePublisher.set(getModuleStates());

        field.setRobotPose(getPose());

        setpointSwerveStatePublisher.set(new SwerveModuleState[]{
            frontLeft.getDesiredState(),
            frontRight.getDesiredState(),
            backLeft.getDesiredState(),
            backRight.getDesiredState()
        });
    }
}
