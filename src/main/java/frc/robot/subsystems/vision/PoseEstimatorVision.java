package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class PoseEstimatorVision extends SubsystemBase
{
    private final DriveSubsystem driveTrain;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Vision camera;

    private final Field2d field = new Field2d();

    public PoseEstimatorVision(Vision _camera, DriveSubsystem _driveTrain)
    {
        driveTrain = _driveTrain;
        camera = _camera;

        poseEstimator = new SwerveDrivePoseEstimator
        (driveTrain.getKinematics(), 
        driveTrain.getHeading(), 
        driveTrain.getModulePositions(), 
        driveTrain.getPose(),
        VecBuilder.fill(PoseEstimatorConstants.kVisionStdDevX, 
            PoseEstimatorConstants.kPositionStdDevY, 
            PoseEstimatorConstants.kPositionStdDevTheta),
        VecBuilder.fill(PoseEstimatorConstants.kVisionStdDevX,
        PoseEstimatorConstants.kPositionStdDevY,
        PoseEstimatorConstants.kPositionStdDevTheta));

        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            driveTrain::getSpeeds, 
            driveTrain::driveWithChassisSpeeds, 
            DrivetrainConstants.kPathFollowerConfig, 
            () -> {
                var alliance = DriverStation.getAlliance();

                if(alliance.isPresent())
                {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }, 
            driveTrain);

        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    }

    public Pose2d getPose()
    {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose)
    {
        poseEstimator.resetPosition(driveTrain.getHeading(), driveTrain.getModulePositions(), pose);
    }

    @Override
    public void periodic()
    {
        poseEstimator.update(driveTrain.getHeading(), driveTrain.getModulePositions());

        try
        {
            //EstimatedRobotPose estimatedRobotPose = camera.getEstimatedRobotPose(poseEstimator.getEstimatedPosition()).get();
            //poseEstimator.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);

        }
        catch(Exception e){}
    }
}
