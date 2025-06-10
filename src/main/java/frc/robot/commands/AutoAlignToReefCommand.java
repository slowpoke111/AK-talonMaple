package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.CustomAprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;

public class AutoAlignToReefCommand extends Command {

    private final AprilTagFieldLayout tagLayout = DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red
                    ? CustomAprilTagFieldLayout.getFieldLayout(new int[] { 6, 7, 8, 9, 10, 11 })
                    : CustomAprilTagFieldLayout.getFieldLayout(new int[] { 17, 18, 19, 20, 21, 22 }))
            .orElseGet(() -> CustomAprilTagFieldLayout.getFieldLayout(new int[] { 17, 18, 19, 20, 21, 22 }));
    private final AprilTagFieldLayout adjustTagFieldLayout = adjustForBumpers(tagLayout, 0.5);
    private final Drive m_Swerve;
    private final Vision m_Vision;
    private AprilTag m_TargetTag = null;
    private Pose2d targetPose = null;
    private Command firstAlign;
    private Command secondPIDAlign;
    private boolean firstAlignFinished = false;
    private boolean secondPIDStarted = false;
    private Timer timerPID = new Timer();
    private Timer timerTotal = new Timer();

    public AutoAlignToReefCommand(Drive drive, Vision vision) {
        this.m_Swerve = drive;
        this.m_Vision = vision;
        addRequirements(drive, vision);
    }

    public void initialize() {
        if (m_TargetTag == null) {
            m_TargetTag = getNearestTag();
        }
        targetPose = m_TargetTag.pose.toPose2d()
                .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
        
        firstAlign = new PIDAlignCommand(m_Swerve, new Pose2d(m_Swerve.getPose().getTranslation(), targetPose.getRotation()))
                .withTimeout(1)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .andThen(new FollowPathCommand(m_Swerve, targetPose)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        secondPIDAlign = new PIDAlignCommand(m_Swerve, targetPose)
                .withTimeout(3)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        firstAlignFinished = false;
        secondPIDStarted = false;
        timerPID.reset();
        timerTotal.reset();
        timerTotal.start();

        firstAlign.schedule();
    }

    public void execute() {
        if (firstAlign.isFinished() && !firstAlignFinished) {
            firstAlignFinished = true;
        }
        
        if (firstAlignFinished && !secondPIDStarted) {
            secondPIDStarted = true;
            timerPID.start();
            secondPIDAlign.schedule();
        }
    }

    public boolean isFinished() {

        return (secondPIDStarted && timerPID.hasElapsed(3)) || 
               timerTotal.hasElapsed(8) ||
               (secondPIDStarted && secondPIDAlign.isFinished())  ;
    }

    public void end(boolean interrupted) {
        if (firstAlign != null && !firstAlign.isFinished()) {
            firstAlign.cancel();
        }
        if (secondPIDAlign != null && !secondPIDAlign.isFinished()) {
            secondPIDAlign.cancel();
        }
        
        timerPID.stop();
        timerPID.reset();
        timerTotal.stop();
        timerTotal.reset();
        
        firstAlignFinished = false;
        secondPIDStarted = false;
        m_TargetTag = null;
        targetPose = null;
        
    }

    public AprilTag getNearestTag() {
        Pose2d robotPose = m_Swerve.getPose();

        AprilTag nearestTag = null;
        double minDistance = Double.MAX_VALUE;

        for (AprilTag tag : adjustTagFieldLayout.getTags()) {
            Optional<Pose3d> tagPoseOptional = adjustTagFieldLayout.getTagPose(tag.ID);
            if (tagPoseOptional.isEmpty()) {
                continue;
            }

            Pose3d tagPose = tagPoseOptional.get();
            Translation2d tagTranslation = tagPose.getTranslation().toTranslation2d();
            double distance = robotPose.getTranslation().getDistance(tagTranslation);

            if (distance < minDistance) {
                minDistance = distance;
                nearestTag = tag;
            }
        }

        return nearestTag;
    }

    public static AprilTagFieldLayout adjustForBumpers(AprilTagFieldLayout originalLayout, double bumperOffsetMeters) {
        List<AprilTag> modifiedTags = new ArrayList<>();
        for (AprilTag tag : originalLayout.getTags()) {
            Pose3d originalTagPose = tag.pose;
            Pose2d tagPose2d = new Pose2d(
                    originalTagPose.getX(),
                    originalTagPose.getY(),
                    originalTagPose.getRotation().toRotation2d());
            Rotation2d desiredRobotHeading = tagPose2d.getRotation();
            Translation2d translationForBumper = new Translation2d(
                    desiredRobotHeading.getCos() * bumperOffsetMeters,  
                    desiredRobotHeading.getSin() * bumperOffsetMeters);  
            Pose2d desiredRobotPose2d = new Pose2d(
                    tagPose2d.getTranslation().plus(translationForBumper),
                    desiredRobotHeading);
            Pose3d newTagPose3d = new Pose3d(
                    desiredRobotPose2d.getX(),
                    desiredRobotPose2d.getY(),
                    originalTagPose.getZ(),
                    new Rotation3d(0, 0, desiredRobotPose2d.getRotation().getRadians()));
            modifiedTags.add(new AprilTag(tag.ID, newTagPose3d));
        }
        return new AprilTagFieldLayout(modifiedTags, originalLayout.getFieldLength(), originalLayout.getFieldWidth());
    }
}
