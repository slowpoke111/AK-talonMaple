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
    private Command secondAlign;
    private Command thirdAlign;
    private Command fullAlign;

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
        if (m_TargetTag != null) {
            Logger.recordOutput("AutoAlignToReefCommand/TargetTagID", m_TargetTag.ID);
            targetPose = m_TargetTag.pose.toPose2d()
                    .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
            Logger.recordOutput("AutoAlignToReefCommand/TargetPose", targetPose);
        } else {
            Logger.recordOutput("AutoAlignToReefCommand/TargetTagID", -1); // Indicate no tag found
            Logger.recordOutput("AutoAlignToReefCommand/TargetPose", new Pose2d()); // Log default pose
            // Handle case where no tag is found, perhaps by ending the command
            DriverStation.reportError("AutoAlignToReefCommand: No target AprilTag found.", false);
            cancel(); // End the command if no tag is found
            return;
        }
        
        firstAlign = new PIDAlignCommand(m_Swerve, new Pose2d(m_Swerve.getPose().getTranslation(), targetPose.getRotation()))
                .withTimeout(1)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        secondAlign = new FollowPathCommand(m_Swerve, targetPose)
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        thirdAlign = new PIDAlignCommand(m_Swerve, targetPose)
                .withTimeout(2)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        
        fullAlign = firstAlign.andThen(secondAlign).andThen(thirdAlign)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
        

        timerPID.reset();
        timerTotal.reset();
        timerTotal.start();

        fullAlign.schedule();
        Logger.recordOutput("AutoAlignToReefCommand/Initialized", true);
    }

    public void execute() {
        Logger.recordOutput("AutoAlignToReefCommand/TimerPID", timerPID.get());
        Logger.recordOutput("AutoAlignToReefCommand/TimerTotal", timerTotal.get());
        if (fullAlign != null) {
            Logger.recordOutput("AutoAlignToReefCommand/FullAlignIsScheduled", fullAlign.isScheduled());
            Logger.recordOutput("AutoAlignToReefCommand/FullAlignIsFinished", fullAlign.isFinished());
        }
    }

    public boolean isFinished() {
        boolean fullAlignDone = fullAlign != null && fullAlign.isFinished();
        boolean timerElapsed = timerTotal.hasElapsed(8);
        boolean result = fullAlignDone || timerElapsed;

        Logger.recordOutput("AutoAlignToReefCommand/IsFinished/FullAlignDone", fullAlignDone);
        Logger.recordOutput("AutoAlignToReefCommand/IsFinished/TimerElapsed", timerElapsed);
        Logger.recordOutput("AutoAlignToReefCommand/IsFinished/Result", result);
        
        return result;
    }

    public void end(boolean interrupted) {
        if (fullAlign != null) {
            fullAlign.cancel();
        }
        
        timerPID.stop();
        timerPID.reset();
        timerTotal.stop();
        timerTotal.reset();
        
        m_TargetTag = null;
        targetPose = null;
        Logger.recordOutput("AutoAlignToReefCommand/EndedInterrupted", interrupted);
        Logger.recordOutput("AutoAlignToReefCommand/Initialized", false);
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
