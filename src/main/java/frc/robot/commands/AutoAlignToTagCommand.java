package frc.robot.commands;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.math.geometry.Pose3d;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.Angle;


public class AutoAlignToTagCommand extends Command {

    private final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private final Drive m_Swerve;
    private final Vision m_Vision;
    private AprilTag m_TargetTag = null;
    private Pose2d targetPose = null;
    private WrapperCommand firstAlign;
    private boolean usePID = false;
    private Timer timerPID = new Timer();
    private Timer timerTotal = new Timer();
    Command rotateToGoalHeading;
    
    public AutoAlignToTagCommand(Drive drive, Vision vision) {
        this.m_Swerve = drive;
        this.m_Vision = vision;
        addRequirements(drive, vision);
    }

    public AutoAlignToTagCommand(Drive drive, Vision vision, int ID) {
        this.m_Swerve = drive;
        this.m_Vision = vision;
        this.m_TargetTag = new AprilTag(ID,tagLayout.getTagPose(ID).isPresent() ? tagLayout.getTagPose(ID).get() : null);
        addRequirements(drive, vision);
    }

    public void initialize() {
        if (m_TargetTag == null) {
            m_TargetTag = getNearestTag();
        }
        targetPose= m_TargetTag.pose.toPose2d().plus(new Transform2d(new Translation2d(),Rotation2d.fromDegrees(180)));
        firstAlign = new FollowPathCommand(m_Swerve, targetPose).withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        Supplier<Rotation2d> rotSupplier = () -> targetPose.getRotation(); //lamdas capture vars at declaration time casuing null ptr

        rotateToGoalHeading = DriveCommands.joystickDriveAtAngle(
            m_Swerve,
            () -> 0.0, 
            () -> 0.0,
            rotSupplier
        ).withTimeout(2.0); 

        timerTotal.start();
        
    }

    public AprilTag getNearestTag() {
        Pose2d robotPose = m_Swerve.getPose();
        
        AprilTag nearestTag = null;
        double minDistance = Double.MAX_VALUE;
        
        for (AprilTag tag : tagLayout.getTags()) {
            Optional<Pose3d> tagPoseOptional = tagLayout.getTagPose(tag.ID);
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

    public void execute(){

        if (firstAlign.isFinished() ){
            usePID = true;
            timerPID.start();
        }
        if (!usePID) {
            new PIDAlignCommand(m_Swerve, new Pose2d(m_Swerve.getPose().getTranslation(),targetPose.getRotation())).withTimeout(1).withInterruptBehavior(InterruptionBehavior.kCancelSelf).andThen(firstAlign).schedule();
        }
        else {
            WrapperCommand PIDcommand = new PIDAlignCommand(m_Swerve, targetPose).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
            PIDcommand.schedule();
        }
    }

    public boolean isFinished() {
        return timerPID.hasElapsed(3) || timerTotal.hasElapsed(8);
    }

    public void end(boolean interrupted){
        timerPID.stop();
        timerPID.reset();
        usePID = false;
        m_TargetTag = null;
        targetPose = null;        
    }
}
