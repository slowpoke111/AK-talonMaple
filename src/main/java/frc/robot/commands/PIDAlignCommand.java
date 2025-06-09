package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class PIDAlignCommand extends Command {
    private final Drive m_drive;
    private final Pose2d m_targetPose;
    
    private final PIDController m_xController;
    private final PIDController m_yController;
    private final PIDController m_rotationController;
    
    private static final double POSITION_TOLERANCE = 0.1; 
    private static final double ROTATION_TOLERANCE = 2; 
    
    private static final double kP_X = 1.2;
    private static final double kI_X = 0.0;
    private static final double kD_X = 0.01;
    
    private static final double kP_Y = 1.2;
    private static final double kI_Y = 0.0;
    private static final double kD_Y = 0.0;
    
    private static final double kP_Rot = 0.7;
    private static final double kI_Rot = 0.0;
    private static final double kD_Rot = 0.1;
    
    private static final double MAX_SPEED_XY = 1.5; 
    private static final double MAX_SPEED_ROT = 12.0; 
    

    public PIDAlignCommand(Drive drive, Pose2d targetPose) {
        m_drive = drive;
        m_targetPose = targetPose;
        
        m_xController = new PIDController(kP_X, kI_X, kD_X);
        m_yController = new PIDController(kP_Y, kI_Y, kD_Y);
        m_rotationController = new PIDController(kP_Rot, kI_Rot, kD_Rot);
        
        m_xController.setTolerance(POSITION_TOLERANCE);
        m_yController.setTolerance(POSITION_TOLERANCE);
        m_rotationController.setTolerance(Math.toRadians(ROTATION_TOLERANCE));
        
        m_rotationController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();
        
        Logger.recordOutput("PIDAlign/TargetPose", m_targetPose);
    }
    
    @Override
    public void execute() {
        Pose2d currentPose = m_drive.getPose();
        
        double xError = m_targetPose.getX() - currentPose.getX();
        double yError = m_targetPose.getY() - currentPose.getY();
        double rotError = m_targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();
        
        Logger.recordOutput("PIDAlign/XError", xError);
        Logger.recordOutput("PIDAlign/YError", yError);
        Logger.recordOutput("PIDAlign/RotError", rotError);
        
        double xSpeed = m_xController.calculate(currentPose.getX(), m_targetPose.getX());
        double ySpeed = m_yController.calculate(currentPose.getY(), m_targetPose.getY());
        double rotSpeed = m_rotationController.calculate(
            currentPose.getRotation().getRadians(),
            m_targetPose.getRotation().getRadians());
        
        xSpeed = clamp(xSpeed, -MAX_SPEED_XY, MAX_SPEED_XY);
        ySpeed = clamp(ySpeed, -MAX_SPEED_XY, MAX_SPEED_XY);
        rotSpeed = clamp(rotSpeed, -MAX_SPEED_ROT, MAX_SPEED_ROT);
        
        Logger.recordOutput("PIDAlign/XSpeed", xSpeed);
        Logger.recordOutput("PIDAlign/YSpeed", ySpeed);
        Logger.recordOutput("PIDAlign/RotSpeed", rotSpeed);
        
        final double finalXSpeed = xSpeed;
        final double finalYSpeed = ySpeed;
        final double finalRotSpeed = rotSpeed;
        
        DriveCommands.joystickDrive(
            m_drive,
            () -> -finalXSpeed,  
            () -> -finalYSpeed,  
            () -> finalRotSpeed 
        ).execute();
    }
    
    @Override
    public boolean isFinished() {
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotationController.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        DriveCommands.joystickDrive(
            m_drive,
            () -> 0.0,  
            () -> 0.0,  
            () -> 0.0   
        ).execute();
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
