// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LocalADStarAK;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class FollowPathCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive m_Swerve;
  private final Pose2d m_targetPose;
  private final PathConstraints m_constraints;


  public FollowPathCommand(Drive drive, Pose2d targetPose) {
    m_Swerve = drive;
    m_targetPose = targetPose;
    m_constraints = new PathConstraints(3.5, 3,3,2);
    addRequirements(drive);
  }

  public FollowPathCommand(Drive drive, Pose2d targetPose, PathConstraints constraints) {
    m_Swerve = drive;
    m_targetPose = targetPose;
    m_constraints = constraints;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pathfinding.setPathfinder(new LocalADStarAK());
    AutoBuilder.pathfindToPose(m_targetPose, m_constraints).schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
