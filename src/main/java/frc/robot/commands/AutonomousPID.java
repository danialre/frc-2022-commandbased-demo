// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

// @@ Autonomous using the built-in PID controller on the Spark MAXs
public class AutonomousPID extends CommandBase {
  private DriveSubsystem m_driveSubsystem;

  // @@ Set our target position for this autonomous command in absolute (field) coordinates - 10 meters X, 4 meters Y
  // this will always be the same position (except when changing alliances) no matter where the robot starts. 
  // It is always assumed that the driver station is at X=0, so 10 meters is past the halfway point
  Pose2d target_position = new Pose2d(7.0, 4.0, new Rotation2d());

  /** Creates a new AutonomousPID. */
  public AutonomousPID(DriveSubsystem subsystem) {
    m_driveSubsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.setPID(0.2, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // @@ get a Transform2d of the X/Y/R delta between robot's current position and the target position
    // this is the difference between the current position and the desired position, in meters (X/Y) & radians (R)
    Transform2d transform = target_position.minus(m_driveSubsystem.getPosition());

    // @@ simple! Change our robot position based on the previous transform data
    m_driveSubsystem.changePosition(transform);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // @@ Stop whenever we're unscheduled
    m_driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
