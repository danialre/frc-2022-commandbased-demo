// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
    DriveSubsystem m_drivesubsystem;
    // ## Used the PS4Controller (instead of Joystick) because I wanted getLeftX() and getRightX() methods
    // (instead of generic getRawAxis() and guessing the axis IDs)
    XboxController drivejoystick;

    double linearMultiplier = 0.04;
    double angularMultiplier = 0.01;

    /** Creates a new DriveCommand. */
    public DriveCommand(DriveSubsystem subsystem, XboxController joystick) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivesubsystem = subsystem;
        drivejoystick = joystick;

        addRequirements(m_drivesubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // ## Let's see when this pops up in the robot log
        System.out.println("drive command initialized!!");
        m_drivesubsystem.setPID(0.03, 0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    // Hee Hee Hoo Hoo
    @Override
    public void execute() {
        // ## Here, we access the joystick's axis methods to read driver inputs
        double y = -drivejoystick.getLeftY() * linearMultiplier; // ## Remember, pressing up is negative, down is positive
        double x = drivejoystick.getLeftX() * linearMultiplier;
        double r = drivejoystick.getRightX() * angularMultiplier;
        SmartDashboard.putNumber( "JoyY", y);
        SmartDashboard.putNumber("JoyX", x);
        SmartDashboard.putNumber("JoyR", r);
        // ## And here is where we give them to the subsystem
        m_drivesubsystem.Drive(y, x, r);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // ## Stop the robot from moving whenever it is disabled
        m_drivesubsystem.Drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
