// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedAutonomous extends CommandBase {
    private final DriveSubsystem m_drivetrain;
    private final Timer timer;

    /** Creates a new BetterAutonomous. */
    public TimedAutonomous(DriveSubsystem drive) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_drivetrain = drive;
        addRequirements(m_drivetrain);

        // ## Make a new timer here so we can keep track of time in this command.
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // ## Every time we make this command, reset the timer to zero and start it.
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // ## Whenever this command is scheduled (active), drive forward (y=1, x=0, r=0)
        // This is a little like driving with Joystick inputs, but without the joystick.
        m_drivetrain.Drive(1, 0, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // ## Whenever the command is stopped, make sure we also stop driving.
        timer.stop();
        m_drivetrain.Drive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // ## I didn't cover this part on Monday - but this returns false until the timer
        // passes 2 seconds, where it starts returning true.
        // Once it returns true, the command scheduler (behind the scenes) will call
        // end() for this command and stop scheduling it, finishing our autonomous action.
        return timer.get() > 2;
    }
}
