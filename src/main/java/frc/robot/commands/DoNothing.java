// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothing extends CommandBase {
    Timer timer = new Timer();

    /** Creates a new DoNothing. */
    public DoNothing() {
        // ## Do nothing for 5 seconds. This is used to demonstrate what happens when multiple commands are scheduled.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // ## Reset and start the timer.
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // ## Seriously, do nothing!
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // ## Only return true (finished) when timer gets above 5 seconds.
        return timer.get() > 5;
    }
}
