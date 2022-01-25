// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DoNothing;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TimedAutonomous;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    // ## My recommendation - make sections for subsystems, sections for inputs & sections for commands

    /***** SUBSYSTEMS *****/
    // ## Make our DriveSubsystem - note that this is the only place where we "make"
    // the subsystem with new DriveSubsystem()
    private final DriveSubsystem m_drivesubsystem = new DriveSubsystem();

    /***** DRIVER INPUTS *****/
    // ## Define our Joystick/controller, on port 0 (first port)
    private final PS4Controller main_joystick = new PS4Controller(0);

    // ## We can also define buttons here
    private final JoystickButton forwardButton = new JoystickButton(main_joystick, Constants.Buttons.FORWARD_BUTTON);
    private final JoystickButton nothingButton = new JoystickButton(main_joystick, Constants.Buttons.NOTHING_BUTTON);

    /***** COMMANDS *****/
    // ## Define our driving command and give it the subsystem and joystick so it can pass joystick data
    // to the subsystem
    private final DriveCommand m_DriveCommand = new DriveCommand(m_drivesubsystem, main_joystick);
    private final TimedAutonomous m_TimedAutonomous = new TimedAutonomous(m_drivesubsystem);
    private final DoNothing m_nothingCommand = new DoNothing();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // ## Set the default command of the subsystem to our driving command - when the subsystem
        // isn't called by any other command (like TimedAutonomous) run m_DriveCommand
        m_drivesubsystem.setDefaultCommand(m_DriveCommand);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // ## Here is where we map a button press to a command - whenever we press the button,
        // a new TimedAutonomous command is created and scheduled.
        forwardButton.whenPressed(m_TimedAutonomous);
        // ## Try this - in the simulator, whenever you're in Teleoperated, press the forward button
        // and the nothing button and see what happens in the scheduler pane
        // (Top menu -> NetworkTables -> LiveWindow -> Ungrouped -> Scheduler)
        nothingButton.whenPressed(m_nothingCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous

        // ## I removed the exampleCommand and put m_TimedAutonomous here - this template automatically
        // runs whatever command you return here during Autonomous and cancel's it at the start of
        // Teleoperated.
        return m_TimedAutonomous;
    }
}
