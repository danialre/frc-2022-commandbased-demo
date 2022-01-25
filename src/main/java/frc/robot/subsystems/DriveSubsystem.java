// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {

    // ## I am defining these as MotorController objects, not PWMSparkMax or CANSparkMax.
    // I'll explain why in the class constructor.
    private final MotorController left_front, right_front, left_rear, right_rear;

    private final MecanumDrive mechdrive;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // ## As I said before, the PWM Spark Max's work in the simulator, but the team's plan
        // is to use CAN Spark Max's on the actual robot so we can access Encoder data.
        // However, the CAN Spark Max's don't work in the simulator.
        //
        // Here is a way around this: use PWM Spark Max objects, whenever we're simulating,
        // otherwise use CAN Spark Max objects.
        if (Robot.isSimulation()) {
            // ## Returns true if simulated - opposite of Robot.isReal()
            left_front = new PWMSparkMax(1);
            right_front = new PWMSparkMax(2);
            // ## PWM IDs 3 and 4 are used from Constants to demonstrate keeping various IDs and
            // robot-specific mappings in one place - functionally, they're the same as
            // defining the port like 1 and 2 above
            left_rear = new PWMSparkMax(Constants.PWM_IDs.LEFT_REAR);
            right_rear = new PWMSparkMax(Constants.PWM_IDs.RIGHT_REAR);
        } else {
            // ## Returns true if this is the real robot

            // ## Notice how left_front, etc. are in both if and else statements - also
            // notice how it can accept both a new PWMSparkMax() and a new CANSparkMax() !
            // This is because the Spark Max classes inherit the MotorController class,
            // which is a common "ancestor" to both of them.
            left_front = new CANSparkMax(1, MotorType.kBrushless);
            right_front = new CANSparkMax(2, MotorType.kBrushless);
            left_rear = new CANSparkMax(3, MotorType.kBrushless);
            right_rear = new CANSparkMax(4, MotorType.kBrushless);
        }
        // ## Remember: Ctrl-Shift-P -> WPILib: Simulate Robot Code to begin your sim

        // ## Also set up the MecanumDrive object by giving it the motor controllers.
        mechdrive = new MecanumDrive(left_front, left_rear, right_front, right_rear);
    }

    // ## Here is a method we made that doesn't exist in the Subsystem template. We can call
    // this in any command that uses this subsystem.
    // In this case, we get X (left/right), Y (forward/back), and R (rotation) from whatever
    // calls Drive() and then pass it to the MecanumDrive object.
    public void Drive(double y, double x, double r) {
        // ## Use the driveCartesian() method given to us in the Mecanum drive object.
        // This handles all the driving math for us.
        mechdrive.driveCartesian(y, x, r);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // ## Note: scheduler run is every 20 milliseconds or 50 times a second
    }
}
