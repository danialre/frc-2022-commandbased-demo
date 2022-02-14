// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
    // ## I made this class so PWM IDs on the RoboRIO are all in one place
    // (that way we don't accidentally swap or double assign motor controllers)
    public static class PWM_IDs {
        public static final int LEFT_FRONT = 1;
        public static final int RIGHT_FRONT = 2;
        public static final int LEFT_REAR = 3;
        public static final int RIGHT_REAR = 4;
    }

    // ## Let's also keep buttons in one place so it's easy to keep track of them
    public static class Buttons {
        // ## Button to move forward/repeat the Autonomous command
        public static final int FORWARD_BUTTON = 3;
        // ## Button that does nothing except schedule a useless command
        public static final int NOTHING_BUTTON = 2;
    }
}
