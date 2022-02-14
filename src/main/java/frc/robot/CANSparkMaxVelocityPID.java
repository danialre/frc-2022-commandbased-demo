// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

/** Add your docs here. */
public class CANSparkMaxVelocityPID extends CANSparkMax {
    // reasonable maximum velocity this should run at - this will be a little less than free speed of the motor
    // note we're not setting free speed here, because the robot has mass
    private final int MAX_VELOCITY_RPM = 4000;

    // replicate set/get API from CANSparkMax - the m_setpoint inside the CANSparkMax 
    // class is private so we can't use that one here
    private double m_setpoint = 0.0; 

    // constructor
    public CANSparkMaxVelocityPID(int deviceId, MotorType type) {
        super(deviceId, type);
    }

    @Override
    public void set(double speed) {
        if(Math.abs(speed) > 1) {
            // limit magnitude to [-1, +1]
            speed = (speed > 0 ? 1.0 : -1.0);
        }
        // update the setpoint
        m_setpoint = speed;
        // set velocity as percentage of maximum velocity (instead of duty cycle)
        // there's not really a practical reason for this here, but since the
        // REV library only supports kVelocity and kVoltage for simulation we're
        // using that for now
        SparkMaxPIDController pid = getPIDController();
        pid.setReference(speed * MAX_VELOCITY_RPM, ControlType.kVelocity);
    }

    public void setPosition(double position) {
        // similar to set(), except we set position instead of velocity/duty cycle
        SparkMaxPIDController pid = getPIDController();
        pid.setReference(position, ControlType.kPosition);
    }

    @Override
    public double get() {
        // just get the setpoint (-1 to +1) - positions, etc. can be grabbed from the encoder class
        return m_setpoint;
    }
}
