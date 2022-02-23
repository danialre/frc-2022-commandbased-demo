// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANSparkMaxVelocityPID;
import frc.robot.Constants;
import frc.robot.Robot;

// comment for "broken" branch

public class DriveSubsystem extends SubsystemBase {
    // @@ Use the special CANSparkMaxVelocityPID so we can simulate it (and run position PID control later)
    private final CANSparkMaxVelocityPID left_front, right_front, left_rear, right_rear;
    private final SparkMaxPIDController lf_PID, rf_PID, lr_PID, rr_PID;
    private final RelativeEncoder lf_encoder, rf_encoder, lr_encoder, rr_encoder;

    private final MecanumDrive mechdrive;

    // @@ Set velocity multiplier - let's pretend we have 6" (.1524m) diameter wheels and an 8:1 gearbox
    // because we're converting RPM to m/s
    private final double velocity_multiplier = ((0.1524 * Math.PI / (8 * 60)));
    // @@ positions are converting R (revolutions) to meters, no minute/second conversion needed
    private final double position_multiplier = ((0.1524 * Math.PI / 8));

    // @@ Set up a Field2d object so we can visualize it in the simulator/shuffleboard
    Field2d field = new Field2d();

    // @@ Wheel locations - helps keep track of where each wheel is relative to the center of the robot
    // Note that these X/Y measurements are in meters
    Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

    // @@ Kinematics object - this lets us convert wheel speeds to robot speeds and vice versa
    MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    // @@ set initial pose (position & angle of robot on field)
    Pose2d m_pose = new Pose2d(6.0, 4.0, new Rotation2d());

    // @@ Set up a gyro object (required for odometry)
    public AHRS imu = new AHRS();

    public Rotation2d getGyroAngle() {
       Rotation2d rotation = new Rotation2d(-imu.getYaw() * Math.PI / 180);
       return rotation;
    }
    
    // @@ Odometry object - keeps robot positions when given a series of wheel speeds while the robot is driving
    MecanumDriveOdometry m_odometry = new MecanumDriveOdometry(m_kinematics, getGyroAngle(), m_pose);

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // @@ Use our special velocity/position CANSparkMax class
        left_front = new CANSparkMaxVelocityPID(Constants.PWM_IDs.LEFT_FRONT, MotorType.kBrushless);
        right_front = new CANSparkMaxVelocityPID(Constants.PWM_IDs.RIGHT_FRONT, MotorType.kBrushless);
        left_rear = new CANSparkMaxVelocityPID(Constants.PWM_IDs.LEFT_REAR, MotorType.kBrushless);
        right_rear = new CANSparkMaxVelocityPID(Constants.PWM_IDs.RIGHT_REAR, MotorType.kBrushless);

        // ## Invert right sides because gearbox is flipped over vs. left side
        right_front.setInverted(true);
        right_rear.setInverted(true);

        // @@ encoder object setups
        lf_encoder = left_front.getEncoder();
        rf_encoder = right_front.getEncoder();
        lr_encoder = left_rear.getEncoder();
        rr_encoder = right_rear.getEncoder();

        // @@ PID controller object setups
        lf_PID = left_front.getPIDController();
        rf_PID = right_front.getPIDController();
        lr_PID = left_rear.getPIDController();
        rr_PID = right_rear.getPIDController();





        // @@ set conversion factors so we are always reading meters or m/s from the encoders
        lf_encoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
        rf_encoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
        lr_encoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);
        rr_encoder.setVelocityConversionFactor(Constants.DriveTrain.velocityConversionRatio);

        lf_encoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
        rf_encoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
        lr_encoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);
        rr_encoder.setPositionConversionFactor(Constants.DriveTrain.positionConversionRation);

        

        // @@ Set initial encoder positions based on where m_pose is
        // remember, individual wheel positions != robot position
        ChassisSpeeds chassisPos = new ChassisSpeeds(m_pose.getX(), m_pose.getY(), m_pose.getRotation().getRadians());
        MecanumDriveWheelSpeeds wheelPos = m_kinematics.toWheelSpeeds(chassisPos);
        lf_encoder.setPosition(wheelPos.frontLeftMetersPerSecond);
        rf_encoder.setPosition(wheelPos.frontRightMetersPerSecond);
        lr_encoder.setPosition(wheelPos.rearLeftMetersPerSecond);
        rr_encoder.setPosition(wheelPos.rearRightMetersPerSecond);

        if(Robot.isSimulation()) {
            // @@ Simulation only - set up the REV Physics simulator for closed loop testing
            REVPhysicsSim.getInstance().addSparkMax(left_front, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(right_front, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(left_rear, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(right_rear, DCMotor.getNEO(1));
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

        // @@ Simulator approximation based on rotation input - maximum 180 degrees per second, or 3.6 degrees per tick
        // this is needed because the gyro simulator doesn't update rates/angles on its own
        if(Robot.isSimulation()) {
            // gyro_sim.setAngle((r * 3.6) + gyro.getAngle());
            // gyro_sim.setRate(r * 180);
        }

        SmartDashboard.putNumber("left rear motor speed", left_rear.get());
        SmartDashboard.putNumber("left rear motor position", lr_encoder.getPosition());
    }

    public void stop() {
        mechdrive.driveCartesian(0, 0, 0);
    }

    public void changePosition(Transform2d transform) {
        // @@ little bit of kinematics library reuse - use it for positions instead of speeds
        // it's not the intended purpose, but it may work well enough
        ChassisSpeeds chassisPos = new ChassisSpeeds(transform.getX(), transform.getY(), transform.getRotation().getRadians());
        MecanumDriveWheelSpeeds wheelPos = m_kinematics.toWheelSpeeds(chassisPos);

        // @@ Now do our position math - our desired position is current position + offset given in transform
        double lf_position = lf_encoder.getPosition() + wheelPos.frontLeftMetersPerSecond;
        double rf_position = rf_encoder.getPosition() + wheelPos.frontRightMetersPerSecond;
        double lr_position = lr_encoder.getPosition() + wheelPos.rearLeftMetersPerSecond;
        double rr_position = rr_encoder.getPosition() + wheelPos.rearRightMetersPerSecond;

        // @@ Position output check
        SmartDashboard.putNumber("fl", lf_position);
        SmartDashboard.putNumber("fr", rf_position);

        // @@ Set wheel positions (current position + offset) for each wheel
        left_front.setPosition(lf_position);
        right_front.setPosition(rf_position);
        left_rear.setPosition(lr_position);
        right_rear.setPosition(rr_position);
        
        // @@ feed the mecanum drive object because we're setting 
        // motor positions directly instead of using the MecanumDrive object
        mechdrive.feed();
    }

    public Pose2d getPosition() {
        return m_pose;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // ## Note: scheduler run is every 20 milliseconds or 50 times a second

        // @@ Read wheel velocities - I noticed that getVelocity() seems to ignore
        // the conversion factors we set earlier, so we're multiplying it here again
        MecanumDriveWheelSpeeds wheelspeeds = new MecanumDriveWheelSpeeds(
            lf_encoder.getVelocity(),
            rf_encoder.getVelocity(),
            lr_encoder.getVelocity(),
            rr_encoder.getVelocity()
        );
        // @@ With our wheel speeds and our gyro rotation, update our position in the odometry object
        m_pose = m_odometry.update(getGyroAngle(), wheelspeeds);
        // @@ Update the field object for sim/shuffleboard visualization, and show our estimated positions
        field.setRobotPose(m_pose);
        SmartDashboard.putData(field);
        SmartDashboard.putNumber("est x", m_pose.getX());
        SmartDashboard.putNumber("est y", m_pose.getY());
    }

    public void simulationPeriodic() {
        // @@ Run the REV physics sim for the Spark MAX's - this only runs during robot simulation
        REVPhysicsSim.getInstance().run();
    }

    public void setPID(double kp, double ki, double kd) {
        lf_PID.setP(kp);
        rf_PID.setP(kp);
        lr_PID.setP(kp);
        rr_PID.setP(kp);

        lf_PID.setI(ki);
        rf_PID.setI(ki);
        lr_PID.setI(ki);
        rr_PID.setI(ki);

        lf_PID.setD(kd);
        rf_PID.setD(kd);
        lr_PID.setD(kd);
        rr_PID.setD(kd);
    }
}
