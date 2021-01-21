package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.lib.utils.PunkSparkMax;
import com.revrobotics.AlternateEncoderType;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.ControlType;

public class ComplexDrivetrain extends SubsystemBase {
        private PunkSparkMax m_leftFront, m_leftBack, m_rightFront, m_rightBack;

	private SpeedControllerGroup m_left, m_right;

	private final DifferentialDrive m_drive;
	private final DifferentialDriveKinematics m_kinematics;
	private final DifferentialDriveOdometry m_odometry;

	private final PIDController m_leftPIDController, m_rightPIDController;
	private CANPIDController m_leftPID, m_rightPID;

	// private DoubleSolenoid m_gearShift;

	private CANEncoder m_leftEncoder, m_rightEncoder;

	private PigeonIMU m_gyro;

        private double[] heading;
        /**
	 * Central class for all drivetrain-related activities.
	 */
	public ComplexDrivetrain() {
		m_leftFront = new PunkSparkMax(Constants.Drivetrain.kmLeftFront, Constants.kDriveMotorConfig);
		m_rightFront = new PunkSparkMax(Constants.Drivetrain.kmRightFront, Constants.kRightDriveConfig);
		m_leftBack = new PunkSparkMax(Constants.Drivetrain.kmLeftBack, Constants.kDriveMotorConfig, m_leftFront);
		m_rightBack = new PunkSparkMax(Constants.Drivetrain.kmRightBack, Constants.kRightDriveConfig, m_rightFront);
		//m_rightFront.setInverted(true);
		//m_rightBack.setInverted(true);
		m_left = new SpeedControllerGroup(m_leftFront, m_leftBack);
		m_right = new SpeedControllerGroup(m_rightFront, m_rightBack);
		m_right.setInverted(true);
		m_gyro = new PigeonIMU(0);
		heading = new double[3];

		m_drive = new DifferentialDrive(m_left, m_right);
		m_kinematics = new DifferentialDriveKinematics(Constants.Drivetrain.kTrackWidth);
		m_odometry = new DifferentialDriveOdometry(getAngle());

		m_leftPIDController = new PIDController(1, 0.0015, 0);
		m_rightPIDController = new PIDController(0.95, 0.001, 0);
		m_leftPIDController.setTolerance(0.01, 0.005);
		m_rightPIDController.setTolerance(0.01, 0.005);

		// m_gearShift = new DoubleSolenoid(Constants.kGearShift[0], Constants.kGearShift[1]);
		m_leftEncoder = m_leftBack.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
		m_rightEncoder = m_rightBack.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
		m_leftEncoder.setPosition(0);
		m_rightEncoder.setPosition(0);
		m_leftEncoder.setInverted(true);
		m_leftPID = m_leftFront.getPIDController();
		m_rightPID = m_rightFront.getPIDController();
		m_leftPID.setFeedbackDevice(m_leftEncoder);
		m_rightPID.setFeedbackDevice(m_rightEncoder);

		// TEST Constructor - Not sure if this works.
		// m_leftPID = new PunkPIDController(m_leftFront, Constants.Drivetrain.kPIDConfig, true);
		// m_rightPID = new PunkPIDController(m_rightFront, Constants.Drivetrain.kPIDConfig, true);

		m_leftPID.setP(0.2);
		m_leftPID.setI(0);
		m_leftPID.setD(0);
		m_leftPID.setIZone(0);
		m_leftPID.setFF(0);
		m_leftPID.setOutputRange(-1, 1);
		m_rightPID.setP(0.1);
		m_rightPID.setI(1e-4);
		m_rightPID.setD(1);
		m_rightPID.setIZone(0);
		m_rightPID.setFF(0);
		m_rightPID.setOutputRange(-1, 1);
		m_gyro.configFactoryDefault();

		SmartDashboard.putData("Drivetrain - Left PID", m_leftPIDController);
		SmartDashboard.putData("Drivetrain - Right PID", m_rightPIDController);
		
        }

	/**
	 * Basic drivetrain operation, with no feed-forward control.
	 * 
	 * @param xSpeed    Speed along the x-axis (Forward is positive)
	 * @param zRotation Rotation rate for robot (Clockwise is positive)
	 */
	SlewRateLimiter filter = new SlewRateLimiter(0.9);
	public void curvatureDrive(double xSpeed, double zRotation) {
		m_drive.curvatureDrive(filter.calculate(xSpeed), zRotation * -1, true);
        }
        
        /**
	 * Set right motor group speed.
	 * @param speed Speed to set motor group to.
	 */
	public void setRight(double speed) {
		m_right.set(speed);
	}

	/**
	 * Set left motor group speed.
	 * @param speed Speed to set motor group to.
	 */
	public void setLeft(double speed) {
		m_left.set(speed);
	}

	/**
	 * Shifts the pancake motors on each of the gearboxes.
	 * 
	 * @param value State to reach (Can be forward, reverse, or off.)
	 */
	public void shiftGears(Value value) {
		// m_gearShift.set(value);
	}

	/**
	 * Retrieves the z-axis rotation from the PigeonIMU.
	 * 
	 * @return Current rotation around z-axis (yaw).
	 */

	public Rotation2d getAngle() {
		m_gyro.getYawPitchRoll(heading);
		return Rotation2d.fromDegrees(heading[0]);
	}

	/**
	 * Updates the odometry object with the latest sensor data.
	 */
	private void updateOdometry() {
		m_odometry.update(getAngle(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
	}

	/**
	 * Retrieves the displacement (in feet) on the left side of the drivetrain.
	 * 
	 * @return Left-side displacement
	 */
	public double getLeftDistance() {
		return (m_leftEncoder.getPosition() * (Math.PI * 6)) / 12;
	}

	/**
	 * Retrieves the displacement (in feet) on the right side of the drivetrain.
	 * 
	 * @return Right-side displacement
	 */
	public double getRightDistance() {
		return (m_rightEncoder.getPosition() * (Math.PI * 6)) / 12;
	}

	/**
	 * Retrieves the average displacement (in meters)between each side of the
	 * drivetrain.
	 * 
	 * @return Average displacement between each encoder.
	 */
	public double getDistance() {
		double avgDist = ((getLeftDistance() + getRightDistance()) / 2);
		SmartDashboard.putNumber("avgDist", avgDist);
		return avgDist;
	}
}
