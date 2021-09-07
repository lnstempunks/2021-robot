// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.ComplexDrivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveLimeLight;
import frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Subsystems
	public static ComplexDrivetrain drivetrain = new ComplexDrivetrain();
	public static Shooting shooter = new Shooting();
	public static Index index = new Index();
	public static Intake intake = new Intake();
	private DigitalInput m_limitSwitch = new DigitalInput(1);
	private boolean limelightOn = false;

	private boolean intakeOn = false;
	// Controllers
	public static Joystick joystick = new Joystick(0);
	public static JoystickButton circle = new JoystickButton(joystick, 3);
	public static JoystickButton leftTrigger = new JoystickButton(joystick, 7);
	public static JoystickButton rightTrigger = new JoystickButton(joystick, 8);
	public static JoystickButton xbutton = new JoystickButton(joystick, 2);
	public static JoystickButton slowTurn = new JoystickButton(joystick, 12);
	// public static DriveToDistance driveToDistance = new DriveToDistance(10,
	// drivetrain);
	public static DriveLimeLight driveLimelight = new DriveLimeLight(drivetrain);
	public static AutoCommands autoCommands = new AutoCommands(drivetrain);

  private RunCommand m_curvatureDrive = new RunCommand(() -> {
		if (slowTurn.get()) {
			drivetrain.curvatureDrive(((joystick.getRawAxis(1)) / 3), ((joystick.getRawAxis(2)) / 3));
		} else {
			drivetrain.curvatureDrive(((joystick.getRawAxis(1)) / 1.5), ((joystick.getRawAxis(2) / 2.5)));
		}
  }, drivetrain);
  
  private RunCommand runIndexer = new RunCommand(() -> {
		rightTrigger.whenReleased(new InstantCommand(() -> index.stopIndexer(), index));
		if (rightTrigger.get()) {
			index.runIndexer();
		} else if (circle.get()) {
			index.reverse();
		} else if (m_limitSwitch.get()) {
			index.stopIndexer();
		}
  }, index);
  
  private RunCommand runShooter = new RunCommand(() -> {
		if (leftTrigger.get()) {
			shooter.shootLimeLight();
		} else if (xbutton.get()) {
			shooter.shoot(-0.75);
		} else {
			shooter.stopShooting();
		}
	}, shooter);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick, 4).whenPressed(new InstantCommand(() -> {
			if (intakeOn == true) {
				intake.stopIntake();
				index.stopIndexer();
				intakeOn = false;
			} else {
				intake.runIntake();
				// index.runIndexer();
				intakeOn = true;
			}

    }, intake, index));
    
		// X (on ps4) -
		new JoystickButton(joystick, 1).toggleWhenPressed(new RunCommand(() -> {
			if (limelightOn == false){
				limelightOn = true;
				driveLimelight.execute();
			} else {
				limelightOn = false;
				driveLimelight.stopLimeLight();
			}
		}, drivetrain));

		new JoystickButton(joystick, 12).whenPressed(new RunCommand(() -> {                         // Radius, degrees
			autoCommands.turnWithRadius(65, 170);
		}, drivetrain));

		new JoystickButton(joystick, 12).whenReleased(new RunCommand(() -> {    
			autoCommands.stopMoving();
		}, drivetrain));
		
		// Togle lights - BUTTON->
		new JoystickButton(joystick, 13).whenPressed(new InstantCommand(() -> {
			NetworkTable m_limelight = NetworkTableInstance.getDefault().getTable("limelight");
			NetworkTableEntry ledMode = m_limelight.getEntry("ledMode");
			if (ledMode.getDouble(0) == 0) {
				ledMode.setDouble(1); // Force OFF
			} else if (ledMode.getDouble(0) == 1) {
				ledMode.setDouble(3); // Force ON
			} else if (ledMode.getDouble(0) == 3) {
				ledMode.setDouble(1); // Force OFF
			} else {
				ledMode.setDouble(1); // Force OFF by default
			}
		}));
  }

  private void configureDefaultCommands() {
	drivetrain.setDefaultCommand(m_curvatureDrive);
	// colorsensor.setDefaultCommand(colorSensor);
	shooter.setDefaultCommand(runShooter);
	// intake.setDefaultCommand(runBoth);
	// driveLimelight.setDefaultCommand(runLimelight);
	index.setDefaultCommand(runIndexer);
	}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SequentialCommandGroup(new ParallelCommandGroup(new RunCommand(() -> {
			shooter.shoot(-0.54);
		}, shooter), new RunCommand(() -> {
			index.runIndexer();
		}, index)).withTimeout(10), new RunCommand(() -> {
			drivetrain.curvatureDrive(-0.25, 0);
    }, drivetrain).withTimeout(1)).withTimeout(15);
  }
}
