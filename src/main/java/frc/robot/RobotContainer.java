// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.IEEE754;
import frc.robot.auto.TerminalAuto;
import frc.robot.auto.TerminalTwoBallAuto;
import frc.robot.commands.climb.ZeroClimb;
import frc.robot.commands.drive.CyborgCommandToggleDrivetrainIdleMode;
import frc.robot.commands.flywheel.InstantCommandIdleFlywheel;
import frc.robot.commands.flywheel.InstantCommandToggleFlywheel;
import frc.robot.commands.intake.CyborgCommandCycleSingleBall;
import frc.robot.commands.turret.CyborgCommadSetPitch;
import frc.robot.commands.turret.CyborgCommandSetTurretPosition;
import frc.robot.commands.turret.CyborgCommandSetYawPosition;
import frc.robot.commands.turret.CyborgCommandZeroTurret;
import frc.robot.commands.turret.ToggleCommandAntiVisionAdjust;
import frc.robot.enumeration.AutonomousCommands;
import frc.robot.subsystems.SubsystemIntake;
import frc.robot.subsystems.SubsystemClimb;
import frc.robot.subsystems.SubsystemDrive;
import frc.robot.subsystems.SubsystemFlywheel;
import frc.robot.subsystems.SubsystemReceiver;
import frc.robot.subsystems.SubsystemTurret;
import frc.robot.subsystems.SubsystemUltrasonic;
import frc.robot.subsystems.SubsystemVision;
import frc.robot.util.Xbox;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Xbox Controllers Declaration and Initalization
  public static final Joystick DRIVER = new Joystick(0);
  public static final Joystick OPERATOR = new Joystick(1);
  // Subsystem Declaration and Initalization
  public static final SubsystemDrive SUB_DRIVE = new SubsystemDrive();
  public static final SubsystemClimb SUB_CLIMB = new SubsystemClimb(OPERATOR);
  public static final SubsystemTurret SUB_TURRET = new SubsystemTurret();
  public static final SubsystemIntake SUB_INTAKE = new SubsystemIntake();
  public static final SubsystemFlywheel SUB_FLYWHEEL = new SubsystemFlywheel();
  public static final SubsystemReceiver SUB_RECEIVER = new SubsystemReceiver();
  public static final SubsystemUltrasonic SUB_ULTRASONIC = new SubsystemUltrasonic();
  public static final SubsystemVision SUB_VISION = new SubsystemVision();
  // Camera Hub Declaration and Initalization
  public static final CameraHub CAMERA_HUB = new CameraHub();
  // Autonomous Command Chooser
  public static final SendableChooser<AutonomousCommands> autonomousCommandChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureDashBoard();
    configureButtonBindings();
  }

  private void configureDashBoard() {
    SmartDashboard.putData("Zero Turret", new CyborgCommandZeroTurret(SUB_TURRET));
    SmartDashboard.putData("Turret PID", new CyborgCommandSetTurretPosition(SUB_TURRET));
    // SmartDashboard.putData("Put Intake Adjust Down", new
    // CyborgCommandToggleAdjust(SUB_INTAKE, true));
    SmartDashboard.putData("Center Turret", new CyborgCommandSetYawPosition(SUB_TURRET, -53D));
    SmartDashboard.putData("Test Pitch PID", new CyborgCommadSetPitch(SUB_TURRET));
    SmartDashboard.putData("Anti Vision", new ToggleCommandAntiVisionAdjust(SUB_TURRET));
    SmartDashboard.putData("Cycle Single Ball", new CyborgCommandCycleSingleBall(SUB_INTAKE));
    SmartDashboard.putData("Zero Climber Encoder", new ZeroClimb(SUB_CLIMB));
    SmartDashboard.putBoolean("Turret PID", false);

    autonomousCommandChooser.addOption("Hanger Auto (2 Ball)", AutonomousCommands.HangerAuto);
    autonomousCommandChooser.addOption("Terminal Auto (4 Ball)", AutonomousCommands.TerminalAuto);
    autonomousCommandChooser.addOption("Terminal Two Ball (Experimental)", AutonomousCommands.TerminalTwoBall);
    SmartDashboard.putData("Autonomous Routine", autonomousCommandChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link Joystick} or {@link XboxController}), and then
   * passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton toggleTurretPID = new JoystickButton(OPERATOR, Xbox.LB);
    toggleTurretPID.toggleOnTrue(new CyborgCommandSetTurretPosition(SUB_TURRET));

    JoystickButton softFlywheelToggle = new JoystickButton(OPERATOR, Xbox.RB);
    softFlywheelToggle.toggleOnTrue(new InstantCommandIdleFlywheel(SUB_FLYWHEEL));

    JoystickButton hardFlywheelToggle = new JoystickButton(OPERATOR, Xbox.START);
    hardFlywheelToggle.onTrue(new InstantCommandToggleFlywheel(SUB_FLYWHEEL));

    JoystickButton centerTurret = new JoystickButton(OPERATOR, Xbox.BACK);
    centerTurret.toggleOnTrue(new CyborgCommandSetYawPosition(SUB_TURRET, -70D));

    JoystickButton zeroTurret = new JoystickButton(DRIVER, Xbox.BACK);
    zeroTurret.onTrue(new CyborgCommandZeroTurret(SUB_TURRET));

    JoystickButton antiVisionAdjust = new JoystickButton(OPERATOR, Xbox.B);
    antiVisionAdjust.toggleOnTrue(new ToggleCommandAntiVisionAdjust(SUB_TURRET));

    JoystickButton toggleIdleMode = new JoystickButton(DRIVER, Xbox.START);
    toggleIdleMode.onTrue(new CyborgCommandToggleDrivetrainIdleMode(SUB_DRIVE));

    // JoystickButton cycleSingleBall = new JoystickButton(OPERATOR, Xbox.A);
    // cycleSingleBall.whenPressed(new CyborgCommandCycleSingleBall(SUB_INTAKE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch (autonomousCommandChooser.getSelected()) {
      case HangerAuto:
        return new IEEE754(SUB_DRIVE, SUB_FLYWHEEL, SUB_TURRET, SUB_INTAKE, SUB_VISION).getAutonomousCommand();
      case TerminalAuto:
        return new TerminalAuto(SUB_DRIVE, SUB_FLYWHEEL, SUB_INTAKE, SUB_TURRET, SUB_ULTRASONIC, SUB_VISION)
            .getAutonomousCommand();
      case TerminalTwoBall:
        return new TerminalTwoBallAuto(SUB_DRIVE, SUB_FLYWHEEL, SUB_INTAKE, SUB_TURRET, SUB_ULTRASONIC, SUB_VISION)
            .getAutonomousCommand();
      default:
        return null;
    }
  }
}
