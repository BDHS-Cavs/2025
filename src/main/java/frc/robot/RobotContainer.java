// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.grabber;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.elevator;

import frc.robot.commands.grabber.grabberOut;
import frc.robot.commands.grabber.grabberIn;
import frc.robot.commands.grabber.grabberStop;
import frc.robot.commands.grabber.compressorEnable;
import frc.robot.commands.grabber.wristRotate;
import frc.robot.commands.grabber.wristRotateOther;

import frc.robot.commands.arm.armUp;
import frc.robot.commands.arm.armDown;
import frc.robot.commands.arm.armExtend;
import frc.robot.commands.arm.armRetract;
import frc.robot.commands.arm.armPivotStop;
import frc.robot.commands.arm.armExtensionStop;

import frc.robot.commands.elevator.elevatorDown;
import frc.robot.commands.elevator.elevatorEncoderReset;
import frc.robot.commands.elevator.elevatorUp;
import frc.robot.commands.elevator.elevatorStop;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
private final SendableChooser<Command> autoChooser;

  public final static grabber grabber = new grabber();
  public final static arm arm = new arm();
  public final static elevator elevator = new elevator();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS5Controller driverXbox = new CommandPS5Controller(0);
  final CommandXboxController controller = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getTriangleButtonPressed,
                                                                 driverXbox.getHID()::getCrossButtonPressed,
                                                                 driverXbox.getHID()::getSquareButtonPressed,
                                                                 driverXbox.getHID()::getCircleButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverXbox.getLeftY() * -1,
                                                                () -> -driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);


  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -driverXbox.getLeftY(),
                                                                   () -> -driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);

    //General NamedCommands
    NamedCommands.registerCommand("Arm Up", new armUp());
    NamedCommands.registerCommand("Arm Down", new armDown());
    NamedCommands.registerCommand("Arm Pivot Stop", new armPivotStop());
    NamedCommands.registerCommand("Arm Extend", new armExtend());
    NamedCommands.registerCommand("Arm Retract", new armRetract());
    NamedCommands.registerCommand("Arm Extension Stop", new armExtensionStop());
    NamedCommands.registerCommand("Grabber Out", new grabberOut());
    NamedCommands.registerCommand("Grabber In", new grabberIn());
    NamedCommands.registerCommand("Grabber Stop", new grabberStop());    
    NamedCommands.registerCommand("Compressor Enable", new compressorEnable());
    NamedCommands.registerCommand("Wrist Rotate", new wristRotate());
    NamedCommands.registerCommand("Elevator Up", new elevatorUp());
    NamedCommands.registerCommand("Elevator Down", new elevatorDown());
    NamedCommands.registerCommand("Elevator Stop", new elevatorStop());

    //Auto NamedCommands
    NamedCommands.registerCommand("Left AimAtReef", this.drivebase.aimAtReef(2, 'l'));
    NamedCommands.registerCommand("Center AimAtReef", this.drivebase.aimAtReef(2, 'c'));
    NamedCommands.registerCommand("Right AimAtReef", this.drivebase.aimAtReef(2, 'r'));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                driveFieldOrientedDirectAngle :
                                driveFieldOrientedDirectAngleSim);

    if (Robot.isSimulation())
    {
      driverXbox.options().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.circle().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.triangle().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.povUp().whileTrue(drivebase.centerModulesCommand());
      driverXbox.L1().onTrue(Commands.none());
      driverXbox.R1().onTrue(Commands.none());
    } else //if teleop
    {
      driverXbox.cross().onTrue((Commands.runOnce(drivebase::zeroGyro))); //makes it like drive straight if u were driving backwards
      driverXbox.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading)); //TODO not useful
      driverXbox.circle().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))) //drive to a position on the field?
                              );
      driverXbox.triangle().whileTrue(drivebase.aimAtSpeaker(2)); //uses apriltags
      driverXbox.options().whileTrue(Commands.none());
      driverXbox.povUp().whileTrue(Commands.none());
      driverXbox.L1().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly()); //makes u not move and makes u hard to push around
      driverXbox.R1().onTrue(Commands.none());

      controller.a().whileTrue(new grabberOut());                     //            A   =   Grabber Out
      controller.b().whileTrue(new grabberIn());                      //            B   =   Grabber In
      controller.x().whileTrue(new compressorEnable());               //            X   =   Enable Compressor
      controller.povUp().onTrue(new wristRotate()); // ontrue         //     D Pad Up   =   Rotate Wrist
      controller.povDown().onTrue(new wristRotateOther()); // ontrue  //   D Pad Down   =   Rotate Wrist (Other Way)
      controller.povLeft().whileTrue(new armExtend());                //   D Pad Left   =   Extend Arm
      controller.povRight().whileTrue(new armRetract());              //  D Pad Right   =   Retract Arm
      controller.rightBumper().whileTrue(new armUp());                // Right Bumper   =   Arm Up
      controller.leftBumper().whileTrue(new armDown());               //  Left Bumper   =   Arm Down
      controller.start().whileTrue(new elevatorUp());                 //        Start   =   Elevator Up
      controller.back().whileTrue(new elevatorDown());                //         Back   =   Elevator Down

      controller.leftTrigger().onTrue(new elevatorEncoderReset());    // Left Trigger   =   Elevator Encoder Reset   //ontrue   //TODO remove?

    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
//    return drivebase.getAutonomousCommand("New Auto");
return autoChooser.getSelected();
}

  public void setDriveMode()
  {
    configureBindings();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
