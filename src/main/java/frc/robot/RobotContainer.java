/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.subsystems.ColorSensorSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.commands.DoNothingAuto;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...\

  private final Joystick driverJoystick = new Joystick(ControllerConstants.Joystick_USB_Driver);

  private final DriveTrain driveTrain = new DriveTrain();
  private final LimeLight limeLight = new LimeLight();
  private final ColorSensorSubsystem colorSensor = new ColorSensorSubsystem();

  private final DriveWithJoystick joystickDrive = new DriveWithJoystick(driveTrain, 
                                                                        () -> {return driverJoystick.getRawAxis(1);}, 
                                                                        () -> { return driverJoystick.getRawAxis(4);});

  private final DoNothingAuto nothingAuto = new DoNothingAuto(driveTrain);

  SendableChooser<Command> chooser = new SendableChooser<>();


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
    configureAutos();
  }

  private void configureAutos() {
    chooser.addOption("Do Nothing", nothingAuto);

    Shuffleboard.getTab("Autonomous").add(chooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    PIDController firstPIDController = new PIDController(
          DriveTrainConstants.TrackingProportional, 
          DriveTrainConstants.TrackingIntegral, 
          DriveTrainConstants.TrackingDerivative);
    firstPIDController.setTolerance(.05);
    firstPIDController.setIntegratorRange(-0.7, 0.7);
    new JoystickButton(driverJoystick, ControllerConstants.Green_Button_ID).whenHeld(
      new PIDCommand(
          firstPIDController,
           limeLight::getTX, 
           0.0, 
           output -> driveTrain.arcadeDrive(0.0, -output), 
           driveTrain));


    PIDController distanceWallPIDController = new PIDController(
          DriveTrainConstants.WallProportional,
          DriveTrainConstants.WallIntegral,
          DriveTrainConstants.WallDerivative
    );
    new JoystickButton(driverJoystick, ControllerConstants.Yellow_Button_ID).whenHeld(
      new PIDCommand(
        distanceWallPIDController, 
        driveTrain::getRange, 
        10.0, 
        output -> driveTrain.arcadeDrive(driverJoystick.getRawAxis(ControllerConstants.Joystick_Left_Y_Axis), output), 
        driveTrain));

    // SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveTrainConstants.KS,
    //                                                                 DriveTrainConstants.KV,
    //                                                                 DriveTrainConstants.KA);

    // new JoystickButton(driverJoystick, ControllerConstants.Red_Button_ID).whenHeld(
    //   new TrapezoidProfileCommand(
    //             new TrapezoidProfile(
    //                 // Limit the max acceleration and velocity
    //                 new TrapezoidProfile.Constraints(
    //                  //DriveTrainConstants.KV,
    //                  //DriveTrainConstants.KA),
    //                   DriveTrainConstants.MaxSpeedMetersPerSecond,
    //                   DriveTrainConstants.MaxAccelerationMetersPerSecondSquared),
    //                 // End at desired position in meters; implicitly starts at 0
    //                 new TrapezoidProfile.State(4, 0)),
    //             // Pipe the profile state to the drive
    //             setpointState -> driveTrain.arcadeDrive(-feedforward.calculate(setpointState.velocity)/12.0, driveTrain.getRotation()),
    //             // Require the drive
    //             driveTrain).beforeStarting(() -> driveTrain.resetEncoders(), driveTrain));
  }

  private void configureDefaultCommands() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.setDefaultCommand(driveTrain, joystickDrive);
    scheduler.registerSubsystem(colorSensor);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }
}
