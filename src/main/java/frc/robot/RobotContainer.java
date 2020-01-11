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
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static frc.robot.Constants.ControllerConstants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final Joystick driverJoystick = new Joystick(ControllerConstants.Joystick_USB_Driver);

  private final DriveTrain driveTrain = new DriveTrain();
  private final LimeLight limeLight = new LimeLight();


  private final DriveWithJoystick joystickDrive = new DriveWithJoystick(driveTrain, 
                                                                        () -> {return driverJoystick.getRawAxis(1);}, 
                                                                        () -> { return driverJoystick.getRawAxis(4);});

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    PIDController pidController = new PIDController(
          DriveTrainConstants.Proportional, 
          DriveTrainConstants.Integral, 
          DriveTrainConstants.Derivative);
    pidController.setTolerance(.05);
    pidController.setIntegratorRange(-0.7, 0.7);
    new JoystickButton(driverJoystick, ControllerConstants.Green_Button_ID).whenHeld(
      new PIDCommand(
          pidController,
           limeLight::getTX, 
           0.0, 
           output -> driveTrain.arcadeDrive(0.0, -output), 
           driveTrain));
           
  }

  private void configureDefaultCommands() {
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.setDefaultCommand(driveTrain, joystickDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
