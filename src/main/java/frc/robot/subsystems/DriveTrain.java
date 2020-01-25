/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.DriveTrainConstants;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final WPI_VictorSPX leftFront = new WPI_VictorSPX(DriveTrainConstants.Left_Front_ID);
  private final WPI_VictorSPX leftMiddle = new WPI_VictorSPX(DriveTrainConstants.Left_Middle_ID);
  private final WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveTrainConstants.Left_Back_ID);

  private final WPI_VictorSPX rightFront = new WPI_VictorSPX(DriveTrainConstants.Right_Front_ID);
  private final WPI_VictorSPX rightMiddle = new WPI_VictorSPX(DriveTrainConstants.Right_Middle_ID);
  private final WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveTrainConstants.Right_Back_ID);

  private final Ultrasonic distanceSensor = new  Ultrasonic(DriveTrainConstants.Ultrasonic_Ping_ID, DriveTrainConstants.Ultrasonic_Echo_ID);

  private static NeutralMode DRIVE_NEUTRALMODE = NeutralMode.Brake;

  private DifferentialDrive drive;

  private PIDController pidController = new PIDController(0.0001, 0, 0);
  /**
   * Creates a new DriveTrain.
   */
  public DriveTrain() {
    leftFront.configFactoryDefault();
    leftMiddle.configFactoryDefault();
    leftBack.configFactoryDefault();

    rightFront.configFactoryDefault();
    rightMiddle.configFactoryDefault();
    rightBack.configFactoryDefault();

    leftFront.setNeutralMode(DRIVE_NEUTRALMODE);
    leftMiddle.setNeutralMode(DRIVE_NEUTRALMODE);
    leftBack.setNeutralMode(DRIVE_NEUTRALMODE);
    
    rightFront.setNeutralMode(DRIVE_NEUTRALMODE);
    rightMiddle.setNeutralMode(DRIVE_NEUTRALMODE);
    rightBack.setNeutralMode(DRIVE_NEUTRALMODE);

    leftFront.follow(leftBack);
    leftMiddle.follow(leftBack);

    rightFront.follow(rightBack);
    rightMiddle.follow(rightBack);

    distanceSensor.setAutomaticMode(true);
    drive = new DifferentialDrive(leftBack, rightBack);
    
  }

  public void arcadeDrive(double speed, double rotation){
    drive.arcadeDrive(speed, rotation);
  }

  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }

  public double getLeftEncoder() {
    return leftBack.getSelectedSensorPosition(0);
  }

  public double getRightEncoder() {
    return rightBack.getSelectedSensorPosition(0);
  }

  public double getRange(){
    return distanceSensor.getRangeInches();
  }

  public void resetEncoders() {
    this.rightBack.setSelectedSensorPosition(0);
    this.leftBack.setSelectedSensorPosition(0);
  }

  public double getRotation() {
    double rot = pidController.calculate(this.getLeftEncoder()+this.getRightEncoder());
    SmartDashboard.putNumber("PID Output", rot);
    return rot;
    //return pidController.calculate(this.getLeftEncoder()+this.getRightEncoder());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance", this.getRange());
    SmartDashboard.putNumber("R Encoder", this.getRightEncoder());
    SmartDashboard.putNumber("L Encoder", this.getLeftEncoder());
    SmartDashboard.putNumber("Rotion Error", this.getLeftEncoder()+this.getRightEncoder());
    // This method will be called once per scheduler run
  }
}
