/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;


public class ColorSensorSubsystem extends SubsystemBase {
  /**
   * Creates a new ColorSensorSubsystem.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  private final ColorMatch colorMatcher = new ColorMatch();

  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

 
  


  public ColorSensorSubsystem() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
  }

  public int getBlue() {
    return colorSensor.getBlue();
  }

  public int getRed() {
    return colorSensor.getRed();
  }

  public int getGreen() {
    return colorSensor.getGreen();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Red Value", this.getRed());
    SmartDashboard.putNumber("Blue Value", this.getBlue());
    SmartDashboard.putNumber("Green Value", this.getGreen());

  
    Color colorDetected = colorSensor.getColor();
  
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(colorDetected);
  
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

  SmartDashboard.putNumber("Red", colorDetected.red);
  SmartDashboard.putNumber("Green", colorDetected.green);
  SmartDashboard.putNumber("Blue", colorDetected.blue);
  SmartDashboard.putNumber("Confidence", match.confidence);
  SmartDashboard.putString("Detected Color", colorString);

  }
}
