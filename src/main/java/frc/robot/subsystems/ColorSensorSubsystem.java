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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Map;


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

  private SuppliedValueWidget colorWidget = 
    Shuffleboard.getTab("Colors").addBoolean("WheelColor", () -> true);

  public ColorSensorSubsystem() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);

    Shuffleboard.getTab("Colors")
    .addBoolean("isBlue", () -> colorMatcher.matchClosestColor(colorSensor.getColor()).color == kBlueTarget)
    .withProperties(Map.of("colorWhenTrue", "blue", "colorWhenFalse","black"));

    Shuffleboard.getTab("Colors")
    .addBoolean("isRed", () -> colorMatcher.matchClosestColor(colorSensor.getColor()).color  == kRedTarget)
    .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse","black"));

    Shuffleboard.getTab("Colors")
    .addBoolean("isGreen", () -> colorMatcher.matchClosestColor(colorSensor.getColor()).color  == kGreenTarget)
    .withProperties(Map.of("colorWhenTrue", "green"));

    Shuffleboard.getTab("Colors")
    .addBoolean("isYellow", () -> colorMatcher.matchClosestColor(colorSensor.getColor()).color  == kYellowTarget)
    .withProperties(Map.of("colorWhenTrue", "yellow"));
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
      colorWidget.withProperties(Map.of("Color when true", "blue"));
    } else if (match.color == kRedTarget) {
      colorString = "Red";
      colorWidget.withProperties(Map.of("Color when true", "red"));
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
      colorWidget.withProperties(Map.of("Color when true", "green"));
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
      colorWidget.withProperties(Map.of("Color when true", "yellow"));
    } else {
      colorString = "Unknown";
      colorWidget.withProperties(Map.of("Color when true", "black"));
    }



    SmartDashboard.putNumber("Red", colorDetected.red);
    SmartDashboard.putNumber("Green", colorDetected.green);
    SmartDashboard.putNumber("Blue", colorDetected.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

  }
}
