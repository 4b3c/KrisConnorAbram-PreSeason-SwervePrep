// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  double x;
  double y;
  double joystickAngle;
  double twist;
  double gyroPos;
  double[] balance_drive;
  double lt;
  double rt;

  @Override
  public void robotInit() {
    Map.initialAngle = Map.gyro.getYaw();
    Map.straightAngle = Map.gyro.getYaw();

    Map.drive1.setNeutralMode(NeutralMode.Brake);
    Map.drive2.setNeutralMode(NeutralMode.Brake);
    Map.drive3.setNeutralMode(NeutralMode.Brake);
    Map.drive4.setNeutralMode(NeutralMode.Brake);

    Map.rotate1.setNeutralMode(NeutralMode.Brake);
    Map.rotate2.setNeutralMode(NeutralMode.Brake);
    Map.rotate3.setNeutralMode(NeutralMode.Brake);
    Map.rotate4.setNeutralMode(NeutralMode.Brake);

    Limelight.ledOff();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    Map.elapsedTime = 0;
  }

  @Override
  public void autonomousPeriodic() {
    Autonomous.returnToOrigin();
  }

  @Override
  public void teleopInit() {
    if (Map.driver.getPOV() != -1) {
      Map.driver_mode[0] = 0;
      Map.driver_mode[1] = 1;
      Map.driver_mode[2] = 4;
    } else {
      Map.driver_mode[0] = 4;
      Map.driver_mode[1] = 5;
      Map.driver_mode[2] = 0;
    }

    Map.elapsedTime = 0;
  }

  @Override
  public void teleopPeriodic() {
    x = Map.driver.getRawAxis(Map.driver_mode[0]) / 2;
    y = Map.driver.getRawAxis(Map.driver_mode[1]) / 2;
    twist = Map.driver.getRawAxis(Map.driver_mode[2]) / 2;

    joystickAngle = 180 + (Math.atan2(y, -x) / (Math.PI) * 180);
    SmartDashboard.putNumber("Joystick Angle", joystickAngle);

    gyroPos = Map.gyro.getYaw();
    
      double cancelDB =0;
    if(Map.driver.getRawButton(2) == true){
      Vision.xOffset = Vision.tx.getDouble(0.0);

      if(Vision.tx.getDouble(0.0)<0){
        cancelDB = -.05;
      } else if (Vision.tx.getDouble(0.0)>0){
        cancelDB = .05;
      }

    }else{
      Vision.xOffset = 0;
      cancelDB = 0;
    }

    if (!Map.driver.getRawButton(5)) {
      DriveTrain.drive(Math.sqrt(x * x + y * y), (joystickAngle + Map.initialAngle - gyroPos), (twist - (Map.straightAngle - gyroPos) / 40) + (((Vision.xOffset) / 60) + cancelDB));
    } else {
      double[] pitch = {Map.gyro.getPitch() / 180, 225};
      if (pitch[0] < 0) {
        pitch[0] = -pitch[0];
        pitch[1] = 45;
      }
      double[] roll = {Map.gyro.getRoll() / 180, 135};
      if (roll[0] < 0) {
        roll[0] = -roll[0];
        roll[1] = 315;
      }
      balance_drive = DriveTrain.addArray(pitch, roll);

      DriveTrain.drive(balance_drive[0] + 0.09, balance_drive[1], 0);
    }

    if (Map.driver.getRawButton(3)) {
      Map.xOdometry = 0;
      Map.yOdometry = 0;
    }

    Odometry.calcVel(Map.initialAngle - gyroPos);
    if (Map.driver.getPOV() != -1) {
      Autonomous.returnToOrigin();
    }
    
    if (Map.driver.getRawButton(6)) {
      Map.initialAngle = gyroPos;
    }

    SmartDashboard.putBoolean("Bump Sensor", Map.bumpSensor.get());

    lt = Map.driver.getRawAxis(2);
    rt = Map.driver.getRawAxis(3);
    Map.servoMotor.set(lt - rt + 0.5);

    if (Map.driver.getRawButtonPressed(4)) {
      if (Map.LimelightLEDOn) {
        Limelight.ledOff();
        Map.LimelightLEDOn = false;
      } else {
        Limelight.ledOn();
        Map.LimelightLEDOn = true;
      }
    }
    //Toggle between pipilines.
    Vision.track();
    if (Map.driver.getRawButtonPressed(1)) {
      if (Vision.pipelineOneOn) {
        Vision.pipelineZero();
        Vision.pipelineOneOn = false;
      } else {
        Vision.pipelineOne();
        Vision.pipelineOneOn = true;
      }
    }
    
}

  

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
