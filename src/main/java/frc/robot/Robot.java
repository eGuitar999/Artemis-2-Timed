// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import com.revrobotics.CANSparkMax;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
  private final CANSparkMax frontLeft = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rearLeft = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rearRight = new CANSparkMax(1, MotorType.kBrushless);

  private final DifferentialDrive drive = new DifferentialDrive(frontLeft, frontRight);

  private final RelativeEncoder leftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder rightEncoder = frontRight.getEncoder();
  private WPI_Pigeon2 gyro = new WPI_Pigeon2(6);
  
  private final XboxController controller = new XboxController(0);

  private PIDController pid = new PIDController(0.04, 0.005, 0);
  private double distance = 0;
  private double setpoint = 0;

  private PIDController gyroPID = new PIDController(0.04, 0.05, 0);

  private double kRevToWheel = 198/2108;
  private double kRevToFeet = 1188 * Math.PI / 25296;

//Limelight Init
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  
  @Override
  public void robotInit() {
    //Motor Controller
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    frontLeft.setInverted(true);
    
    //Drive
    drive.setMaxOutput(0.2);
    drive.setDeadband(0.1);

    //Enoder
    leftEncoder.setPositionConversionFactor(kRevToFeet);
    rightEncoder.setPositionConversionFactor(kRevToFeet);
    pid.setIntegratorRange(-0.01, 0.01);

    //Gyro
    gyroPID.setIntegratorRange(-0.5, 0.5);
    gyroPID.setTolerance(1);

  }

  @Override
  public void robotPeriodic() {
    //Encoder
    distance = (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2;
    
    //Gyro
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("Error", gyroPID.getPositionError());

    //Limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }

  @Override
  public void autonomousInit() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    gyro.setYaw(0);
  }

  @Override
  public void autonomousPeriodic() {
    //Auto-correcting
    double error = gyro.getAngle();
    drive.tankDrive(error * 0.04, -error * 0.04);
  }

  @Override
  public void teleopInit() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    gyro.setYaw(0);
  }

  @Override
  public void teleopPeriodic() {
    //Resets angle to 0
    if (controller.getLeftBumper()) {
      gyro.setYaw(0);
    }
    /* Manual PID
    double speed = -controller.getLeftY();
    double turn = controller.getRightX();
    gyroError = gyroSetpoint - gyro.getAngle();

    drive.tankDrive(speed + gyroError * gyrokP, speed - gyroError * gyrokP); 
    */
    
    //Setpoint
    if (controller.getAButtonPressed()) {
      gyroPID.setSetpoint(90);
    } else if (controller.getBButtonPressed()) {
      gyroPID.setSetpoint(0);
    }


    double angle = gyro.getAngle();
    double leftSpeed = gyroPID.calculate(angle);
    double rightSpeed = -gyroPID.calculate(angle);
    
    if (!gyroPID.atSetpoint()) {
      drive.tankDrive(leftSpeed, rightSpeed);
    } else {
      drive.tankDrive(0, 0);
    }
    
    //drive.arcadeDrive(-controller.getLeftY(), -controller.getRightX());
  }
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    setpoint = 0;
  }

  @Override
  public void testPeriodic() {

    //Setpoint
    if (controller.getAButtonPressed()) {
      setpoint = 5;
    } else if (controller.getBButtonPressed()) {
      setpoint = 0;
    }
    double speed = pid.calculate(distance, setpoint);

    //Max Speed
    if (speed > 0.2) {
      speed = 0.2;
    } else if (speed < -0.2) {
      speed = -0.2;
    }

    frontLeft.set(speed);
    frontRight.set(speed);

    /*Manual PID
    double error = setpoint - distance;

    double output = kP * error;

    frontLeft.set(output);
    frontRight.set(output);
    */
  }

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
