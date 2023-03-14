// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.lang.Math;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kConeAuto = "cone";
  private static final String kCubeAuto = "cube";
  private static final String kAuto = "auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are using NEO's.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkMax driveLeftSpark1 = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax driveRightSpark2 = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax driveLeftSpark11 = new CANSparkMax(11, MotorType.kBrushless);
  CANSparkMax driveRightSpark21 = new CANSparkMax(21, MotorType.kBrushless);
  RelativeEncoder m_leftDriveEncoder = driveLeftSpark1.getEncoder();
  RelativeEncoder m_rightDriveEncoder = driveRightSpark2.getEncoder();

  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(56);
  WPI_TalonSRX armMotor = new WPI_TalonSRX(8);

  SlewRateLimiter filter = new SlewRateLimiter(0.5, -0.5, 0);

  public void zeroDriveSensors() {
    //if (m_isActive == false) {
    //  return;
    //}
    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);
    // if (gyro) {
    //   Gyro.getInstance();
    //   Gyro.reset();
    // }
  }
  public double getLeftEncoder() {
    // if (m_isActive == false) {
    //   return 0;
    // }
    return m_leftDriveEncoder.getPosition() / 10.75;
  }
  public double getRightEncoder() {
    // if (m_isActive == false) {
    //   return 0;
    // }
    return m_rightDriveEncoder.getPosition() / 10.75;
  }

  public double getLeftDistanceInches() {
    return getLeftEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
  }
  public double getRightDistanceInches() {
    return getRightEncoder() * Constants.DriveBaseWheelDiameter * Math.PI;
  }

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (TalonFX, TalonSRX, Spark, VictorSP) to match your
   * robot.
   * 
   * The arm is a NEO on Everybud.
   * The intake is a NEO 550 on Everybud.
   */
  /**/// CANSparkMax arm = new CANSparkMax(5, MotorType.kBrushless);
  /**/// CANSparkMax intake = new CANSparkMax(6, MotorType.kBrushless);

  /**
   * The starter code uses the most generic joystick class.
   * 
   * The reveal video was filmed using a logitech gamepad set to
   * directinput mode (switch set to D on the bottom). You may want
   * to use the XBoxController class with the gamepad set to XInput
   * mode (switch set to X on the bottom) or a different controller
   * that you feel is more comfortable.
   */
  Joystick j = new Joystick(0);
  Joystick k = new Joystick(1);
  /*
   * Magic numbers. Use these to adjust settings.
   */

  /**
   * How many amps the arm motor can use.
   */
  static final int ARM_CURRENT_LIMIT_A = 20;

  /**
   * Percent output to run the arm up/down at
   */
  static final double ARM_OUTPUT_POWER = 0.4;

  /**
   * How many amps the intake can use while picking up
   */
  static final int INTAKE_CURRENT_LIMIT_A = 25;

  /**
   * How many amps the intake can use while holding
   */
  static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;

  /**
   * Percent output for intaking
   */
  static final double INTAKE_OUTPUT_POWER = 1.0;

  /**
   * Percent output for holding
   */
  static final double INTAKE_HOLD_POWER = 0.07;

  /**
   * Time to extend or retract arm in auto
   */
  static final double ARM_EXTEND_TIME_S = 2.0;

  /**
   * Time to throw game piece in auto
   */
  static final double AUTO_THROW_TIME_S = 0.375;

  /**
   * Time to drive back in auto
   */
  static final double AUTO_DRIVE_TIME = 6.0;

  /**
   * Speed to drive backwards in auto
   */
  static final double AUTO_DRIVE_SPEED = -0.25;

  /**
   * This method is run once when the robot is first started up.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("mobility", kAuto);
    m_chooser.addOption("cone and mobility", kConeAuto);
    m_chooser.addOption("do nothing", kNothingAuto);
    m_chooser.addOption("cube and mobility", kCubeAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    /*
     * You will need to change some of these from false to true.
     * 
     * In the setDriveMotors method, comment out all but 1 of the 4 calls
     * to the set() methods. Push the joystick forward. Reverse the motor
     * if it is going the wrong way. Repeat for the other 3 motors.
     */
    driveLeftSpark1.setCANTimeout(100);
    driveLeftSpark11.setCANTimeout(100);
    driveRightSpark2.setCANTimeout(100);
    driveRightSpark21.setCANTimeout(100);
    driveLeftSpark1.setSmartCurrentLimit(30);
    driveLeftSpark11.setSmartCurrentLimit(30);
    driveRightSpark2.setSmartCurrentLimit(30);
    driveRightSpark21.setSmartCurrentLimit(30);
    // driveLeftSpark1.setOpenLoopRampRate(0.1);
    // driveLeftSpark11.setOpenLoopRampRate(0.1);
    // driveRightSpark2.setOpenLoopRampRate(0.1);
    // driveRightSpark21.setOpenLoopRampRate(0.1);
    driveLeftSpark1.setInverted(false);
    driveLeftSpark11.setInverted(false);
    driveRightSpark2.setInverted(false);
    driveRightSpark21.setInverted(false);
    //driveLeftSpark1.
    driveLeftSpark1.burnFlash();
    driveLeftSpark11.burnFlash();
    driveRightSpark2.burnFlash();
    driveRightSpark21.burnFlash();


    /*
     * Set the arm and intake to brake mode to help hold position.
     * If either one is reversed, change that here too. Arm out is defined
     * as positive, arm in is negative.
     */
    /**///arm.setInverted(true);
    /**///arm.setIdleMode(IdleMode.kBrake);
    /**///arm.setSmartCurrentLimit(ARM_CURRENT_LIMIT_A);
    /**///intake.setInverted(false);
    /**///intake.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Calculate and set the power to apply to the left and right
   * drive motors.
   * 
   * @param forward Desired forward speed. Positive is forward.
   * @param turn    Desired turning speed. Positive is counter clockwise from
   *                above.
   */
  public void setDriveMotors(double forward, double turn) {
    SmartDashboard.putNumber("drive forward power (%)", forward);
    SmartDashboard.putNumber("drive turn power (%)", turn);

    /*
     * positive turn = counter clockwise, so the left side goes backwards
     */
    double left = forward - turn;
    double right = forward + turn;

    SmartDashboard.putNumber("drive left power (%)", left);
    SmartDashboard.putNumber("drive right power (%)", right);

    // see note above in robotInit about commenting these out one by one to set
    // directions.
    driveLeftSpark1.set(-left);
    driveLeftSpark11.set(-left);
    driveRightSpark2.set(right);
    driveRightSpark21.set(right);
  }

  /**
   * Set the arm output power. Positive is out, negative is in.
   * 
   * @param percent
   */
  public void setArmMotor(double percent) {
    /**///arm.set(percent);
    /**///SmartDashboard.putNumber("arm power (%)", percent);
    /**///SmartDashboard.putNumber("arm motor current (amps)", arm.getOutputCurrent());
    /**///SmartDashboard.putNumber("arm motor temperature (C)", arm.getMotorTemperature());
  }

  /**
   * Set the arm output power.
   * 
   * @param percent desired speed
   * @param amps current limit
   */
  public void setIntakeMotor(double percent, int amps) {
    /**///intake.set(percent);
    /**///intake.setSmartCurrentLimit(amps);
    /**///SmartDashboard.putNumber("intake power (%)", percent);
    /**///SmartDashboard.putNumber("intake motor current (amps)", intake.getOutputCurrent());
    /**///SmartDashboard.putNumber("intake motor temperature (C)", intake.getMotorTemperature());
  }

  /**
   * This method is called every 20 ms, no matter the mode. It runs after
   * the autonomous and teleop specific period methods.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }

  double autonomousStartTime;
  double autonomousIntakePower;

  @Override
  public void autonomousInit() {
    zeroDriveSensors();
    driveLeftSpark1.setIdleMode(IdleMode.kBrake);
    driveLeftSpark11.setIdleMode(IdleMode.kBrake);
    driveRightSpark2.setIdleMode(IdleMode.kBrake);
    driveRightSpark21.setIdleMode(IdleMode.kBrake);

    //armMotor.set(0.4);
    //intakeMotor.set(1);

    driveLeftSpark1.set(0.3);
    driveLeftSpark11.set(0.3);
    driveRightSpark2.set(-0.3);
    driveRightSpark21.set(-0.3);

    
        // m_autoSelected = m_chooser.getSelected();
    // System.out.println("Auto selected: " + m_autoSelected);

    // if (m_autoSelected == kConeAuto) {
                                                       //   autonomousIntakePower = INTAKE_OUTPUT_POWER;
    // } else if (m_autoSelected == kCubeAuto) {
    //   autonomousIntakePower = -INTAKE_OUTPUT_POWER;
    // }

    // autonomousStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double leftInches = Math.abs(getLeftDistanceInches());
    double rightInches = Math.abs(getRightDistanceInches());

    SmartDashboard.putNumber("LeftInches: ", leftInches);
    SmartDashboard.putNumber("RightInches: ", rightInches);

    double averageInches = (leftInches + rightInches) / 2;
    if (averageInches > 66) 
    {
      driveLeftSpark1.set(0);
      driveLeftSpark11.set(0);
      driveRightSpark2.set(0);
      driveRightSpark21.set(0);
    }
    // if (m_autoSelected == kNothingAuto) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    //   return;
    // }

    // double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    // if (timeElapsed < ARM_EXTEND_TIME_S) {
    //   setArmMotor(ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(autonomousIntakePower, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S) {
    //   setArmMotor(-ARM_OUTPUT_POWER);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // } else if (timeElapsed < ARM_EXTEND_TIME_S + AUTO_THROW_TIME_S + ARM_EXTEND_TIME_S + AUTO_DRIVE_TIME) {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(AUTO_DRIVE_SPEED, 0.0);
    // } else {
    //   setArmMotor(0.0);
    //   setIntakeMotor(0.0, INTAKE_CURRENT_LIMIT_A);
    //   setDriveMotors(0.0, 0.0);
    // }
  }

  /**
   * Used to remember the last game piece picked up to apply some holding power.
   */
  static final int CONE = 1;
  static final int CUBE = 2;
  static final int NOTHING = 3;
  int lastGamePiece;

  @Override
  public void teleopInit() {
    zeroDriveSensors();
    driveLeftSpark1.setIdleMode(IdleMode.kBrake);
    driveLeftSpark11.setIdleMode(IdleMode.kBrake);
    driveRightSpark2.setIdleMode(IdleMode.kBrake);
    driveRightSpark21.setIdleMode(IdleMode.kBrake);

    lastGamePiece = NOTHING;
  }
double armEncoder;
boolean lastPressed;
  @Override
  public void teleopPeriodic() {
    double armPower;
    if (j.getRawButton(7)) {
      // lower the arm
      armPower = -ARM_OUTPUT_POWER;
    } else if (j.getRawButton(5)) {
      // raise the arm
      armPower = ARM_OUTPUT_POWER;
    } else {
      // do nothing and let it sit where it is
      armPower = 0.0;
    }
    setArmMotor(armPower);
    boolean aVar = k.getRawButton(5);
    boolean oaVar = k.getRawButton(6);
    boolean iVar = k.getRawButton(7);
    boolean oiVar = k.getRawButton(8);

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.setNeutralMode(NeutralMode.Brake);

    if (iVar) {
      intakeMotor.set(1);
    } else if (oiVar) {
      intakeMotor.set(-1);
    } else {
      intakeMotor.set(0);
    }
    if (aVar) {
      armMotor.set(0.4);
      armEncoder = armMotor.getSelectedSensorPosition();
      SmartDashboard.putNumber("distance", armEncoder);
    } else if (oaVar) {
      armMotor.set(-0.4);
      armEncoder = armMotor.getSelectedSensorPosition();
      SmartDashboard.putNumber("distance", armEncoder);
    } else {
      armMotor.set(0);
    }
    iVar = false;
    oiVar = false;
    aVar = false;
    oaVar = false;
  
    double intakePower;
    int intakeAmps;
    if (j.getRawButton(8)) {
      // cube in or cone out
      intakePower = INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CUBE;
    } else if (j.getRawButton(6)) {
      // cone in or cube out
      intakePower = -INTAKE_OUTPUT_POWER;
      intakeAmps = INTAKE_CURRENT_LIMIT_A;
      lastGamePiece = CONE;
    } else if (lastGamePiece == CUBE) {
      intakePower = INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else if (lastGamePiece == CONE) {
      intakePower = -INTAKE_HOLD_POWER;
      intakeAmps = INTAKE_HOLD_CURRENT_LIMIT_A;
    } else {
      intakePower = 0.0;
      intakeAmps = 0;
    }
    setIntakeMotor(intakePower, intakeAmps);
 
    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     */
    double x =  -j.getRawAxis(1);
    double y =  j.getRawAxis(2);
    x = (x>0?1:-1) * x * x;
    y = (y>0?1:-1) * y * y;
    //y *= 0.7;

    //x = filter.calculate(x);

    setDriveMotors(x,y);
  }
}
