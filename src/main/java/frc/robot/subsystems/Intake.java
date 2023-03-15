//Copyright (c) FIRST and other WPILib contributors.
 //Open Source Software; you can modify and/or share it under the terms of
  //the WPILib BSD license file in the root directory of this project.

  package frc.robot.subsystems;

  import edu.wpi.first.wpilibj.PneumaticsModuleType;
  import edu.wpi.first.wpilibj.Solenoid;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.wpilibj2.command.SubsystemBase;
 
  import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
 
  import frc.robot.Constants;
  import frc.robot.Robot;
 
  public class Intake extends SubsystemBase {
    public boolean isActive = true;
    static Intake Instance = null;
    private WPI_TalonSRX intakeMotor;
    private int mode = Constants.IntakeOFF_MODE;
    private boolean deployed = false;
 
    public Intake() {
      if (isActive == false) {
        return;
      }
      intakeMotor = new WPI_TalonSRX(56);
    }
 
    public static Intake getInstance() {
      if(Instance == null) {
        synchronized (Intake.class) {
          if(Instance == null) {
            Instance = new Intake();
          }
        }
      }
      return Instance;
    }
 
    //Cone out/Cube in
    //button 7
    public void cubeIn() {
     intakeMotor.set(1);
    }
    //Cone in/Cube out
    //button 8
    public void cubeOut() {
     intakeMotor.set(-1);
    }
    //Hold cube
    public void holdCube() {
     intakeMotor.set(0);
    }
    //Hold cone
    public void holdCone() {
     intakeMotor.set(-0.25);
    }
 
    public void setIntakeMotorLevel(double mLevel) { //runs storage if intaking
      intakeMotor.set(mLevel);
    }
 
    public double getIntakeMotorLevel() {
      return intakeMotor.get();
    }
   
     // 0 = no
     // 1 = yes (forwards)
    public boolean isIntakeRunning() {
      double intakeML = getIntakeMotorLevel();
      return (intakeML < 0);
    }
 
    @Override
    public void periodic() {
       //This method will be called once per scheduler run
      SmartDashboard.putNumber("IntakeMotorLevel", getIntakeMotorLevel());
 
      
    }
  }
 