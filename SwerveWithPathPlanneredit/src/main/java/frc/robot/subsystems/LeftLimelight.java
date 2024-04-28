// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LeftLimelight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private boolean leftBool;
  

  


  





  public LeftLimelight() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    long a = NetworkTableInstance.getDefault().getTable("limelight-l").getEntry("tv").getInteger(0);
    if(a == 1){
      double b = NetworkTableInstance.getDefault().getTable("limelight-l").getEntry("tid").getDouble(0);
      if(b == 7 || b == 8 || b == 4 || b == 3){
        leftBool = true;

      }
      else{
        leftBool = false;
      }
    }
    else{
      leftBool = false;
    }
    SmartDashboard.putBoolean("LeftLime Bool", leftBool);
    // This method will be called once per scheduler run
  }
  public boolean getLeftBool(){
    return leftBool;
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
