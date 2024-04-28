// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PigeonAutoAim extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private final Pigeon2 thePigeon2 = new Pigeon2(60);
  private double desiredAngle;
  
  

  


  





  public PigeonAutoAim() {}

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
    SmartDashboard.putNumber("Rotation", thePigeon2.getAngle());
    // This method will be called once per scheduler run
  }
  public void leftSpinSet(){
    desiredAngle = thePigeon2.getAngle()-90;

  }
  public double error(){
    return Units.degreesToRadians(thePigeon2.getAngle() - desiredAngle);
  }
  public void rightSpinSet(){
    desiredAngle = thePigeon2.getAngle()+90;
  }
  /*public double errorRight(){
    return Units.degreesToRadians(thePigeon2.getAngle() - desiredAngle);
  }*/

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}