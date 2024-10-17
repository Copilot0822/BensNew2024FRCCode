// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private final TalonSRX elevatorTalonSRX = new TalonSRX(Constants.m_elevatorTalon);
  private final VictorSPX elevatorFollower = new VictorSPX(Constants.m_elevatorVictor);

  private final Servo ratchet = new Servo(0);
  private final DigitalInput bottomLimit = new DigitalInput(0);

  
  



  

  


  





  public Elevator() {
    elevatorTalonSRX.configSelectedFeedbackSensor(FeedbackDevice.PulseWidthEncodedPosition);

  }

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

    SmartDashboard.putNumber("Elevator Position", elevatorTalonSRX.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
