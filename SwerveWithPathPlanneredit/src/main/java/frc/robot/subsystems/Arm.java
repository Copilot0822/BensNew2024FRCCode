// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private final CANSparkMax leftSparkMax = new CANSparkMax(Constants.leftArmId, MotorType.kBrushless);
  private final CANSparkMax rightSparkMax = new CANSparkMax(Constants.rightArmId, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = leftSparkMax.getEncoder();
  private final DigitalInput upStop = new DigitalInput(3);
  private final DigitalInput downStop = new DigitalInput(1);
  private double setPosition = 0;
  private double error;
  private double output;
  private XboxController m_newController = new XboxController(1);
  

  


  





  public Arm() {
    leftSparkMax.setIdleMode(IdleMode.kBrake);
    rightSparkMax.setIdleMode(IdleMode.kBrake);
    leftSparkMax.setInverted(false);
    rightSparkMax.follow(leftSparkMax, true);
    leftEncoder.setPositionConversionFactor(2.04081633);//changes position to degrees of arm movement
    leftEncoder.setVelocityConversionFactor(2.04081633);
    




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
  public void addDegree(int size){//adds degree by specified amount mostly used for auto aim calibration
    if(setPosition + size <= 83.75){
      setPosition += size;
    }
    //setPosition += size;

  }
  public void removeDegree(int size){//removes degree by specified amount mostly used for auto aim calibration
    //setPosition -= size;
    if(setPosition - size >= 0){
      setPosition -= size;
    }
  }
  public void setNewPosition(double position){// sets the arm position from the bottom stop in degrees
    setPosition = position;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ArmPos", leftEncoder.getPosition());
    SmartDashboard.putBoolean("UpStop", !upStop.get());
    SmartDashboard.putBoolean("downStop", !downStop.get());
    SmartDashboard.putNumber("ArmSet pos", setPosition);
    if(!upStop.get()){
      //leftEncoder.setPosition(83.75);

    }
    else if(!downStop.get()){//if the arm is at the bottom stop then set arm pos to 0
      leftEncoder.setPosition(0);
    }
    //SmartDashboard.putNumber("Stickpos", m_newController.getRightY());

    //leftSparkMax.set(m_newController.getRightY()*-0.2);
    SmartDashboard.putNumber("output", leftSparkMax.getAppliedOutput());//for setting the arm "PID" really just P; just to see the output to keep arm up
    if(leftEncoder.getPosition() < setPosition){//if set pos is higher then current pos calculate the error and then solve for output using armslope from Constants
      error = setPosition - leftEncoder.getPosition();
      output = error*Constants.armSlope;//up calc
      if(output > Constants.maxUpAppliedPower){//arm max up output
        leftSparkMax.set(Constants.maxUpAppliedPower);
      }
      else{//else run it at slope
        leftSparkMax.set(output);

      }

    }
    if(leftEncoder.getPosition() > setPosition){//if set position is lower then the actual pos do negative PID
      error = setPosition - leftEncoder.getPosition();
      output = error*Constants.downArmSlope;//get output
      if(output < Constants.maxDownAppliedPower){// if output is above max then reduce
        leftSparkMax.set(Constants.maxDownAppliedPower);
      }
      else{// else output
        leftSparkMax.set(output);
      }
    }
    else{//for when at zero then not commanding the motor at stop
      leftSparkMax.set(0);
    }

    
    
    

    
    // This method will be called once per scheduler run
  }
  public double getArmPos(){//gets the current arm pos
    return leftEncoder.getPosition();
  }
  public double getArmVelocity(){//returns the arm speed
    return leftEncoder.getVelocity();
  }
  public double getArmSetPos(){//gets the set pos
    return setPosition;
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
