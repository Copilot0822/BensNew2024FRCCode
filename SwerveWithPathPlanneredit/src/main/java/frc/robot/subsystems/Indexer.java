// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private final TalonSRX indexSrx = new TalonSRX(Constants.indexerId);
  public final StopWatch indexStopWatch = new StopWatch();
  public final StopWatch indexStopWatch2 = new StopWatch();
  public final StopWatch indexStopWatch3 = new StopWatch();
  
  

  public Indexer() {
    indexSrx.setInverted(Constants.indexerInvert);
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
  public void periodic() { //printing the stator current
    //System.out.println(indexSrx.getStatorCurrent());
    SmartDashboard.putNumber("Stator Current: Indexer: ", indexSrx.getStatorCurrent());
    SmartDashboard.putNumber("Input Current: Indexer:", indexSrx.getSupplyCurrent());
    // This method will be called once per scheduler run
  }
  public void setIndexer(double speed){ //sets the speed of indexer
    indexSrx.set(ControlMode.PercentOutput, speed);
  }
  public double getIndexerCurrent(){ // used to trigger backout when note has hit shooter wheels 
    double a = indexSrx.getSupplyCurrent();

    return a;

  }
  public void startIndexTimer(){ //used in backout indexer command
    indexStopWatch.start();
  }
  public int getIndexTimer(){ // used in backout indexer command
    return indexStopWatch.getDurationMs();
  }
  public void startIndex2Timer(){
    indexStopWatch2.start();
  }
  public int getIndex2Timer(){
    return indexStopWatch2.getDurationMs();
  }
  public void startIndex3Timer(){
    indexStopWatch3.start();
  }
  public int getIndex3Timer(){
    return indexStopWatch3.getDurationMs();
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
