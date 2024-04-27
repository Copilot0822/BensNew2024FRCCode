// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BackPhotonVision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private final PhotonCamera backPhotonCamera = new PhotonCamera("Camera_Module_v1");
  

  


  





  public BackPhotonVision() {}

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
    var newResult = backPhotonCamera.getLatestResult();
    double x;
    int a;
    double y;
    if(newResult.hasTargets()){
      Transform3d bestToTarget = newResult.getBestTarget().getBestCameraToTarget();
      x = bestToTarget.getX();
      a = newResult.getBestTarget().getFiducialId();
      y = bestToTarget.getY();
    }
    else{
      x = -1;
      a = -1;
      y = -1;
    }

    SmartDashboard.putNumber("Distance to Tag", x);
    SmartDashboard.putNumber("Side Dist to tag", y);
    SmartDashboard.putNumber("Tag Id", a);

    // This method will be called once per scheduler run
  }
  public double getX(){
    var result = backPhotonCamera.getLatestResult();
    double x;
    if(result.hasTargets()){
      Transform3d bestToTarget = result.getBestTarget().getBestCameraToTarget();
      x = bestToTarget.getX();
    }
    else{
      x = -1;
    }
    return x;
  }
  public double getY(){
    var result = backPhotonCamera.getLatestResult();
    double x;
    if(result.hasTargets()){
      Transform3d bestToTarget = result.getBestTarget().getBestCameraToTarget();
      x = bestToTarget.getY();
    }
    else{
      x = 0;
    }
    return x;

  }
  public boolean hasTargets(){
    var result = backPhotonCamera.getLatestResult();
    
    return result.hasTargets();
  }

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
