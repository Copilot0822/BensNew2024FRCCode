// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BackPhotonVision extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  //private final TalonSRX intakeSrx = new TalonSRX(30);
  private final PhotonCamera backPhotonCamera = new PhotonCamera("Camera_Module_v1");
  private double x;
  private double y;
  private double a;
  public final StopWatch photonStopWatch = new StopWatch();
  public final StopWatch pStopWatch = new StopWatch();
  
  

  


  





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
    SmartDashboard.putNumber("ShootTimer", pStopWatch.getDurationMs());
    
    
    
    if(newResult.hasTargets()){ // enters searching if the camera has targets else returns a,x = -1 and y = 0
      List<PhotonTrackedTarget> listOfTags = newResult.getTargets();//creates the list of Targets

      //if(DriverStation.getAlliance().get() == Alliance.Blue){
        //List<PhotonTrackedTarget> listOfTags = newResult.getTargets();
        int length = listOfTags.size();//gets the length of the List of tags
        SmartDashboard.putNumber("Number of Tags", length); // puts number of tags to dashboard
        for(int i = 0; i < length; i++){  // searches the list for ID 7 or 4 and returns the id as a and gets the x and y of the tag else returns a,x = -1 and y = 0
          Transform3d Target = listOfTags.get(i).getBestCameraToTarget();
          int b = listOfTags.get(i).getFiducialId();
          if(b == 7 || b == 4){
            a = b;
            x = Target.getX();
            y = Target.getY();
            break; 

          }
          else if(i == length -1){
            a = -1;
            x = -1;
            y = 0;
          }
        }




        /*if(newResult.getBestTarget().getFiducialId() == 7){
          Transform3d bestToTarget = newResult.getBestTarget().getBestCameraToTarget();
          x = bestToTarget.getX();
          a = newResult.getBestTarget().getFiducialId();
          y = bestToTarget.getY();
        }
        else{
          //List<PhotonTrackedTarget> listOfTags = newResult.getTargets();
          //if(listOfTags.contains())
        }*/
      //}
      //else if(DriverStation.getAlliance().get() == Alliance.Red){
        //int length = listOfTags.size()+1;
        //SmartDashboard.putNumber("Number of Tags", length);

      //}




      //if(newResult.getBestTarget().getFiducialId() == 7 && DriverStation.getAlliance().get() == Alliance.Blue){
        //Transform3d bestToTarget = newResult.getBestTarget().getBestCameraToTarget();
        //x = bestToTarget.getX();
        //a = newResult.getBestTarget().getFiducialId();
        //y = bestToTarget.getY();
      //}
      //Transform3d bestToTarget = newResult.getBestTarget().getBestCameraToTarget();
      //x = bestToTarget.getX();
      //a = newResult.getBestTarget().getFiducialId();
      //y = bestToTarget.getY();
    }
    else{
      x = -1;
      a = -1;
      y = 0;
    }

    SmartDashboard.putNumber("Distance to Tag", x);
    SmartDashboard.putNumber("Side Dist to tag", y);
    SmartDashboard.putNumber("Tag Id", a);

    //double equation = (15.8*x + -23.9);
    double equation = (213 + -360*x + 196*x*x + -33.5*x*x*x); // the auto aim equation
    SmartDashboard.putNumber("Auto Equation", equation); //writes it to dashboard
    

    // This method will be called once per scheduler run

        

  }
  public double getX(){ // gets the x value of tag **remember that x == -1 when no tag and y == 0**
    
    return x;
  }
  public double getY(){//gets the y zalue of tag **remember that y == 0 when no tag**
    
    return y;

  }
  public boolean hasTargets(){ //gets if there is targets; could also use x == -1 for this as that factors out tags that are not 4 or 7
    var result = backPhotonCamera.getLatestResult();
    
    return result.hasTargets();
  }
  public void startStopWatch(){
    pStopWatch.start();
  }
  public int getStopWatch(){
    return pStopWatch.getDurationMs();
  }
  //double equation = (15.8*x + -23.9);
  //SmartDashboard.putNumcber("Auto Equation", equation);

  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
