// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LeftLimelight;
import frc.robot.subsystems.PigeonAutoAim;
//import frc.robot.subsystems.LeftPhotonVision;
import frc.robot.subsystems.RightLimelight;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BackPhotonVision;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** An example command that uses an example subsystem. */
public class AutoAim extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Arm m_arm;
  private final BackPhotonVision m_photon;
  //private final LeftPhotonVision m_lPhoton;
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private boolean d = false;
  private boolean e = false;
  private int mode;
  private boolean f = false;
  private double equation;
  private final LeftLimelight m_leftLimelight;
  private final RightLimelight m_rightlimelight;
  private final PigeonAutoAim m_pigeonAutoAim;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAim(Arm m_arm, BackPhotonVision m_photon, CommandSwerveDrivetrain drivetrain, Shooter m_shooter, Indexer m_indexer, LeftLimelight m_leftLimelight, RightLimelight m_rightLimelight, PigeonAutoAim m_pigeonAutoAim) {
    //m_subsystem = subsystem;
    this.m_arm = m_arm;
    this.m_photon = m_photon;
    this.m_shooter = m_shooter;
    this.m_indexer = m_indexer;
    this.m_leftLimelight = m_leftLimelight;
    this.m_rightlimelight = m_rightLimelight;
    this.m_pigeonAutoAim = m_pigeonAutoAim;
    //this.m_lPhoton = m_lPhoton;
    //this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    addRequirements(m_arm);
    addRequirements(m_photon);
    addRequirements(drivetrain);
    addRequirements(m_shooter);
    addRequirements(m_indexer);
    addRequirements(m_leftLimelight);
    addRequirements(m_pigeonAutoAim);

    //final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_photon.startStopWatch();
    d = false;
    f = false;
    equation = -1;
    //e = false;
    m_shooter.runShooter(Constants.shooterSpeed);
    //e = !m_photon.hasTargets();
    //if(m_photon.getX() == -1 || m_photon.getX() < 1.4 && m_lPhoton.getSpeakerBool() == false){
      //e = true;

    //}
    //else{
      e = false;
    //}
    if(m_photon.getX() != -1){
      mode = 1;
    }
    else if(m_leftLimelight.getLeftBool() && m_photon.getX() == -1){
      mode = 2;
      m_pigeonAutoAim.leftSpinSet();

    }
    else if(m_rightlimelight.getRightBool() && m_photon.getX() == -1){
      mode = 3;
      m_pigeonAutoAim.rightSpinSet();
    }
    else{
      e = true;
    }
    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
    double x = m_photon.getX();
    double y = m_photon.getY();
    //final SwerveRequest.RobotCentric drive = new SwerveRequest().
    /*if(y*2 > 2){
      drivetrain.setControl(drive.withRotationalRate(2));
    } 
    else if(y*2 < -2){
      drivetrain.setControl(drive.withRotationalRate(-2));
    }
    else{
      drivetrain.setControl(drive.withRotationalRate(y*2));
    }*/

    if(mode == 2){
      drivetrain.setControl(drive.withRotationalRate(m_pigeonAutoAim.error()*3));
      if(m_photon.getX() != -1 || Units.radiansToDegrees(m_pigeonAutoAim.error()) < 5 && Units.radiansToDegrees(m_pigeonAutoAim.error()) > -5){
        mode = 1;

      }

    }
    else if(mode == 3){
      drivetrain.setControl(drive.withRotationalRate(m_pigeonAutoAim.error()*3));
      if(m_photon.getX() != -1|| Units.radiansToDegrees(m_pigeonAutoAim.error()) < 5 && Units.radiansToDegrees(m_pigeonAutoAim.error()) > -5){
        drivetrain.setControl(drive.withRotationalRate(0));
        mode = 1;
      }
    }
    else if(mode == 1){
      drivetrain.setControl(drive.withRotationalRate(y*3));
      if(f = false){
        
        f = true;
        m_shooter.runShooter(Constants.shooterSpeed);
      }
      //double g = drivetrain.getCurrentRobotChassisSpeeds(drive.RotationalRate());
      if(!d && y < 0.1 && y > -0.1){
      equation = (213 + -360*x + 196*x*x + -33.5*x*x*x);
      if(equation >= 20){
        m_arm.setNewPosition(20);
      }
      else if(equation <= 0){
        //m_arm.setNewPosition(0);
      }
      else{
        m_arm.setNewPosition(equation);
        d = true;
      }
      
    }
    if(m_shooter.getShooterRPM()> 5000 && m_arm.getArmVelocity() < 0.01 && m_arm.getArmPos() > equation - 0.5 && equation != -1 && m_arm.getArmSetPos() != 20 && m_arm.getArmPos() < m_arm.getArmSetPos()){
      m_indexer.setIndexer(Constants.indexerPush);
      //m_indexer.startIndexTimer();
    }
    //if(m_indexer.getIndexTimer() > 500){
      //e = true;

    //}


  }
    //drivetrain.setControl(drive.withRotationalRate(y*3));
    
    
    /*double equation = (15.8*x + -23.9);
    SmartDashboard.putNumber("Auto Equation", equation);
    if(equation >= 20){
      m_arm.setNewPosition(20);
    }
    else if(equation <= 0){
      //m_arm.setNewPosition(0);
    }
    else{
      m_arm.setNewPosition(equation);
    }*/
    
    
    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setNewPosition(0);
    m_indexer.setIndexer(0);
    m_shooter.runShooter(0);
    d = false; 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return !m_photon.hasTargets();
    //return false;
    return e;
  }
}
