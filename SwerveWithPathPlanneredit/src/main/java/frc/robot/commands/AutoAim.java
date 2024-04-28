// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.BackPhotonVision;
import frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** An example command that uses an example subsystem. */
public class AutoAim extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;
  private final Arm m_arm;
  private final BackPhotonVision m_photon;
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private boolean d = false;
  private boolean e = false;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoAim(Arm m_arm, BackPhotonVision m_photon, CommandSwerveDrivetrain drivetrain, Shooter m_shooter, Indexer m_indexer) {
    //m_subsystem = subsystem;
    this.m_arm = m_arm;
    this.m_photon = m_photon;
    this.m_shooter = m_shooter;
    this.m_indexer = m_indexer;
    //this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(subsystem);
    addRequirements(m_arm);
    addRequirements(m_photon);
    addRequirements(drivetrain);
    addRequirements(m_shooter);
    addRequirements(m_shooter);

    //final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      //.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      
      //.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_photon.startStopWatch();
    d = false;
    //e = false;
    m_shooter.runShooter(Constants.shooterSpeed);
    //e = !m_photon.hasTargets();
    if(m_photon.getX() == -1 || m_photon.getX() < 1.4){
      e = true;

    }
    else{
      e = false;
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
    drivetrain.setControl(drive.withRotationalRate(y*3));
    
    
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
    if(m_photon.getStopWatch() > 500 && !d){
      double equation = (15.8*x + -23.9);
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
    if(m_photon.getStopWatch() > 800){
      m_indexer.setIndexer(Constants.indexerPush);
    }
    if(m_photon.getStopWatch() > 1400){
      e = true;

    }

    
    
    

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
