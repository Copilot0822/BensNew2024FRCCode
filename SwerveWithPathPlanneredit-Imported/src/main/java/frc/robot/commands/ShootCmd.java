// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Arm;
//import java.util.timer;

import com.ctre.phoenix.time.StopWatch;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;
  private final Indexer m_indexer;
  private final Arm m_arm;

  private boolean x;
  private boolean y;
  public StopWatch shooterStopWatch;
  public StopWatch shooterStopWatch2;
  public StopWatch shooterWatch = new StopWatch();

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCmd(Shooter m_shooter, Indexer m_indexer, Arm m_arm) {
    //m_subsystem = subsystem;
    this.m_shooter = m_shooter;
    this.m_indexer = m_indexer;
    this.m_arm = m_arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_arm.getArmPos() > 50){
      m_shooter.runShooter(0.2);
    }
    else{
      m_shooter.runShooter(Constants.shooterSpeed);
    }
    //m_shooter.runShooter(Constants.shooterSpeed);
    m_indexer.startIndex2Timer();
    

    ///m_indexer.setIndexer(0);
    x = false;
    y = false;
    m_indexer.startIndexTimer();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //m_shooter.runShooter(Constants.shooterSpeed);
    if(m_arm.getArmPos() > 50){
      m_shooter.runShooter(0.2);
    }
    else{
      m_shooter.runShooter(Constants.shooterSpeed);
    }
    if(m_indexer.getIndexTimer() > 1000){
      m_indexer.setIndexer(Constants.indexerPush);
      //m_indexer.startIndex2Timer();
      y = true;

    }
    if(m_indexer.getIndex2Timer() > 3000){
      x = true;
    }


    
    


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.runShooter(0);
    m_indexer.setIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x;
    //return false;
  }
}
