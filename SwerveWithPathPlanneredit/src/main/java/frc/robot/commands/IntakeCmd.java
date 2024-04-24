// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;

import com.ctre.phoenix.time.StopWatch;
 
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class IntakeCmd extends Command {
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;
  private StopWatch newTimer = new StopWatch();
  private StopWatch new2Timer = new StopWatch();
  
  
  private boolean x = false; //x is the trigger for ending the command (on true)
  private boolean y = false; //y is the backout trigger (on true)
  private boolean z = false;
  private boolean a = false;
  private boolean b = false;
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final Intake Intake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem
   * the subsystem used by this command.
   */
  public IntakeCmd(Intake intake, Indexer indexer, Shooter shooter) {
    this.indexer = indexer;
    this.intake = intake;
    this.shooter = shooter;

    //m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(indexer);
    addRequirements(shooter);
    //addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {//sets the booleans to false
    y = false;
    x = false;
    z = false;
    a = false;
    b = false;
    indexer.setIndexer(Constants.indexSpeedIn);
    indexer.startIndex3Timer();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //indexer.setIndexer(Constants.indexSpeedIn);
    //intake.setIntake(Constants.intakeSpeedIn);


    if(indexer.getIndex3Timer() > Constants.indexCurrentDelay){

      if(shooter.getShooterRPM() < 10){
        if(!x){
         intake.setIntake(Constants.intakeSpeedIn);
         indexer.setIndexer(Constants.indexSpeedIn);
         if(indexer.getIndexerCurrent() > Constants.indexCurrentThreshould){
            x = true;
            intake.setIntake(0);
            indexer.setIndexer(0);
         }
        }
        else if(x){
           if(!y){
            newTimer.start();
            indexer.setIndexer(Constants.indexSpeedIn);

            y = true;
           }
        else if(y){
          if(newTimer.getDurationMs() > 250){
            indexer.setIndexer(0);
            z = true;
          }
          if(z){
            if(!a){
              new2Timer.start();
              a = true;
              indexer.setIndexer(-Constants.indexBackOutSpeed);
              shooter.runShooter(-0.1);
            }
            if(a){
              if(new2Timer.getDurationMs() > Constants.indexBackOutTime){
                indexer.setIndexer(0);
                b = true;

              }
              else{
                indexer.setIndexer(-Constants.indexBackOutSpeed);
                shooter.runShooter(-0.1);
              }
            }

          }          
        }
      }
    }
  }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    indexer.setIndexer(0);
    intake.setIntake(0);
    shooter.runShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if(indexer.getIndexerCurrent()> Constants.indexCurrentThreshould){
      return true;
    }
    else{
      return false;
    }*/


    return b;
  }
}
