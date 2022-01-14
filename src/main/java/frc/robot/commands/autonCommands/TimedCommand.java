/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands;

import java.util.Date;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimedCommand extends CommandBase {
  /**
   * Creates a new TimedCommand.
   */
  Date timer;
  Timer timer2;
  long startTime;
  Command command;
  long timeInMilliseconds;
  double timeInSeconds;
  long currentTime;
  public TimedCommand(Command command, long timeInMilliseconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.timeInMilliseconds = timeInMilliseconds;
    // timer2 = new Timer();
    // timer = new Date();
    this.command = command;
    // this.timeInMilliseconds = timeInMilliseconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // startTime = timer.getTime();
    startTime = System.currentTimeMillis();
    // timer2.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    System.out.println("starttime " + startTime);
    System.out.println("current time " + System.currentTimeMillis());
    System.out.println("different "+(System.currentTimeMillis()-startTime));
    currentTime = System.currentTimeMillis();
    command.schedule();
    // System.out.println("time" + timer2.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("timedcommandended. interrupted?" + interrupted);
    command.cancel();
    // command.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return timer2.hasPeriodPassed(timeInSeconds);
    // return ((timer.getTime()-startTime) >= timeInMilliseconds);
    return ((currentTime-startTime) >= timeInMilliseconds);
  }
}
