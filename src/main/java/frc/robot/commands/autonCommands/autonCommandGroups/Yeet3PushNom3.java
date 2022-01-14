/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonCommands.autonCommandGroups;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commands.autonCommands.*;
import frc.robot.sensors.Lidar;
import frc.robot.subsystems.BlasterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Yeet3PushNom3 extends SequentialCommandGroup {
  /**
   * Creates a new Yeet3PushNom3.
   */
  public Yeet3PushNom3(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem, TurretSubsystem turretSubsystem, BlasterSubsystem blasterSubsystem, Lidar lidar, ConveyorSubsystem conveyorSubsystem, XboxController joystick) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      // release control panel arm
      new WaitCommand(0),
      new RaiseIntakeArmCommand(intakeSubsystem),
      new ParallelCommandGroup(
        new TimedCommand(new ManualTurretCommand(turretSubsystem, ()->0, ()->1), 2750/1000),
        // new TimedCommand(new BlasterDistanceBasedCommand(blasterSubsystem, lidarSubsystem), 3000)
        new TimedCommand(new BlasterConstantOutputCommand(blasterSubsystem, lidar, Constants.AUTON_TARGET_CENTER_LINE_CONSTANT_VELOCITY), 2750/1000)
      ),
      
      
      // new ParallelDeadlineGroup(
      //   new WaitCommand(3), 
      //   new ManualTurretCommand(turretSubsystem, ()->0, ()->1)
      //   ),

      //   new InstantCommand(()->{turretSubsystem.setSpeed(0);}, turretSubsystem),



      // YEET 3 BALLS (PHASE 1)
      // new BackboardNearCommand(blasterSubsystem),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(4),
      //   new TurnToTargetCommand(turretSubsystem)
      // ),
     // new TimedCommand(new BlasterDistanceBasedCommand(blasterSubsystem, lidar), 2000),
     //new TimedCommand(TurnToTargetCommand(turretSubsystem, lidar), 3000),
     new TurnToTargetCommand(turretSubsystem, lidar),
      
      // new InstantCommand(()->{turretSubsystem.setSpeed(0);}, turretSubsystem),
      // new ParallelCommandGroup(
      //   new TimedCommand(new ManualTurretCommand(turretSubsystem, () -> 0, () -> 1), 5000))
      // ),

      // new ParallelDeadlineGroup(
      //   new WaitCommand(5),
      //   new BlasterDistanceBasedCommand(blasterSubsystem, lidar),
      //   new IndexToBlasterCommand(intakeSubsystem),
      //   new ConveyorCommand(conveyorSubsystem, ()->-1.0)
      // ),

      new ParallelCommandGroup(
        new TimedCommand(new BlasterDistanceBasedCommand(blasterSubsystem, lidar, joystick), 5000/1000),
        new TimedCommand(new IndexToBlasterCommand(intakeSubsystem), 5000/1000),
        new TimedCommand(new ConveyorCommand(conveyorSubsystem, ()->-1.0), 5000/1000)
      ),
      new InstantCommand(()->{blasterSubsystem.setVelocity(0);}, blasterSubsystem),
      // PUSH OTHER ROBOT OFF LINE (PHASE 2)
      new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.AUTON_PUSH_ROBOT_DISTANCE, Constants.AUTON_PUSH_ROBOT_SPEED, 0, true),
     
      // DRIVE TO OTHER BALLS (diagonally)
      new LowerIntakeArmCommand(intakeSubsystem),

      new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.YEET3PUSHNOM3_DIAG_DISTANCE_TO_TRENCH, -.75, -45, true), 
        new StartIntakeCommand(intakeSubsystem, ()->1.0)
      ),
      
      new TurnToAngleCommand(driveTrainSubsystem, .5, 0, false),

      // NOM/INTAKE 3 BALLS (FINAL PHASE) (also keep driving (parallel to balls and guardrail))
      new ParallelDeadlineGroup( // deadline because it should move on after it has reached the position
        new DriveForwardGyroDistanceCommand(driveTrainSubsystem, Constants.YEET3PUSHNOM3_3_BALL_STRAIGHT_DISTANCE, -.75, 45, true), 
        new StartIntakeCommand(intakeSubsystem, ()->1.0)
      )
      );
  }
}
