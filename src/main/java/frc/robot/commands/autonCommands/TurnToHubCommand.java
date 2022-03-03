package frc.robot.commands.autonCommands;

import frc.robot.Constants;
import frc.robot.sensors.Gyro;

import java.security.KeyStore.TrustedCertificateEntry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;

/** Add your docs here. */
public class TurnToHubCommand extends CommandBase {
  double counter;
  DriveTrainSubsystem driveTrain;
  double targetAngle;
  private GyroPIDSubsystem gyroPIDSubsystem;
  private Gyro gyro;
  boolean angleBool = false;
  NetworkTable table;
  NetworkTableEntry hubAngleEntry;
  double turnAngle;

  public TurnToHubCommand(DriveTrainSubsystem inpuDriveTrain) {
    driveTrain = inpuDriveTrain;

    gyroPIDSubsystem = new GyroPIDSubsystem();
    gyro = Gyro.getInstance();

    gyroPIDSubsystem.getController().enableContinuousInput(-180, 180);

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turnAngle = 1;
    angleBool = false;
    gyro.reset();
    driveTrain.resetEncoders();
    //Gets the nework table
    counter = 0;
    targetAngle = 0;
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    table = instance.getTable("Vision");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

          hubAngleEntry = table.getEntry("hubAngle");
          targetAngle = hubAngleEntry.getDouble(Constants.ANGLE_NOT_DETECTED);  
          SmartDashboard.putNumber("hubAutonAngle", targetAngle);
          /*
          double turnSpeed = gyroPIDSubsystem.getController().calculate(gyroPIDSubsystem.getMeasurement(), turnAngle);
          turnAngle = turnAngle + 1;
          SmartDashboard.putNumber("turnAngle", turnAngle);
          
          driveTrain.arcadeDrive(0.05, 10);
          SmartDashboard.putNumber("gyroValue", gyroPIDSubsystem.getMeasurement());
          */
          driveTrain.arcadeDrive(0, 0.25);
          
          if (targetAngle != Constants.ANGLE_NOT_DETECTED) {
            angleBool = true;
            }
        
      }
      
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gyro.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return angleBool;
  }
}