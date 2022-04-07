package frc.robot.commands.autonCommands;

import frc.robot.Constants;
import frc.robot.sensors.Gyro;

import java.security.KeyStore.TrustedCertificateEntry;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.GyroPIDSubsystem;

/** Add your docs here. */
public class AllignToHubCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;
  boolean atSetPoint = false;
  NetworkTable table;
  NetworkTableEntry hubAngleEntry;
  private GyroPIDSubsystem gyroPIDSubsystem;
  private double targetAngle;
  private double gyroAngle;
  private Gyro gyro;

  public AllignToHubCommand(DriveTrainSubsystem inpuDriveTrain) {
    driveTrain = inpuDriveTrain;

    
    gyroPIDSubsystem = new GyroPIDSubsystem();
    gyro = Gyro.getInstance();

    gyroPIDSubsystem.getController().enableContinuousInput(-180, 180);

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.reset();
    atSetPoint = false;
    driveTrain.resetEncoders();
    //Gets the nework table
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    table = instance.getTable(Constants.VISION_TABLE_NAME);
    hubAngleEntry = table.getEntry(Constants.HUB_ANGLE_ENTRY_NAME);
    targetAngle = hubAngleEntry.getDouble(Constants.ANGLE_NOT_DETECTED);

    if (targetAngle == Constants.ANGLE_NOT_DETECTED){
      atSetPoint = true;
    } else {
      gyroAngle = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new TurnToAngleCommand(driveTrain, Math.abs(gyroAngle - targetAngle), true);
    // gyroAngle = Math.abs(gyro.getYaw());

    // if (targetAngle > 0){
    //     driveTrain.arcadeDrive(0, 0.4);
    // }else{
    //     driveTrain.arcadeDrive(0, -0.4);
    // }
        
    // if (Math.abs(gyroAngle - targetAngle) == Constants.ANGLE_RANGE) {
    //   atSetPoint = true;
    //   // driveTrain.arcadeDrive(0, 0);
    // }
  }
      
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gyro.reset();
    driveTrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return atSetPoint;
  }
}