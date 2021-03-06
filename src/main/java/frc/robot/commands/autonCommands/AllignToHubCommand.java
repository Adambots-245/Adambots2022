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
  boolean angleBool = false;
  NetworkTable table;
  NetworkTableEntry hubAngleEntry;
  private GyroPIDSubsystem gyroPIDSubsystem;
  private double targetAngle;
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
    angleBool = false;
    driveTrain.resetEncoders();
    //Gets the nework table
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    table = instance.getTable("Vision");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
          hubAngleEntry = table.getEntry("hubAngle");
          targetAngle = hubAngleEntry.getDouble(Constants.ANGLE_NOT_DETECTED);  
          
          double turnSpeed = gyroPIDSubsystem.getController().calculate(gyroPIDSubsystem.getMeasurement(), targetAngle);
          driveTrain.arcadeDrive(Constants.HUB_TURN_SPEED, turnSpeed);
          
          if (targetAngle < Constants.ANGLE_RANGE && targetAngle > -(Constants.ANGLE_RANGE)) {
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