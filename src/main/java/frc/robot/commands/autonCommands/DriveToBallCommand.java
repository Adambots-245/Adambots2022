package frc.robot.commands.autonCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.*;

public class DriveToBallCommand extends CommandBase {
    
    private NetworkTable table;
    private boolean closeToBall = false;
    private DriveTrainSubsystem driveTrainSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ConveyorSubsystem conveyorSubsystem;
    private double distanceFromIntakeArm;
    private double calculatedDistance;
    private boolean intakeDetectedBall = false;
    private PhotoEye intakePhotoEye;

    public DriveToBallCommand(DriveTrainSubsystem driveTrainSubsystem, IntakeSubsystem intakeSubsystem, ConveyorSubsystem conveyorSubsystem, PhotoEye intakePhotoEye) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.conveyorSubsystem = conveyorSubsystem;
        this.intakePhotoEye = intakePhotoEye;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(driveTrainSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        driveTrainSubsystem.resetEncoders();
        double verticalDegreesToCenter = table.getEntry("ty").getDouble(0); // robot needs to already be aligned with the ball

        // calculate distance to the ball and put it in smart dashboard
        // calculatedDistance = Constants.LIMELIGHT_HEIGHT_FROM_GROUND / Math.tan(Math.abs(verticalDegreesToCenter) * (Math.PI / 180.0) + Constants.LIMELIGHT_ANGLE_TO_HORIZONTAL * (Math.PI / 180.0));
        // distanceFromIntakeArm = calculatedDistance - Constants.LIMELIGHT_DISTANCE_TO_INTAKE_ARM;
        SmartDashboard.putNumber("Calculated Distance to Ball", calculatedDistance);
        SmartDashboard.putNumber("Distance from Intake Arm", distanceFromIntakeArm);

        // start the intake and conveyor motors so that they ball moves into the intake photo eye
        intakeSubsystem.intake(-1.0);
        conveyorSubsystem.runConveyor(0.5, true); //Set boolean to false to enable photo eyes
        conveyorSubsystem.runAlignmentBelt(0.5);
    }

    @Override
    public void execute() {
        // removed code that would continuously check the distance to a ball
        // if(distanceFromIntakeArm > Constants.ACCEPTABLE_FINAL_DISTANCE) {
        //     System.out.println("DRIVING TOWARD BALL. Distance: " + calculatedDistance);
        //     driveTrainSubsystem.arcadeDrive(-Constants.AUTON_DRIVE_FORWARD_SPEED, 0.0);
        // } else {
        //     System.out.println("CLOSE ENOUGH TO BALL");
        //     driveTrainSubsystem.arcadeDrive(0.0, 0.0);
        //     closeToBall = true;
        // }

        // move the robot forward
        driveTrainSubsystem.arcadeDrive(-Constants.AUTON_DRIVE_FORWARD_SPEED, 0.0);
        System.out.println("DriveToBallCommand.execute() just ran");
    }

    @Override
    public void end(boolean interrupted) {
        driveTrainSubsystem.arcadeDrive(0.0, 0.0);
        intakeSubsystem.intake(0.0);
        conveyorSubsystem.stopConveyorMotor();
        driveTrainSubsystem.arcadeDrive(0.0, 0.0);
        
        if(interrupted)
            System.out.println("DRIVE TO BALL COMMAND INTERRUPTED");
        else
            System.out.println("DRIVE TO BALL COMMAND ENDED");
    }

    @Override
    public boolean isFinished() {
        System.out.println("Drive encoder values:" + driveTrainSubsystem.getAverageDriveEncoderValue());
        if(driveTrainSubsystem.getAverageDriveEncoderValue() >= ( calculatedDistance * Constants.ENCODER_TICKS_PER_INCH ) + 70000)
            closeToBall = true;
        else if(intakePhotoEye.isDetecting()) {
            intakeDetectedBall = true;
        }

        System.out.println("Intake Detected Ball: " + intakeDetectedBall + ". Close to Ball: " + closeToBall);
        return closeToBall || intakeDetectedBall;
    }

}
