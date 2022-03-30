package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj.Solenoid;

import org.opencv.core.*;

/**
 * Subsystem that only streams camera images to shuffleboard and 
 * does not process the video for any purposes.
 */
public class CameraSubsystem extends SubsystemBase {

    private static UsbCamera hubDetectionCamera;
    private static UsbCamera ballDetectionCamera;
    private static CvSink hubCvSink;
    private static CvSink ballCvSink;
    // private static Mat hubMat;
    private Thread visionThread;
    private Solenoid ringLight;

    public CameraSubsystem(Solenoid ringLight) {
        this.ringLight = ringLight;

        init();
    }

    public void init() {

        try {
            ringLight.set(false);
            hubDetectionCamera = CameraServer.startAutomaticCapture(Constants.HUB_CAM_NUMBER);
            ballDetectionCamera = CameraServer.startAutomaticCapture(Constants.BALL_CAM_NUMBER);


            hubCvSink = CameraServer.getVideo(hubDetectionCamera);
            ballCvSink = CameraServer.getVideo(ballDetectionCamera);
            hubDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, Constants.FRAME_WIDTH, Constants.FRAME_HEIGHT, Constants.PROCESSING_FPS);
            ballDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, Constants.FRAME_WIDTH, Constants.FRAME_HEIGHT, Constants.PROCESSING_FPS);
            // hubMat = new Mat();

            visionThread = new Thread(() -> {
                run();
            });
        } catch (Exception e) {
            System.out.println("Video Camera Error: " + e.getMessage());
        }
    }

    public void run() {
        // Main vision loop - not really required, but will be useful to print errors etc.
        while (!Thread.interrupted()) {
        //     if (hubCvSink.grabFrame(hubMat) == 0) {
        //         System.out.println("Error in camera server! No frames grabbed");
        //         // processedOutputStreamHub.notifyError(hubCvSink.getError());
        //         continue;
        //     }
        }
    }

    public Thread getVisionThread() {
        return visionThread;
    }
}