package frc.robot.subsystems;

import java.util.ArrayList;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.HubGripPipeline;
import frc.robot.vision.RedGripPipeline;
import frc.robot.vision.BlueGripPipeline;
import edu.wpi.first.cscore.*;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class VisionProcessorSubsystem extends SubsystemBase {

    private static UsbCamera ballDetectionCamera;
    private static UsbCamera hubDetectionCamera;
    private static CvSource processedOutputStreamHub;
    private static CvSource processedOutputStreamRed;
    private static CvSource processedOutputStreamBlue;
    private static CvSink cvSink;
    private static HubGripPipeline hubGrip;
    private static RedGripPipeline redGrip;
    private static BlueGripPipeline blueGrip;
    private static Mat mat;
    private static Point crosshair;
    private static Point[] pts = new Point[4];
    private static int pixelDistance;
    private static double angle;
    private Object lock = new Object();
    private Thread visionThread;
    private NetworkTableEntry angleEntry;
    private Solenoid ringLight;

    public VisionProcessorSubsystem(Solenoid ringLight, RedGripPipeline redGrip, HubGripPipeline hubGrip, BlueGripPipeline blueGrip) {
        this.ringLight = ringLight;

        init();
        this.redGrip = redGrip;
        this.hubGrip = hubGrip;
        this.blueGrip = blueGrip;
    }

    public void init() {
        // ringLight.clearAllPCMStickyFaults();
        // if pcm status is flashing orange (sticky fault), run once
        // ringLight = new Solenoid(Constants.RING_LIGHT_PORT);
        ringLight.set(true);
        ballDetectionCamera = CameraServer.startAutomaticCapture(Constants.BALL_CAM_NUMBER);
        hubDetectionCamera = CameraServer.startAutomaticCapture(Constants.HUB_CAM_NUMBER);
        // camera.setExposureManual(Constants.CAM_EXPOSURE);
        processedOutputStreamHub = CameraServer.putVideo("CameraHub-Output", Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
        processedOutputStreamHub.setVideoMode(PixelFormat.kGray, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.DRIVER_STATION_FPS);
        processedOutputStreamHub.setFPS(Constants.DRIVER_STATION_FPS);
        processedOutputStreamHub.setPixelFormat(PixelFormat.kGray);

        processedOutputStreamRed = CameraServer.putVideo("CameraRed-Output", Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
        processedOutputStreamRed.setVideoMode(PixelFormat.kGray, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.DRIVER_STATION_FPS);
        processedOutputStreamRed.setFPS(Constants.DRIVER_STATION_FPS);
        processedOutputStreamRed.setPixelFormat(PixelFormat.kGray);

        processedOutputStreamBlue = CameraServer.putVideo("CameraBlue-Output", Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
        processedOutputStreamBlue.setVideoMode(PixelFormat.kGray, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.DRIVER_STATION_FPS);
        processedOutputStreamBlue.setFPS(Constants.DRIVER_STATION_FPS);
        processedOutputStreamBlue.setPixelFormat(PixelFormat.kGray);

        cvSink = CameraServer.getVideo();
        // grip = new GripPipeline();
        mat = new Mat();

        ballDetectionCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.PROCESSING_FPS);
        hubDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.PROCESSING_FPS);
        // camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);
        // camera.setFPS(Constants.PROCESSING_FPS);
        // camera.setResolution(Constants.IMG_WIDTH, Constants.IMG_HEIGHT);

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("Vision");
        angleEntry = table.getEntry("Angle");

        visionThread = new Thread(() -> {
            run();
        });

    }

    public void run() {
        // Main vision loop
        int frameCount = 0;
        while (!Thread.interrupted()) {
            crosshair = null;
            if (cvSink.grabFrame(mat) == 0) {
                processedOutputStreamRed.notifyError(cvSink.getError());
                continue;

            }

            hubGrip.process(mat);
            //redGrip.process(mat);
            //blueGrip.process(mat);

        /*    RotatedRect[] rects = findBoundingBoxes();
            if (rects.length != 0) {
              RotatedRect rect = findLargestRect(rects);
              draw(rect);
            }

           
            RotatedRect[] rects = findBoundingBoxes();
            if (rects.length != 0) {
                for (int i = 0; i < rects.length; i++) {
                    draw(rects[i]);
                    
                }
                RotatedRect rect = findLargestRect(rects);
                draw(rect);
              }

               */
            RotatedRect[] rects = findBoundingBoxes();
            


            if (crosshair != null) {
                synchronized (lock) {
                    calculateAngle();

                }
                
            }
            
            if (frameCount == 1) {
                processedOutputStreamHub.putFrame(mat);
                //processedOutputStreamRed.putFrame(redGrip.hsvThresholdOutput());
                //processedOutputStreamBlue.putFrame(blueGrip.hsvThresholdOutput());
                frameCount = 0;
            }

            frameCount++;
        }

    }

    public RotatedRect[] findBoundingBoxes() {
        ArrayList<MatOfPoint> contours = hubGrip.filterContoursOutput();
        //System.out.println(contours.size());
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++)
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

        /*System.out.print(rects.length);
        SmartDashboard.putNumber("rect0 x", rects[0].boundingRect().x);
        SmartDashboard.putNumber("rect last x-1", rects[rects.length - 1].boundingRect().x);
        SmartDashboard.putNumber("rect0 y", rects[0].boundingRect().y);
        SmartDashboard.putNumber("rect last y-1", rects[rects.length - 1].boundingRect().y);
*/
        int minX = rects[0].boundingRect().x;
        int maxX = 0;
        int minY = rects[0].boundingRect().y;
        int maxY = 0;

        for(int a=0; a==rects.length; a++ ) {
            minX = Math.min(minX, rects[a].boundingRect().x);
            maxX = Math.max(maxX, rects[a].boundingRect().x);
            minY = Math.min(minY, rects[a].boundingRect().y);
            maxY = Math.max(maxY, rects[a].boundingRect().y);


        }
        SmartDashboard.putNumber("minX", minX);
        SmartDashboard.putNumber("minY", minY);
        SmartDashboard.putNumber("maxX", maxX);
        SmartDashboard.putNumber("maxY", maxY);
        

        RotatedRect boundingBox = new RotatedRect();

        boundingBox.boundingRect().x = minX;
        boundingBox.boundingRect().y = minY;
        boundingBox.boundingRect().width = maxX - minX;
        boundingBox.boundingRect().height = maxY - minY;

        draw(boundingBox);
           

        return rects;

    }

    public RotatedRect findLargestRect(RotatedRect[] rects) {
        RotatedRect rect = rects[0];
        for (int i = 0; i < rects.length; i++) {
            if (rects[i].size.area() > rect.size.area())
                rect = rects[i];

        }


        return rect;
    }

    public void draw(RotatedRect rect) {

        rect.points(pts);
        
        drawRect(pts);
        findCrosshair(pts);

        if (crosshair != null)
            drawCrosshair();

    }

    // Draw bounding box around the reflective tape
    public void drawRect(Point[] pts) {
        for (int i = 0; i < 4; i++)
            Imgproc.line(mat, pts[i], pts[(i + 1) % 4], Constants.GREEN, 1);

    }

    // Calculate the crosshair position
    public void findCrosshair(Point[] pts) {
        // i is starting point for line, j is next point
        int j;
        for (int i = 0; i < 4; i++) {
            j = (i + 1) % 4;
            if (crosshair == null || (pts[i].y + pts[j].y) / 2 < crosshair.y)
                crosshair = new Point((pts[i].x + pts[j].x) / 2, (pts[i].y + pts[j].y) / 2);

        }

    }

    // Draw the crosshair on the frame
    public void drawCrosshair() {
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y - 5), new Point(crosshair.x + 5, crosshair.y + 5), Constants.RED, 3);
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y + 5), new Point(crosshair.x + 5, crosshair.y - 5), Constants.RED, 3);

    }

    // Calculate horizontal turret angle
    public void calculateAngle() {
        pixelDistance = (int) crosshair.x - Constants.IMG_HOR_MID;
        angle = pixelDistance * Constants.HOR_DEGREES_PER_PIXEL;
        angleEntry.setDouble(angle);

    }

    // Getter for angle
    public double getAngle() { 
        return angle;
    }

    public Thread getVisionThread() {
        return visionThread;
    }

}