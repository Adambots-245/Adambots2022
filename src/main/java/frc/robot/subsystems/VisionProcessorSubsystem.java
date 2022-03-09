package frc.robot.subsystems;

import java.sql.Driver;
import java.util.ArrayList;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.vision.HubGripPipeline;
import frc.robot.vision.RedGripPipeline;
import frc.robot.vision.BlueGripPipeline;
import edu.wpi.first.cscore.*;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class VisionProcessorSubsystem extends SubsystemBase {

    private static UsbCamera ballDetectionCamera;
    private static UsbCamera hubDetectionCamera;
    private static CvSource processedOutputStreamHub;
    private static CvSource processedOutputStreamRed;
    private static CvSource processedOutputStreamBlue;
    private static CvSink hubCvSink;
    private static CvSink ballCvSink;
    private static HubGripPipeline hubGrip;
    private static RedGripPipeline redGrip;
    private static BlueGripPipeline blueGrip;
    private static Mat hubMat;
    private static Mat redMat;
    private static Mat blueMat;
    private static Point hubCrosshair;
    private static Point ballCrosshair;
    private static Point[] hubPts = new Point[4];
    private static Point[] ballPts = new Point[4];
    private static int pixelDistance;
    private static double hubAngle;
    private static double ballAngle;
    private Object lock = new Object();
    private Thread visionThread;
    private NetworkTableEntry hubAngleEntry;
    private NetworkTableEntry ballAngleEntry;
    private Solenoid ringLight;
    private MedianFilter hubMaxYFilter;
    private MedianFilter hubMinYFilter;
    private MedianFilter hubMaxXFilter;
    private MedianFilter hubMinXFilter;
    private MedianFilter ballMaxYFilter;
    private MedianFilter ballMinYFilter;
    private MedianFilter ballMaxXFilter;
    private MedianFilter ballMinXFilter;
    ArrayList<MatOfPoint> redContours;
    ArrayList<MatOfPoint> blueContours;


    public VisionProcessorSubsystem(Solenoid ringLight, HubGripPipeline hubGrip) {
        this.ringLight = ringLight;

        init();
        this.hubGrip = hubGrip;
    }

    public void init() {
        // ringLight.clearAllPCMStickyFaults();
        // if pcm status is flashing orange (sticky fault), run once
        // ringLight = new Solenoid(Constants.RING_LIGHT_PORT);
        // Creates a MedianFilter with a window size of 5 samples
        hubMaxYFilter = new MedianFilter(5);
        hubMinYFilter = new MedianFilter(5);
        hubMaxXFilter = new MedianFilter(5);
        hubMinXFilter = new MedianFilter(5);
 
        ballMaxYFilter = new MedianFilter(5);
        ballMinYFilter = new MedianFilter(5);
        ballMaxXFilter = new MedianFilter(5);
        ballMinXFilter = new MedianFilter(5);

        ringLight.set(true);
        //ballDetectionCamera = CameraServer.startAutomaticCapture(Constants.BALL_CAM_NUMBER);
        hubDetectionCamera = CameraServer.startAutomaticCapture(Constants.HUB_CAM_NUMBER);

        processedOutputStreamHub = CameraServer.putVideo("CameraHub-Output", Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
        processedOutputStreamHub.setVideoMode(PixelFormat.kGray, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.DRIVER_STATION_FPS);
        processedOutputStreamHub.setFPS(Constants.DRIVER_STATION_FPS);
        processedOutputStreamHub.setPixelFormat(PixelFormat.kGray);

        /*
        processedOutputStreamRed = CameraServer.putVideo("CameraRed-Output", Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
        processedOutputStreamRed.setVideoMode(PixelFormat.kGray, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.DRIVER_STATION_FPS);
        processedOutputStreamRed.setFPS(Constants.DRIVER_STATION_FPS);
        processedOutputStreamRed.setPixelFormat(PixelFormat.kGray);

        processedOutputStreamBlue = CameraServer.putVideo("CameraBlue-Output", Constants.IMG_WIDTH, Constants.IMG_HEIGHT);
        processedOutputStreamBlue.setVideoMode(PixelFormat.kGray, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.DRIVER_STATION_FPS);
        processedOutputStreamBlue.setFPS(Constants.DRIVER_STATION_FPS);
        processedOutputStreamBlue.setPixelFormat(PixelFormat.kGray);
        */

        hubCvSink = CameraServer.getVideo(hubDetectionCamera);
        //ballCvSink = CameraServer.getVideo(ballDetectionCamera);

        // grip = new GripPipeline();
        hubMat = new Mat();
        //redMat = new Mat();
        //blueMat = new Mat();


        //ballDetectionCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.PROCESSING_FPS);
        hubDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.PROCESSING_FPS);
        // camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG);
        // camera.setFPS(Constants.PROCESSING_FPS);
        // camera.setResolution(Constants.IMG_WIDTH, Constants.IMG_HEIGHT);

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable("Vision");
        hubAngleEntry = table.getEntry("hubAngle");
        //ballAngleEntry = table.getEntry("ballAngle");

        visionThread = new Thread(() -> {
            run();
        });

    }

    public void run() {
        // Main vision loop
        int frameCount = 0;
        while (!Thread.interrupted()) {
            hubCrosshair = null;
            ballCrosshair = null;
            if (hubCvSink.grabFrame(hubMat) == 0) {
                processedOutputStreamHub.notifyError(hubCvSink.getError());
                continue;
                /*
            }else if(ballCvSink.grabFrame(redMat) == 0) {
                processedOutputStreamRed.notifyError(ballCvSink.getError());
                continue;
            }else if(ballCvSink.grabFrame(blueMat) == 0) {
                processedOutputStreamBlue.notifyError(ballCvSink.getError());
                continue;
            */
            }
            hubGrip.process(hubMat);
            /*redGrip.process(redMat);
            blueGrip.process(blueMat);

            redContours = redGrip.filterContoursOutput();
            blueContours = blueGrip.filterContoursOutput();

            SmartDashboard.putString("Team", DriverStation.getAlliance().toString());

            RotatedRect[] rects;
            Mat properMat;
            if (DriverStation.getAlliance() == Alliance.Red) {
                 rects = findBoundingBoxes(redContours);
                 properMat = redMat;
            }
            else {
                 rects = findBoundingBoxes(blueContours);
                 properMat = blueMat;
            }
            
            //SmartDashboard.putNumber("rectslength", rects.length);
            //SmartDashboard.putNumber("redContourslength", blueContours.size());
            if (rects.length != 0) {
                //RotatedRect rect = findLargestRect(rects);
                RotatedRect rect = rects[0];
                draw(rect, properMat);
                //findCrosshair(ballPts, ballCrosshair);
                //drawCrosshair(ballCrosshair, blueMat);
            }
            */

            findBoundingBoxesHub();
            
            //findBoundingBoxesBall(redContours, redMat);



            if(hubCrosshair != null) {
                // synchronized (lock) {
                    hubAngle = calculateAngle(hubCrosshair);
                    hubAngleEntry.setDouble(hubAngle); 
                // }
            }else{
                hubAngleEntry.setDouble(Constants.ANGLE_NOT_DETECTED);
            }
            /*
            
            if(ballCrosshair != null) {
                synchronized (lock) {
                    ballAngle = calculateAngle(ballCrosshair);
                    ballAngleEntry.setDouble(ballAngle);
                }
            }else{
                b           ballAngleEntry.setDouble(Constants.ANGLE_NOT_DETECTED);
            }
            */

            SmartDashboard.putNumber("hubAngle", hubAngle);
            //SmartDashboard.putNumber("ballAngle", ballAngle);
            
            
            if (frameCount == 1) {
                processedOutputStreamHub.putFrame(hubMat);
                //processedOutputStreamRed.putFrame(redMat);
                //processedOutputStreamBlue.putFrame(blueMat);
                frameCount = 0;
            }

            frameCount++;
        }

    }

    public void findBoundingBoxesHub() {
        ArrayList<MatOfPoint> contours = hubGrip.filterContoursOutput();
        //System.out.println(contours.size());
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++)
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

        if (contours.size() != 0){
            double minX = rects[0].boundingRect().x;
            double maxX = 0;
            double minY = rects[0].boundingRect().y;
            double maxY = 0;

        for(int a=0; a<rects.length; a++ ) {
            minX = Math.min(minX, rects[a].boundingRect().x);
            maxX = Math.max(maxX, rects[a].boundingRect().x);
            minY = Math.min(minY, rects[a].boundingRect().y);
            maxY = Math.max(maxY, rects[a].boundingRect().y);


        }

        double fMinY = hubMinYFilter.calculate(minY);
        double fMaxY = hubMaxYFilter.calculate(maxY);
        double fMinX = hubMinXFilter.calculate(minX);
        double fMaxX = hubMaxXFilter.calculate(maxX);
        
        //SmartDashboard.putNumber("minX", fMinY);
        //SmartDashboard.putNumber("minY", fMaxY);
        //SmartDashboard.putNumber("maxX", fMinX);
        //SmartDashboard.putNumber("maxY", fMaxX);
        
        

        //RotatedRect boundingBox = new RotatedRect();
     
        hubPts[0] = new Point(fMinX, fMinY);
        hubPts[1] = new Point(fMaxX, fMinY);
        hubPts[2] = new Point(fMaxX, fMaxY);
        hubPts[3] = new Point(fMinX, fMaxY);


        double fieldOfView = 68.5;
        int pixelWidth = (int) (fMaxX - fMinX);
        int initialDistance = 96;
        int calculatedDistance = 0; 
        int error = 8;
        double slope = 0.125;

        
        double width = 1016; // width of the hub in mm
        //focalLength = (pixels * initialDistance) / width;
        double radVal = Math.toRadians(fieldOfView);
        double arcTanVal = Constants.IMG_HEIGHT / Constants.IMG_WIDTH;
        double cosVal = Math.cos(arcTanVal);
        double tanVal = Math.tan(radVal * cosVal);
        double angrad = Math.atan(tanVal);
        double horizontalFieldOfView = Math.toDegrees(angrad);
        // H_FOV = np.degrees(np.arctan(np.tan(np.radians(D_FOV)*np.cos(np.arctan(height/width)))))
        double focalLength = Constants.IMG_WIDTH / (2*Math.tan(Math.toRadians(horizontalFieldOfView/2)));
        //focalLength = 60; // mm https://commons.wikimedia.org/wiki/File:Microsoft_Lifecam_HD-3000_webcam.jpg
        calculatedDistance = (int) (((width * focalLength) / pixelWidth)/25.4); // in inch

        // equation for accurate distance to hub
        //double finalError = error + slope * (calculatedDistance - initialDistance);
        // =8+0.125*(A10-96)
        double a =0.00199916;
        double b = -0.450253;
        double c = 35.7879;

        double parabolaError = a * Math.pow(calculatedDistance, 2) + b * calculatedDistance + c;
        //0.00199916x^{2}-0.450253x+35.7879

        double finalDistance = calculatedDistance + parabolaError;
        
        SmartDashboard.putNumber("initial distance", calculatedDistance);
        SmartDashboard.putNumber("final distance", finalDistance);
        SmartDashboard.putNumber("focalLength", focalLength);
        SmartDashboard.putNumber("pixelWidth", pixelWidth);
        SmartDashboard.putNumber("parabola Error", parabolaError);

        //if ()
        drawRect(hubPts, hubMat);
        hubCrosshair = findCrosshair(hubPts, hubCrosshair);
        
        if (hubCrosshair != null)
            drawCrosshair(hubCrosshair, hubMat);        
        }
    }

    public RotatedRect[] findBoundingBoxes(ArrayList<MatOfPoint> contours) {
        // System.out.println(contours.size());
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++)
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

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

    public void draw(RotatedRect rect, Mat mat) {
        rect.points(ballPts);
        drawRect(ballPts, mat);
        ballCrosshair = findCrosshair(ballPts, ballCrosshair);

        if (ballCrosshair != null)
            drawCrosshair(ballCrosshair, mat);
    }

    // Draw bounding box around the reflective tape
    public void drawRect(Point[] pts, Mat mat) {
        for (int i = 0; i < 4; i++)
            Imgproc.line(mat, pts[i], pts[(i + 1) % 4], Constants.GREEN, 2);

    }

    // Calculate the crosshair position
    public Point findCrosshair(Point[] pts, Point crosshair) {
        // i is starting point for line, j is next point
        //int j;
        crosshair = new Point((pts[0].x + pts[2].x) / 2, (pts[0].y + pts[2].y) / 2);
        /*
        for (int i = 0; i < 4; i++) {
            j = (i + 1) % 4;
            if (crosshair == null || (pts[i].y + pts[j].y) / 2 < crosshair.y)
                crosshair = new Point((pts[i].x + pts[j].x) / 2, (pts[i].y + pts[j].y) / 2);
        }
        */
        return crosshair;
    }

    // Draw the crosshair on the frame
    public void drawCrosshair(Point crosshair, Mat mat) {
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y - 5), new Point(crosshair.x + 5, crosshair.y + 5), Constants.BLACK, 3);
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y + 5), new Point(crosshair.x + 5, crosshair.y - 5), Constants.BLACK, 3);

    }

    // Calculate horizontal turret angle
    public double calculateAngle(Point crosshair) {
        pixelDistance = (int) crosshair.x - Constants.IMG_HOR_MID;
        double angle = pixelDistance * Constants.HOR_DEGREES_PER_PIXEL;
        return angle;
    }

    // Getter for angle
    //public double getAngle() { 
        //return angle;
    //}

    public Thread getVisionThread() {
        return visionThread;
    }

}