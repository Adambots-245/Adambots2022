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
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class VisionProcessorSubsystem extends SubsystemBase {

    /**
     *
     */
    private static final int SAMPLE_SIZE = 5;
    private static UsbCamera ballDetectionCamera;
    private static UsbCamera hubDetectionCamera;
    private static CvSource processedOutputStreamHub;
    private static CvSource processedOutputStreamRed;
    private static CvSource processedOutputStreamBlue;
    private static CvSink hubCvSink;
    private static CvSink ballCvSink;
    private static HubGripPipeline hubGrip;
    // private static RedGripPipeline redGrip;
    // private static BlueGripPipeline blueGrip;
    private static Mat hubVideoFrame;
    // private static Mat redMat;
    // private static Mat blueMat;
    private static Point hubCrosshair;
    private static Point ballCrosshair;
    // private static Point[] hubPts = new Point[4];
    private static Point[] ballPts = new Point[4];
    private static int pixelDistance;
    private static double hubAngle;
    // private static double ballAngle;
    // private Object lock = new Object();
    private Thread visionThread;
    private NetworkTableEntry hubAngleEntry;
    // private NetworkTableEntry ballAngleEntry;
    private Solenoid ringLight;
    private MedianFilter hubMaxYFilter;
    private MedianFilter hubMinYFilter;
    private MedianFilter hubMaxXFilter;
    private MedianFilter hubMinXFilter;
    // private MedianFilter ballMaxYFilter;
    // private MedianFilter ballMinYFilter;
    // private MedianFilter ballMaxXFilter;
    // private MedianFilter ballMinXFilter;
    // ArrayList<MatOfPoint> redContours;
    // ArrayList<MatOfPoint> blueContours;
    private static double focalLength = 0;


    public VisionProcessorSubsystem(Solenoid ringLight, HubGripPipeline hubGripPipeline) {
        this.ringLight = ringLight;

        init();
        hubGrip = hubGripPipeline;
    }

    public void init() {
        // Creates a MedianFilter with a window size of 5 samples
        // Used to avoid flickering of detected objects when distance is large
        hubMaxYFilter = new MedianFilter(SAMPLE_SIZE);
        hubMinYFilter = new MedianFilter(SAMPLE_SIZE);
        hubMaxXFilter = new MedianFilter(SAMPLE_SIZE);
        hubMinXFilter = new MedianFilter(SAMPLE_SIZE);
 
        // ballMaxYFilter = new MedianFilter(5);
        // ballMinYFilter = new MedianFilter(5);
        // ballMaxXFilter = new MedianFilter(5);
        // ballMinXFilter = new MedianFilter(5);

        ringLight.set(true);
        //ballDetectionCamera = CameraServer.startAutomaticCapture(Constants.BALL_CAM_NUMBER);
        hubDetectionCamera = CameraServer.startAutomaticCapture(Constants.HUB_CAM_NUMBER);
        hubDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, Constants.FRAME_WIDTH, Constants.FRAME_HEIGHT, Constants.PROCESSING_FPS);
        hubDetectionCamera.setExposureManual(Constants.HUB_CAMERA_EXPOSURE); //10% exposure

        processedOutputStreamHub = CameraServer.putVideo("CameraHub-Output", Constants.FRAME_WIDTH, Constants.FRAME_HEIGHT);
        processedOutputStreamHub.setVideoMode(PixelFormat.kGray, Constants.FRAME_WIDTH, Constants.FRAME_HEIGHT, Constants.DRIVER_STATION_FPS);
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

        hubVideoFrame = new Mat();
        //redMat = new Mat();
        //blueMat = new Mat();

        //ballDetectionCamera.setVideoMode(VideoMode.PixelFormat.kMJPEG, Constants.IMG_WIDTH, Constants.IMG_HEIGHT, Constants.PROCESSING_FPS);

        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable(Constants.VISION_TABLE_NAME);
        hubAngleEntry = table.getEntry(Constants.HUB_ANGLE_ENTRY_NAME);
        //ballAngleEntry = table.getEntry("ballAngle");

        visionThread = new Thread(() -> {
            run();
        });

    }

    public void run() {
        // Main vision loop
        while (!Thread.interrupted()) {
            hubCrosshair = null;
            ballCrosshair = null;
            if (hubCvSink.grabFrame(hubVideoFrame) == 0) {
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
            hubGrip.process(hubVideoFrame);
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

            Point[] hubBounds = findBoundingBoxesHub(hubVideoFrame, hubGrip.filterContoursOutput());

            if (hubBounds != null){
                double finalDistance = findDistance(hubBounds[0].x, hubBounds[1].x);
                  
                NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
                NetworkTableEntry teDist = ntinst.getTable(Constants.VISION_TABLE_NAME).getEntry(Constants.HUB_DISTANCE_ENTRY_NAME);
                teDist.setDouble(finalDistance);
                SmartDashboard.putNumber(Constants.HUB_DISTANCE_ENTRY_NAME, finalDistance);
                
                drawRect(hubBounds, hubVideoFrame);
                Point ch = findCrosshair(hubBounds);
                
                NetworkTableEntry teAngle = ntinst.getTable(Constants.VISION_TABLE_NAME).getEntry(Constants.HUB_ANGLE_ENTRY_NAME);
                double hubAngle = Constants.ANGLE_NOT_DETECTED; // 600 is angle not found
                
                if (ch != null){
                    drawCrosshair(ch, hubVideoFrame);
                    
                    hubAngle = calculateAngle(ch);
                    teAngle.setDouble(hubAngle); 
                }
                SmartDashboard.putNumber(Constants.HUB_ANGLE_ENTRY_NAME, hubAngle);
                
                int location = 0;
                Imgproc.putText(hubVideoFrame, String.format("Angle: %.2f Deg", hubAngle), new Point(0, location = location + 30), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2); 
                Imgproc.putText(hubVideoFrame, String.format("Dist: %.2f\"", finalDistance), new Point(hubVideoFrame.width()-225, 25), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 0, 255), 2); 
            }
            
            //findBoundingBoxesBall(redContours, redMat);

            /*
            
            if(ballCrosshair != null) {
                synchronized (lock) {
                    ballAngle = calculateAngle(ballCrosshair);
                    ballAngleEntry.setDouble(ballAngle);
                }
            }else{
                ballAngleEntry.setDouble(Constants.ANGLE_NOT_DETECTED);
            }
            */

            //SmartDashboard.putNumber("ballAngle", ballAngle);
            
            processedOutputStreamHub.putFrame(hubVideoFrame);
            //processedOutputStreamRed.putFrame(redMat);
            //processedOutputStreamBlue.putFrame(blueMat);
        }
    }

    // Calculate the crosshair (center) position
    public Point findCrosshair(Point[] pts) {
        Point crosshair = new Point((pts[0].x + pts[2].x) / 2, (pts[0].y + pts[2].y) / 2);
        
        return crosshair;
      }

    public Point[] findBoundingBoxesHub(Mat videoFrame, ArrayList<MatOfPoint> contours) {
        // ArrayList<MatOfPoint> contours = hubGrip.filterContoursOutput();
        //System.out.println(contours.size());
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++)
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

        Point framePts[] = null;

        if (contours.size() != 0){
            double minX = rects[0].boundingRect().x;
            double maxX = 0;
            double minY = rects[0].boundingRect().y;
            double maxY = 0;

            for(int a=0; a<rects.length; a++ ) {
                Point[] ppts = new Point[4];
                rects[a].points(ppts);
                drawRect(ppts, videoFrame, new Scalar(0, 0, 255)); //red

                minX = Math.min(minX, rects[a].boundingRect().x);
                maxX = Math.max(maxX, rects[a].boundingRect().x + rects[a].boundingRect().width);
                minY = Math.min(minY, rects[a].boundingRect().y);
                maxY = Math.max(maxY, rects[a].boundingRect().y + rects[a].boundingRect().height);
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

            framePts = new Point[4];
     
            framePts[0] = new Point(fMinX, fMinY);
            framePts[1] = new Point(fMaxX, fMinY);
            framePts[2] = new Point(fMaxX, fMaxY);
            framePts[3] = new Point(fMinX, fMaxY);
        }

        return framePts;
    }

    private double findDistance(double fMinX, double fMaxX) {
        int pixelWidth = (int) (fMaxX - fMinX);
        int calculatedDistance = 0; 

        double width = 1016; // width of the hub in mm
        
        double focalLength = findFocalLength();
        // System.out.println("Focal Length: " + focalLength);
  
        //focalLength = 60; // mm https://commons.wikimedia.org/wiki/File:Microsoft_Lifecam_HD-3000_webcam.jpg
        // focalLength = (pixels * initialDistance) / width;

        calculatedDistance = (int) (((width * focalLength) / pixelWidth)/25.4); // in inch (25.4 mm per inch)
  
        // equation for accurate distance to hub
        // Get error values from various distances and use https://www.dcode.fr/function-equation-finder to find an equation
        //Parabola Equation: 0.00199916 * (distance ^2) + (-0.450253 * distance) + 35.7879
        double a = 0.00199916;
        double b = -0.450253;
        double c = 35.7879;
  
        // double parabolaError = a * Math.pow(calculatedDistance, 2) + b * calculatedDistance + c;
        double parabolaError = a * (calculatedDistance * calculatedDistance) + (b * calculatedDistance) + c;
        
  
        double finalDistance = calculatedDistance + parabolaError;
        return finalDistance;
    }

    private double findFocalLength(){
        
        if (focalLength == 0){
            // Adjust field of view for the camera type - this is for Microsoft Lifecam HD-3000
            double fieldOfView = Constants.CAMERA_FOV; //Source: https://dl2jx7zfbtwvr.cloudfront.net/specsheets/WEBC1010.pdf
            double radVal = Math.toRadians(fieldOfView);
            double arcTanVal = Constants.FRAME_HEIGHT / Constants.FRAME_WIDTH;
            double cosVal = Math.cos(arcTanVal);
            double tanVal = Math.tan(radVal * cosVal);
            double angrad = Math.atan(tanVal);
            double horizontalFieldOfView = Math.toDegrees(angrad);
            // H_FOV = np.degrees(np.arctan(np.tan(np.radians(D_FOV)*np.cos(np.arctan(height/width)))))

            // focal Length f = A / tan(a) where A = frame width / 2 and a = HFOV / 2 in radians
            focalLength = Constants.FRAME_WIDTH/(2*Math.tan(Math.toRadians(horizontalFieldOfView/2)));
        }

        return focalLength;
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

    // Draw bounding box around the reflective tape in Green or a chosen color
    public void drawRect(Point[] pts, Mat mat, Scalar... color) {
        Scalar chosenColor = new Scalar(0, 255, 0);
        if (color.length >= 1){
          chosenColor = color[0];
        }

        for (int i = 0; i < 4; i++)
            Imgproc.line(mat, pts[i], pts[(i + 1) % 4], chosenColor, 2);
      }

    // Calculate the crosshair position
    public Point findCrosshair(Point[] pts, Point crosshair) {
        crosshair = new Point((pts[0].x + pts[2].x) / 2, (pts[0].y + pts[2].y) / 2);
        return crosshair;
    }

    // Draw the crosshair on the frame
    public void drawCrosshair(Point crosshair, Mat mat, Scalar... color) {
        Scalar chosenColor = new Scalar(255, 255, 255);
  
        if (color.length >= 1){
          chosenColor = color[0];
        }
  
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y - 5), new Point(crosshair.x + 5, crosshair.y + 5), chosenColor, 3);
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y + 5), new Point(crosshair.x + 5, crosshair.y - 5), chosenColor, 3);
      }

    // Calculate horizontal turret angle
    public double calculateAngle(Point crosshair) {
        pixelDistance = (int) crosshair.x - Constants.IMG_HOR_MID;
        double angle = pixelDistance * Constants.HOR_DEGREES_PER_PIXEL;
        return angle;
    }

    public Thread getVisionThread() {
        return visionThread;
    }

}