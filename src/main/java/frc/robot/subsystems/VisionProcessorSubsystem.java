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

    private static UsbCamera ballDetectionCamera;
    private static UsbCamera hubDetectionCamera;
    private static CvSource processedOutputStreamHub;
    private static CvSource processedOutputStreamRed;
    private static CvSource processedOutputStreamBlue;
    private static CvSink cvSink;
    private static HubGripPipeline hubGrip;
    private static RedGripPipeline redGrip;
    private static BlueGripPipeline blueGrip;
    private static Mat hubMat;
    private static Mat blueMat;
    private static Mat redMat;
    private static Point hubCrosshair;
    private static Point blueCrosshair;
    private static Point redCrosshair;
    private static Point[] hubPts = new Point[4];
    private static Point[] bluePts = new Point[4];
    private static Point[] redPts = new Point[4];
    private static int pixelDistance;
    private static double hubAngle;
    private static double blueAngle;
    private static double redAngle;
    private Object lock = new Object();
    private Thread visionThread;
    private NetworkTableEntry angleEntry;
    private Solenoid ringLight;
    private MedianFilter hubMaxYFilter;
    private MedianFilter hubMinYFilter;
    private MedianFilter hubMaxXFilter;
    private MedianFilter hubMinXFilter;
    private MedianFilter redMaxYFilter;
    private MedianFilter redMinYFilter;
    private MedianFilter redMaxXFilter;
    private MedianFilter redMinXFilter;
    private MedianFilter blueMaxYFilter;
    private MedianFilter blueMinYFilter;
    private MedianFilter blueMaxXFilter;
    private MedianFilter blueMinXFilter;
    //int finalAngle;

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
        // Creates a MedianFilter with a window size of 5 samples for hub
        hubMaxYFilter = new MedianFilter(5);
        hubMinYFilter = new MedianFilter(5);
        hubMaxXFilter = new MedianFilter(5);
        hubMinXFilter = new MedianFilter(5);

        //red
        redMaxYFilter = new MedianFilter(5);
        redMinYFilter = new MedianFilter(5);
        redMaxXFilter = new MedianFilter(5);
        redMinXFilter = new MedianFilter(5);

        //blue
        blueMaxYFilter = new MedianFilter(5);
        blueMinYFilter = new MedianFilter(5);
        blueMaxXFilter = new MedianFilter(5);
        blueMinXFilter = new MedianFilter(5);
 
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
        hubMat = new Mat();
        redMat = new Mat();
        blueMat = new Mat();

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
            hubCrosshair = null;
            if (cvSink.grabFrame(hubMat) == 0) {
                processedOutputStreamHub.notifyError(cvSink.getError());
            }
            if (cvSink.grabFrame(redMat) == 0) {
                processedOutputStreamRed.notifyError(cvSink.getError());
            }
            if (cvSink.grabFrame(blueMat) == 0) {
                processedOutputStreamBlue.notifyError(cvSink.getError());
            }
            
            hubGrip.process(hubMat);
            redGrip.process(redMat);
            blueGrip.process(blueMat);

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
            findBoundingBoxesHub();
            findBoundingBoxesBlue();
            findBoundingBoxesRed();

            if (hubCrosshair != null) {
                synchronized (lock) {
                    calculateAngle(hubAngle, hubCrosshair);
                }
            }
            if (redCrosshair != null) {
                synchronized (lock) {
                    calculateAngle(redAngle, redCrosshair);
                }
            }
            if (blueCrosshair != null) {
                synchronized (lock) {
                    calculateAngle(blueAngle, blueCrosshair);
                }
            }
          
            if (frameCount == 1) {
                processedOutputStreamHub.putFrame(hubMat);
                processedOutputStreamRed.putFrame(redMat);
                processedOutputStreamBlue.putFrame(blueMat);
                frameCount = 0;
            }

            frameCount++;
    
        }
    }    

    public void findBoundingBoxesRed() {
        ArrayList<MatOfPoint> contours = redGrip.filterContoursOutput();
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++){
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
        }

        if (contours.size() != 0) {
            double rMinX = rects[0].boundingRect().x;
            double rMaxX = 0;
            double rMinY = rects[0].boundingRect().y;
            double rMaxY = 0;

            for (int a = 0; a < rects.length; a++) {
                rMinX = Math.min(rMinX, rects[a].boundingRect().x);
                rMaxX = Math.max(rMaxX, rects[a].boundingRect().x);
                rMinY = Math.min(rMinY, rects[a].boundingRect().y);
                rMaxY = Math.max(rMaxY, rects[a].boundingRect().y);
            }

            double frMinX = redMinXFilter.calculate(rMinX);
            double frMaxX = redMaxXFilter.calculate(rMaxX);
            double frMinY = redMinYFilter.calculate(rMinY);
            double frMaxY = redMaxYFilter.calculate(rMaxY);

            redPts[0] = new Point(frMinX, frMinY);
            redPts[1] = new Point(frMaxX, frMinY);
            redPts[2] = new Point(frMaxX, frMaxY);
            redPts[3] = new Point(frMinX, frMaxY);

            drawRect(redPts, redMat);
            findCrosshair(redPts, hubCrosshair);
        
            if (redCrosshair != null){
                drawCrosshair(redCrosshair, redMat);
                calculateAngle(redAngle, redCrosshair);
            }
            SmartDashboard.putNumber("redAngle", redAngle);
        }
    }

    public void findBoundingBoxesBlue() {
        ArrayList<MatOfPoint> contours = blueGrip.filterContoursOutput();
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++){
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));
        }

        if (contours.size() != 0) {
            double bMinX = rects[0].boundingRect().x;
            double bMaxX = 0;
            double bMinY = rects[0].boundingRect().y;
            double bMaxY = 0;

            for (int a = 0; a <rects.length; a++) {
                bMinX = Math.min(bMinX, rects[a].boundingRect().x);
                bMaxX = Math.max(bMaxX, rects[a].boundingRect().x);
                bMinY = Math.min(bMinY, rects[a].boundingRect().y);
                bMaxY = Math.max(bMaxY, rects[a].boundingRect().y);
            }

            double fbMinX = blueMinXFilter.calculate(bMinX);
            double fbMaxX = blueMaxXFilter.calculate(bMaxX);
            double fbMinY = blueMinYFilter.calculate(bMinY);
            double fbMaxY = blueMaxYFilter.calculate(bMaxY);

            bluePts[0] = new Point(fbMinX, fbMinY);
            bluePts[1] = new Point(fbMaxX, fbMinY);
            bluePts[2] = new Point(fbMaxX, fbMaxY);
            bluePts[3] = new Point(fbMinX, fbMaxY);

            drawRect(bluePts, blueMat);
            findCrosshair(bluePts, hubCrosshair);
        
            if (blueCrosshair != null){
                drawCrosshair(blueCrosshair, blueMat);
                calculateAngle(blueAngle, blueCrosshair);
            }
            SmartDashboard.putNumber("blueAngle", blueAngle);
        }
    }

    public void findBoundingBoxesHub() {
        ArrayList<MatOfPoint> contours = hubGrip.filterContoursOutput();
        //System.out.println(contours.size());
        RotatedRect[] rects = new RotatedRect[contours.size()];
        for (int i = 0; i < contours.size(); i++)
            rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

        if (contours.size() != 0){
            double hMinX = rects[0].boundingRect().x;
            double hMaxX = 0;
            double hMinY = rects[0].boundingRect().y;
            double hMaxY = 0;

            for(int a=0; a<rects.length; a++ ) {
                hMinX = Math.min(hMinX, rects[a].boundingRect().x);
                hMaxX = Math.max(hMaxX, rects[a].boundingRect().x);
                hMinY = Math.min(hMinY, rects[a].boundingRect().y);
                hMaxY = Math.max(hMaxY, rects[a].boundingRect().y);
            }

            double fhMinY = hubMinYFilter.calculate(hMinY);
            double fhMaxY = hubMaxYFilter.calculate(hMaxY);
            double fhMinX = hubMinXFilter.calculate(hMinX);
            double fhMaxX = hubMaxXFilter.calculate(hMaxX);
        
            //SmartDashboard.putNumber("HubMinX", fMinY);
            //SmartDashboard.putNumber("HubMinY", fMaxY);
            //SmartDashboard.putNumber("HubMaxX", fMinX);
            //SmartDashboard.putNumber("HubMaxY", fMaxX);

            //RotatedRect boundingBox = new RotatedRect();
            hubPts[0] = new Point(fhMinX, fhMinY);
            hubPts[1] = new Point(fhMaxX, fhMinY);
            hubPts[2] = new Point(fhMaxX, fhMaxY);
            hubPts[3] = new Point(fhMinX, fhMaxY);

            double fieldOfView = 68.5;
            int pixelWidth = (int) (fhMaxX - fhMinX);
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

            int finalDistance = (int) (calculatedDistance + parabolaError);
        
            /*SmartDashboard.putNumber("initial distance", calculatedDistance);
            SmartDashboard.putNumber("final distance", finalDistance);
            SmartDashboard.putNumber("focalLength", focalLength);
            SmartDashboard.putNumber("pixelWidth", pixelWidth);
            SmartDashboard.putNumber("parabola Error", parabolaError);
*/
            //if ()
            drawRect(hubPts, hubMat);
            findCrosshair(hubPts, hubCrosshair);
        
            if (hubCrosshair != null)
                drawCrosshair(hubCrosshair, hubMat);
                calculateAngle(hubAngle, hubCrosshair);
               //. SmartDashboard.putNumber("finalAngle", finalAngle);
            }
            SmartDashboard.putNumber("hubAngle", hubAngle);
    }

    public RotatedRect findLargestRect(RotatedRect[] rects) {
        RotatedRect rect = rects[0];
        for (int i = 0; i < rects.length; i++) {
            if (rects[i].size.area() > rect.size.area())
                rect = rects[i];
        }
        return rect;
    }

    // Draw bounding box around the reflective tape
    public void drawRect(Point[] pts, Mat mat) {
        for (int i = 0; i < 4; i++)
            Imgproc.line(mat, pts[i], pts[(i + 1) % 4], Constants.RED, 2);

    }

    // Calculate the crosshair position
    public void findCrosshair(Point[] pts, Point crosshair) {
        // i is starting point for line, j is next point
        int j;
        for (int i = 0; i < 4; i++) {
            j = (i + 1) % 4;
            if (crosshair == null || (pts[i].y + pts[j].y) / 2 < crosshair.y)
            crosshair = new Point((pts[i].x + pts[j].x) / 2, (pts[i].y + pts[j].y) / 2);
        }
    }

    // Draw the crosshair on the frame
    public void drawCrosshair(Point crosshair, Mat mat) {
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y - 5), new Point(crosshair.x + 5, crosshair.y + 5), Constants.BLUE, 3);
        Imgproc.line(mat, new Point(crosshair.x - 5, crosshair.y + 5), new Point(crosshair.x + 5, crosshair.y - 5), Constants.BLUE, 3);
    }

    // Calculate horizontal turret angle
    public void calculateAngle(double angle, Point crosshair) {
        pixelDistance = (int) crosshair.x - Constants.IMG_HOR_MID;
        //SmartDashboard.putNumber("pixelDistance", pixelDistance);
        angle = pixelDistance * Constants.HOR_DEGREES_PER_PIXEL;
        //double a = -0.00549184;
        //double b = 0.100438;
        //double c = -1.47937;

        //double parabolaError = a * Math.pow(angle, 2) + b * angle + c;
        //-\ 0.00549184x^{2}+0.100438x-1.47937
        //finalAngle = (int) (angle + parabolaError);

        angleEntry.setDouble(angle);
    };


    // Getter for angle
    public double getAngle() { 
        return hubAngle;
    }

    public Thread getVisionThread() {
        return visionThread;
    }
}