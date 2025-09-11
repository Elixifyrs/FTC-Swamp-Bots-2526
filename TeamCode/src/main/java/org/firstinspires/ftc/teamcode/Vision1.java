package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "Vision1")
public class Vision1 extends LinearOpMode {
    //constants for cam, gotta figure out what they do tho
    double cX = 0;
    double cY = 0;
    double width = 0;

    //cam var
    private OpenCvCamera maincam; // use OpenCVCamera class from FTC SDK
    //I have no clue where FTC SDK is

    //Cam dimensions
    //Make sure its the dimension of the camera we have
    private static final int Cam_Width = 640;
    private static final int Cam_Height = 480;
    public static final double objectWidthInRealWorldUnits = 5;     //should be in inches i believe
    public static final double focalLength = 999; //have to calculate for our cam ofc
    @Override
    public void runOpMode() {
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(maincam, 30); //set to whatever fps we got

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch" , (BallDetectionPipeline.getDistance(width)));
            telemetry.update();

            // OpenCV pipeline auto processes frames and handles detection or should be
        }

        //release resources
        maincam.stopStreaming();
    }

    private void initOpenCV() {
        //Create cam instance (declare a video cap obj basically)
        int cameraMonitorViewID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        //Use OpenCvCamera Factory class on Main branch to create cam instance
        maincam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewID);
        maincam.setPipeline(new BallDetectionPipeline());
        maincam.openCameraDevice();
        maincam.startStreaming(Cam_Width, Cam_Height, OpenCvCameraRotation.UPRIGHT); //da rotation depends on physical and the actual cam
    }


    class BallDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input){
            Mat purpleMask = preprocessFrame(input);

            //Find contours or bounds of detected purple regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(purpleMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            //Find the largest contour
            MatOfPoint largestContour = findLargestContour(contours);

            if(largestContour != null){
                //Draw a red outline around the largest detected obj
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 0, 255), 2);
                //calcualte the width of the bounding box
                width = calculateWidth(largestContour);

                //Display width next to label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY +30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                //Display Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                //Calculate the centroid of the largest contour
                Moments m = Imgproc.moments(largestContour);
                cX = m.get_m10() / m.get_m00();
                cY = m.get_m01() / m.get_m00();

                //Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY + 90), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 4, new Scalar(0, 255, 0), -1);

            }

            return input;

        }

        //may make 2 for purple and green
        private Mat preprocessFrame(Mat input){

            Mat hsvFrame = new Mat(); //stores HSV values
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_RGB2HSV); //assume that we have to change the color

            Scalar lowerPurple = new Scalar(130, 150, 0);
            Scalar upperPurple = new Scalar(179, 255, 255);

            Mat purpleMask = new Mat(); //stores RGB values
            Core.inRange(hsvFrame, lowerPurple, upperPurple, purpleMask); //identify purple pixels and makes em white to mask

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)); //to make img clearer in binary???
            Imgproc.morphologyEx(purpleMask, purpleMask, Imgproc.MORPH_CLOSE, kernel); //to make kernal even more clear and fill in gaps due to lighting and stuff
            return purpleMask;
        }

        //finds the biggest blob of purple pixels
        private MatOfPoint findLargestContour(List<MatOfPoint> contours){
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for(MatOfPoint contour : contours){
                double area = Imgproc.contourArea(contour);
                if(area > maxArea){
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }

        //width of the purple blob
        private double calculateWidth(MatOfPoint contour){
            Rect boundingRect = Imgproc.boundingRect(contour);
            double width = boundingRect.width;
            return width;
        }
        private static double getDistance(double width){
            double distance = (objectWidthInRealWorldUnits * focalLength) / width;
            return distance;
        }
    }
}
