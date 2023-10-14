package org.firstinspires.ftc.teamcode.x.previous;

//Different

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Autonomous(name="BenAutoGyroTest")

public class BenAutoGyroTest extends LinearOpMode 
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    
    // The IMU sensor object
    private BNO055IMU imu;
        
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;
    private DcMotor elevator = null;
    private Servo intake = null;
    private CRServo rise = null;
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";

    private static int valR = -1;
    private static int valG = -1;
    private static int valB = -1;

    private static float rectHeight = 5f/8f;
    private static float rectWidth = 1.5f/8f;
// 2 6.5
    private static float[] leftPos = {5.0f/8f,4.25f/8f};

    private final int rows = 640;
    private final int cols = 480;
    private int location = 0;
    double risePower = 0.0;

    OpenCvCamera webcam;

    int cameraMonitorViewId;

    @Override
    public void runOpMode() 
    {
        telemetry.addData("Status", "A");
        telemetry.update();

        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        intake     = hardwareMap.get(Servo.class, "intake");
        elevator   = hardwareMap.get(DcMotor.class, "elevator");
        rise     = hardwareMap.get(CRServo.class, "rise");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        // parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status", "B");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new StageSwitchingPipeline());
         webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        while (opModeInInit()) {
            
            if (valR > valG && valR > valB)
            {
                location = 1;
            }
            
            if (valG > valR && valG > valB)
            {
                location = 2;
            }
            
            if (valB > valR && valB > valG)
            {
                location = 3;
            }
            
            telemetry.addData("Values", valR+"   "+valG+"   "+valB);
            telemetry.addData("Location", location);
            telemetry.update();
        }

        runtime.reset();
        // Turn is evil
        
       //grab cone
        intake.setPosition(0.2);
        
        //whole elevator deploys
        rise.setPower(-0.5);
        
        sleep(3300);
        
        rise.setPower(-0.15);
    
        move( 0, -250, 0.3 );
        
        sleep(1000);
        
        elevator.setPower(-0.5);
        
        sleep(1000);
        
        elevator.setPower(0.0);
        
        move( 0, -1000, 0.3 );

        //move( -40.5, 387, 0.3 );
        //intake.setPosition(0.6) ;
        //sleep(3000);
        //move( -40.5,-420, 0.3 );
        
        //move( -90, 0, 0.3 );
        
        if (location == 1) {
            move( -90, 1000, 0.3 );
            move( 0, 0, 0.3 );
        } else if (location == 3) {
            move( -90, -50, 0.3 );
            move( -90, -850, 0.3);
            move( 0, 0, 0.3 );
        } else if (location == 2) {
            turn (5);
        }
        
        
        move( 0, -375, 0.3 );
        
        

        sleep(3000);
    }

    private void move( double targetAngle, int target, double power )
    {
        double P = 0.00004;
        double I = 0.000002;  // 0.000001;
        double sumError = 0;
        double error;
        Orientation angles;

        turn( targetAngle );
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightDrive.setTargetPosition(target);
        leftDrive.setTargetPosition(-target);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(power);
        rightDrive.setPower(power);

        while(rightDrive.isBusy() )  // && rightDrive.isBusy()) 
        {
            angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = (targetAngle - angles.firstAngle);
            sumError += error;
            
            double errorPower = (error * P) + (sumError * I);
            //double errorPower = (error * P);
            leftDrive.setPower(power-errorPower);
            rightDrive.setPower(power+errorPower);
            
            telemetry.addData("Left", "%s | %s | %s", leftDrive.getPower(), leftDrive.getTargetPosition(), leftDrive.getCurrentPosition());
            telemetry.addData("Right", "%s | %s | %s", rightDrive.getPower(), rightDrive.getTargetPosition(), rightDrive.getCurrentPosition());
            telemetry.addData("IMU", "Angle: %s", formatDegrees(angles.firstAngle));
            telemetry.addData("ErrorPower", "%f", errorPower);
            telemetry.addData("Error", "%f", error);
            telemetry.update();
        }
        
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    

    private void turn(double targetAngle)
    {
        double P = 0.0035;
        double I = 0.00004;
        double MAX_ERROR = 0.375;
        double sumError = 0;
        Orientation angles;
        
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = (targetAngle - angles.firstAngle);

        while(Math.abs(error) > MAX_ERROR) 
        {
            angles  = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            error = (targetAngle - angles.firstAngle);
            sumError += error;
            
            double power = (error * P) + Math.signum(error) * 0.2 + (sumError * I);
            leftDrive.setPower(-power);
            rightDrive.setPower(-power);

            telemetry.addData("IMU", "Angle: %s", formatDegrees(angles.firstAngle));
            telemetry.addData("Power", "%f", power);
            telemetry.addData("Error", "%f", error);
            telemetry.update();
        }    
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            input = input.submat(new Rect(new Point(160, 240), new Point(480, 480)));
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            //Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            //Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            //Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            //Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            //yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame

            double[] pixLeft = input.get((int)(input.rows() * leftPos[1]), (int)(input.cols() * leftPos[0]));//gets value at circle
            valR = (int)pixLeft[0];
            valG = (int)pixLeft[1];
            valB = (int)pixLeft[2];
            
            


            //create three points
            //Point pointMid = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point pointLeft = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows() * leftPos[1]));
           //Point pointRight = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            //Imgproc.circle(all, pointMid,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            //draws circle
            //Imgproc.circle(all, pointRight,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(input, pointLeft,5, new Scalar( 255, 0, 0 ),1 );
            Imgproc.circle(input, pointLeft,25, new Scalar( 0, 255, 0 ),1 );
            return input;

            //draw 3 rectangles

    }}
}

