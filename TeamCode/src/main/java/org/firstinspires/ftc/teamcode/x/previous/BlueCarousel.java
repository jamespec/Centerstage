package org.firstinspires.ftc.teamcode.x.previous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name="BlueCarousel")
@Disabled
public class BlueCarousel extends LinearOpMode 
{
    // private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode()
    {
        ElapsedTime runtime = new ElapsedTime();

        int level = 3;  // Target level, 3 unless we decide otherwise.

        telemetry.addData("Status", "DO NOT RUN YET");
        telemetry.update();

        // We are using a Chassis class to handle all motion
        TwoMotorChassis twoMotorChassis = new TwoMotorChassis( hardwareMap, telemetry );

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        DcMotor wheel      = hardwareMap.get(DcMotor.class, "wheel");
        CRServo elevator   = hardwareMap.get(CRServo.class, "elevator");
        CRServo intake     = hardwareMap.get(CRServo.class, "intake");

        // Couldn't get the Distance Sensors to work.
        // We'll put these back in if we get time to debug.
        // DistanceSensor leftDistanceSensor = hardwareMap.get(DistanceSensor.class, "left_distance");
        // DistanceSensor rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "right_distance");

        // Wait for the game to start (driver presses PLAY)
        initVuforia();
        initTfod();  // Sets the tfod value.

        if (tfod != null) {
            tfod.activate();
            //tfod.setZoom(1.5, 16.0/9.0);
        }

        // -----------------------------
        // Wait for the game to begin
        // -----------------------------
        telemetry.addData(">", "Press Play to start Autonomous");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("label ", "(%d): %s",
                                        i, recognition.getLabel() );
                    telemetry.addData( "  left,top ", "(%d): %.03f , %.03f",
                                        i, recognition.getLeft(), recognition.getTop() );
                    telemetry.addData( "  right,bottom ", "(%d): %.03f , %.03f",
                                        i, recognition.getRight(), recognition.getBottom() );

                    // Ignore detected objects that don't reach the bottom of the FOV.
                    if( recognition.getBottom() < 300 )
                        continue;

                    i++;
                    if (recognition.getLeft() > 280) {
                        // Object on right side indicate level 2.
                        level = 2;
                    }
                    else {
                        level = 1;
                    }
                }
            }
        }
   
        telemetry.addData("level", "(%d)", level);
        telemetry.update();
        sleep(1000);
        
        double wheelPower = .75;
        
        // ======================
        // == first step       ==
        // ======================

        // Backup to Carousel
        twoMotorChassis.move( 0.0, -1250, 0.2, 1.0 );

        // Start raising elevator - slowly, don't wait, just set power.
        if (level == 3) {
            elevator.setPower(-0.125);
        }

        // Turn wheel for 3 seconds to deliver Duck.
        wheel.setPower(wheelPower);
        sleep(3000);
        wheel.setPower(0);
       
        // Forward along wall to align with Hub.
        twoMotorChassis.move( 0.0, 2175, 0.3, 1.0 );
        
        // TURN 90 degrees right, note negative angle is clockwise.
        twoMotorChassis.turn(-90, 2.5);

        // Back into wall to straighten out and be precise distance from the hub.
        twoMotorChassis.move( -90, -200, 0.2, 1.0 );

        // Raise elevator ahead of advancing to Hub to avoid bumping.
        // Forward to the hub.
        // Different values depending on the level.
        switch( level ) {
            case 1:
                elevator.setPower(-0.3);
                sleep(1500);
                elevator.setPower(-0.0625);  // small amount to avoid the elevator sagging.

                // Move to Hub
                twoMotorChassis.move( -90, 1155, 0.2, 1.0 );
                break;
            case 2:
                elevator.setPower(-0.3);
                sleep(2000);
                elevator.setPower(-0.0625);

                // Move to Hub
                twoMotorChassis.move( -90, 1150, 0.2, 1.0 );
                break;
            case 3:
                twoMotorChassis.move( -90, 1120, 0.2, 1.0 );
                break;
        }

        // drop block on Hub.
        intake.setPower(0.15);
        sleep(1500);
        intake.setPower(0.0);
        
        // Back away from hub, allow elevator to fall
        twoMotorChassis.move( -90, -1100, 0.3, 1.0 );
        elevator.setPower(0.2);

        // Turn left towards the warehouse, slow down elevator fall
        twoMotorChassis.turn(25, 3.0);
        elevator.setPower(0.1);

        // Drive into warehouse, stop elevator
        twoMotorChassis.move( 25, 3000, 0.5, 1.0 );
        elevator.setPower(0.0);
    }

    // Initialization of Vuforia software
    // This works with the camera to to create a video stream
    // The stream is sent to TensorFlow to attempt to recognize objects
    private void initVuforia()
    {
        final String VUFORIA_KEY =
                "AROHLt//////AAABmRqtKHXnCETrrKry+MJXVcdqPPGOE6f4kvj9Kh5mhFwXKquVVjlrr+9T0G3ckUn7PEtQocoVIAwVfdY7y1cqkibeTI3IxRhf1taeZO+ovnE8T++Udbtqy8Y9sN9IwHbXus5AXTLp1s1jEZGB5thPT7rLACUDOouus47BeF8Ygyj5ygGSDIGEh0hWdwtF3G4zI1HUjPNtVakFVYQMqIaaUmv0FtTRUJP+2aBeEtETU5dmuq6eLZ76sHWarETv+lzUe1rOZM7fTNKfRGT/M6TZZXmnbOB2w45TM6nS5Z15vUjzuvX+L+Nxn+BeaAjmPpWk87gecXYnTyQ5bz+oOJtld5EkIMgu3DSyFEg46O374Ytr";

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine with supplied parameters
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // Initialize the TensorFlow Object Detection engine.
    // The Vuforia video stream is scanned in an attempt to recognize objects
    // The TensorFlowObjectDetector (tfod) is created and initialize to report:
    //   Objects with an 80% probability (lowered from 90)
    //   Limited view of 320 wide pixels from the camera
    //   The data about the objects is stored in a "resource" file
    private void initTfod()
    {
        final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
        final String[] LABELS = {
                "Ball",
                "Cube",
                "Duck"
                // , "Marker"
        };

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

//       TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//       tfodParameters.minResultConfidence = 0.8f;
//       tfodParameters.isModelTensorFlow2 = true;
//       tfodParameters.inputSize = 320;

//       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
