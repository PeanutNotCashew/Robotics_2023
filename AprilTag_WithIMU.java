/*
 * Robot design for 2023-2024 FTC - Asynchronous period
 * 
 * Hardware:
 * 4 motors, for wheels
 * 1 motor and 2 servos, for claws
 * 
 */

// FIRST Robotics imports
package org.firstinspires.ftc.teamcode;
// Misc
import java.util.List;
import java.util.HashMap;
import java.util.Arrays;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// Motors and Servos
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
// Webcam and April Tags
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
// IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class AprilTag_WithIMU extends LinearOpMode {
    // Misc Init
    private IMU imu;
    private ElapsedTime time = new ElapsedTime();
    static final boolean BLUE_SIDE = true; // Starting position
    static final boolean CLOSE = true;
    static final boolean OUTSIDE = false; // Ending position
    
    // Distance conversion
    Float feet = 15500f;
    // Wheel Init
    private DcMotor[] drivingMotors = new DcMotor[4];
    // Claw Init
    private DcMotor clawMotor;
    private Servo clawRotServo;
    private CRServo clawCloseServo;
    
    // Vision Inits
    AprilTagProcessor aprilTags;
    TfodProcessor tfod;
    VisionPortal visionPortal;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/GamePieces_2023_24.tflite";
    private static final String[] LABELS = { // Define labels recognized. Must be in training order
       "blue_element",
       "red_element",
    };
    
    String gamePiece;
    int goalAprilTag;
    
    // Path inits
    String [] pathCmds = {
        // Checking for game piece
        "f", "s", "gp", "s", "gp",
        // Move to front of board
        "s", "r", "s",
        // Repeatedly look for april tag
        "at", "s", "at", "s", "at",
        // End portion: Leave
        "s", "c"
    }

    float [] pathAmts = {
        // Checking for game piece
        0.2, 1.7, 1, -2.0, 2,
        // Move to front of board
        -2, -90  3,
        // Repeatedly look for april tag
        0, 1, 0, 1, 0,
        // End portion: Leave & drop
        1, 1
    };
    
    private void pause (int seconds) {
        Double end = time.seconds() + seconds;
        while (time.seconds() <= end) {
            // Do nothing
        }
    }

    // Movement Functions ------------------------------------------------------
    private void halt() {
        for (DcMotor i : drivingMotors){
            i.setPower(0);
        }
    }

    private void forward(float distance){
        drivingMotors[0].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[1].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[2].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[3].setDirection(DcMotor.Direction.FORWARD);

        float computedDistance = distance * feet;

        while (computedDistance >= 1) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
            computedDistance -= 1;
        }
        while (computedDistance <= -1) {
            for (DcMotor i : drivingMotors){
                i.setPower(-1);
            }
            computedDistance += 1;
        }
        halt();
    }

    private void rotate(float angle){
        drivingMotors[0].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[1].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[2].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[3].setDirection(DcMotor.Direction.REVERSE);
        
        int botHeading = (int)imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        
        // Compute angle to move to
        // IMU uses degree angles of -180 to 180.
        int computedAngle = ((int)angle + botHeading) % 360;
        if (computedAngle > 180) {
            computedAngle = 180 - computedAngle;
        }
        
        if (botHeading > angle) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
        } else {
            for (DcMotor i : drivingMotors){
                i.setPower(-1);
            }
        }
        
        while (Math.round(botHeading) != computedAngle) {
            botHeading = (int)imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        halt();
    }
    
    private void strafe(float distance){
        drivingMotors[0].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[1].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[2].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[3].setDirection(DcMotor.Direction.FORWARD);

        float computedDistance = distance * feet;

        while (computedDistance >= 1) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
            computedDistance -= 1;
        }
        while (computedDistance <= -1) {
            for (DcMotor i : drivingMotors){
                i.setPower(-1);
            }
            computedDistance += 1;
        }
        halt();
    }
    
    // Object Recognition Related Functions ------------------------------------
    private void claw(int presetPos) {
        int armPosition;
        int wristPosition;
        if (presetPos == 1) { // Put on the board
            clawMotor.setTargetPosition(2690);
            clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawMotor.setPower(0.5);
            pause(1);
            clawRotServo.setPosition(0.3);
            while (clawMotor.isBusy()) {
                // Do nothing
            }
            clawCloseServo.setPower(-1);
        } else if (presetPos == 2) { // On the ground
            clawMotor.setTargetPosition(3260);
            clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            clawMotor.setPower(0.5);
            pause(1);
            clawRotServo.setPosition(0.3);
            while (clawMotor.isBusy()) {
                // Do nothing
            }
            clawCloseServo.setPower(-1);
        }
        
        pause(1);
        
        // Go back to starting position
        clawMotor.setTargetPosition(10);
        clawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawMotor.setPower(-0.3);
        pause(1);
        clawRotServo.setPosition(0.8);
        clawCloseServo.setPower(1);
        clawMotor.setPower(-0.5);
        while (clawMotor.isBusy()) {
            // Do nothing
        }
    }
    
    private boolean toBoard(int goalID) {
        boolean tagFound = false;
        
        float forwardDistance = 0; // Inches
        float sidewaysDistance = 0; // Inches
        float rotation = 0; // Degrees
        
        // Find and get AprilTag data
        visionPortal.setProcessorEnabled(aprilTags, true);
        Double end = time.seconds() + 1;
        while (time.seconds() <= end) {
            for (AprilTagDetection detection : aprilTags.getDetections()) {
                if ((detection.metadata != null) && (detection.id == goalID)) {
                    tagFound = true;
                    
                    forwardDistance = (float)detection.ftcPose.y;
                    sidewaysDistance = (float)detection.ftcPose.x;
                    rotation = (float)detection.ftcPose.yaw;
                }
            }
        }
        visionPortal.setProcessorEnabled(aprilTags, false);
        
        // Move to board & place pixel
        if (tagFound){
            rotate(rotation);
            forward((forwardDistance - 6) / 12);
            strafe((sidewaysDistance - 30) / 12); // Compensates for webcam/arm offset
            
            claw(1);
        }
        
        return tagFound;
    }
    
    private int checkForGamePiece(int botPosition) {
        visionPortal.setProcessorEnabled(tfod, true);
        
        Integer gpPosition = null;
        
        // Get game piece data
        visionPortal.setProcessorEnabled(tfod, true);
        Double end = time.seconds() + 1;
        while (time.seconds() <= end) {
            for (Recognition recognition : tfod.getRecognitions()) {
                if (recognition.getLabel() == gamePiece) {
                    gpPosition = (int)((recognition.getLeft() + recognition.getRight())/2);
                }
            }
        }
        visionPortal.setProcessorEnabled(tfod, false);
        
        // Determine which april tag to go to
        if (gpPosition != null) {
            if (((gpPosition < 500) && (botPosition == 2))) {
                if (BLUE_SIDE) {
                    return 1;
                } else {
                    return 4;
                }
            } else if ((gpPosition > 200) && (botPosition == 1)) {
                if (BLUE_SIDE) {
                    return 3;
                } else {
                    return 6;
                }
            } else {
                if (BLUE_SIDE) {
                    return 2;
                } else {
                    return 5;
                }
            }
        }
        return 0;
    }

    // Code Running ------------------------------------------------------------
    @Override
    public void runOpMode(){
        // Mapping
        drivingMotors[0] = hardwareMap.get(DcMotor.class, "motor_fl");
        drivingMotors[1] = hardwareMap.get(DcMotor.class, "motor_fr");
        drivingMotors[2] = hardwareMap.get(DcMotor.class, "motor_bl");
        drivingMotors[3] = hardwareMap.get(DcMotor.class, "motor_br");

        clawMotor = hardwareMap.get(DcMotor.class, "motor_arm");
        clawRotServo = hardwareMap.get(Servo.class, "wrist_rot");
        clawCloseServo = hardwareMap.get(CRServo.class, "claw_rot");
        
        // Vision Configuration
        aprilTags = AprilTagProcessor.easyCreateWithDefaults();
        tfod = new TfodProcessor.Builder()
            .setModelFileName(TFOD_MODEL_FILE)
            .setModelLabels(LABELS)
            .setIsModelTensorFlow2(true)
            .setIsModelQuantized(true)
            .setModelInputSize(300)
            .setModelAspectRatio(16.0 / 9.0)
            .build();
        tfod.setMinResultConfidence(0.6f);
        visionPortal = VisionPortal.easyCreateWithDefaults(
            hardwareMap.get(WebcamName.class, "Webcam 1"),
            aprilTags,
            tfod
        );
        // Turn off to save resources
        visionPortal.setProcessorEnabled(aprilTags, false);
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.stopLiveView();

        // Setting motor and servo settings ------------------------------------
        // Universal wheel properties
        for (DcMotor i : drivingMotors){
            i.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Arm properties
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Setting IMU (gyroscope) orientation params
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu.initialize(imuParameters);
        imu.resetYaw();

        // Set to starting positions -------------------------------------------
        clawRotServo.setPosition(0.8f);
        clawCloseServo.setPower(1);

        // Set pathing settings ------------------------------------------------
        if (OUTSIDE) {
            pathAmts[13] = -3;
        }

        if (BLUE_SIDE) {
            goalAprilTag = 2;
            gamePiece = "blue_element";
            if (!CLOSE) {
                pathAmts[5] = -7;
            }
        } else {
            goalAprilTag = 5;
            gamePiece = "red_element";
            for (int i = 5; i < (pathAmts.length() - 1); i++) { // Reverse direction
                pathAmts[i] *= -1;
            }
            if (!CLOSE) {
                pathAmts[5] = 7.3;
            }
        }
        
        // Ready to start ------------------------------------------------------
        telemetry.addData(">", "Ready to start");
        telemetry.update();
        waitForStart();
        time.reset();

       
        int i = 0;
        // Interates through path array to move bot
        while ((opModeIsActive()) && (i < path.length)) {
            String cmd = pathCmds[i];
            float amt = pathAmts[i];
            
            // Go forwards for "amt" of feet. Negative for backwards
            if (cmd == "f"){
                forward(amt);

            // Strafe right for "amt" of feet. Negative for leftwards
            } else if (cmd == "r"){.
                rotate(-1 * amt);
            
            // Rotate right for "amt" of degrees. Negative for leftwards
            } else if (cmd == "s"){
                strafe(amt);

            // Pause for "amt" of seconds
            } else if (cmd == "p"){
                pause((int)amt);

            // Find april tag position, then place pixel
            } else if (cmd == "at"){
                if (toBoard(goalAprilTag)) {
                    // Cancel other april tag checks
                    pathCmds[10] = "n/a";
                    pathCmds[12] = "n/a";
                    // Cancel just-in-case pixel placement
                    pathCmds[14] = "n/a";
                }
            
            // Find gamepiece position
            } else if (cmd == "gp"){
                int temp = checkForGamePiece((int)amt);
                if (temp != 0) {
                    goalAprilTag = temp;
                }
            }
            
            telemetry.addData("Tag", goalAprilTag);
            telemetry.update();
            
            i += 1; // Next step
        }
        
    }
}
