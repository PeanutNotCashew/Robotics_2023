/*
 * Robot design for 2023-2024 FTC - Asynchronous period
 * 
 * Hardware:
 * 4 motors, for wheels
 * 1 motor and 2 servos, for claws
 * 1 servo, for claws
 * 
 +-+-+ PLAN FOR AUTONOMOUS PERIOD +-+-+
1. Drive forward to arms distance from each spike mark
2. Rotate the camera 360 degrees to scan the apriltag on the backboard
3. Program decides whether to drive the long or short distance
4. Turn in the direction of the apriltag.
5. Place down the purple pixel.
6. Drive forward to backboard.
7. Place yellow pixel on backboard.
8. Stay parked in backboard area.
*/

// FIRST Robotics imports
package org.firstinspires.ftc.teamcode;

import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
// Motors and Servos
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
// Webcam and April Tags
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
// IMU
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Autonomous
public class Auto_Default extends LinearOpMode {
    // Distance conversion
    Float feet = 15500f;
    // Wheel Init
    private DcMotor[] drivingMotors = new DcMotor[4];
    // Claw Init
    private DcMotor clawMotor;
    private Servo clawRotServo;
    private CRServo clawCloseServo;

    /*
    // April Tag Init
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    */
    
    // Misc Init
    private IMU imu;
    private ElapsedTime time = new ElapsedTime();

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

        while (computedDistance > 1) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
            computedDistance -= 1;
        }
        while (computedDistance < -1) {
            for (DcMotor i : drivingMotors){
                i.setPower(-1);
            }
            computedDistance += 1;
        }
        halt();
    }

    // Positive to go right
    private void rotate(float angle){
        drivingMotors[0].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[1].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[2].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[3].setDirection(DcMotor.Direction.FORWARD);
        
        imu.resetYaw();
        double botHeading = 0;
        
        if (botHeading > angle) {
            for (DcMotor i : drivingMotors){
                i.setPower(-1);
            }
        } else {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
        }
        
        while (Math.round(botHeading) != angle) {
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }
        halt();
    }
    
    private void strafe(float distance){
        drivingMotors[0].setDirection(DcMotor.Direction.FORWARD);
        drivingMotors[1].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[2].setDirection(DcMotor.Direction.REVERSE);
        drivingMotors[3].setDirection(DcMotor.Direction.FORWARD);

        float computedDistance = distance * feet;

        while (computedDistance > 1) {
            for (DcMotor i : drivingMotors){
                i.setPower(1);
            }
            computedDistance -= 1;
        }
        while (computedDistance < -1) {
            for (DcMotor i : drivingMotors){
                i.setPower(-1);
            }
            computedDistance += 1;
        }
        halt();
    }
    
    private void pause (float seconds) {
        Double end = time.seconds() + seconds;
        while (time.seconds() <= end) {
            // Do nothing
        }
    }

    /*
     * Positions:
     * 1: rest
     * 2: arm up
     * 3: arm down
     */
    
    /*private Float armPos(float goalPos) {
        Float current;
        Float goal;
        Float clawTarget;

        // Sets current arm rotation
        if (lastPos == 1) {}
        else if (lastPos == 2) {}
        else if (lastPos == 3) {}

        // Sets goal arm rotation and target claw rotation
        if (goalPos == 1) {}
        else if (goalPos == 2) {}
        else if (goalPos == 3) {}

        // Computes the amount to move
        current -= goal;
        current *= -1;

        // Rotating the arm
        while (current < -1) {
            clawMotor.setPower(-1);
            current += 1;
        }
        while (current > 1) {
            clawMotor.setPower(1);
            current -= 1;
        }
        clawMotor.setPower(current);
        current = 0f;
        clawMotor.setPower(current);

        // If picking up or dropping, opens the claw
        if ((clawTarget == 2) || (clawTarget == 3)) {
            Double end = time.seconds() + 5;
            while (time.seconds() <= end) {
                clawCloseServo.setPower(1);
            }
            clawCloseServo.setPower(-1);
        }
    }*/

    // Code Running ------------------------------------------------------
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

        /*
        camServo = hardwareMap.get(CRServo.class, "camera");
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
        hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        */

        // Setting motor and servo settings ------------------------------
        // Universal wheel properties
        for (DcMotor i : drivingMotors){
            // Tells motors to run based on power level, not velocity
            i.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // Attempts to turn motor not resisted
            i.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Claw properties
        clawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        /*
        clawRotServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clawRotServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clawCloseServo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BREAK);
        clawCloseServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */
        
        // Setting IMU (gyroscope) orientation params
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
            )
        );
        imu.initialize(imuParameters);

        // Set to starting positions -------------------------------------
        //clawRotServo.setPosition(0.94f);

        // Set path ------------------------------------------------------
        String [][] path = {
            {"f", "2.5"},
            {"r", "90"}, // "l" if blue
            {"f", "8.5"}, // "4" if close
            {"c", "0"},
            {"s", "1.5"}, // "-2" if blue
            {"w", "1"}
        };

        // Ready to start ------------------------------------------------
        telemetry.addData(">", "Ready to start");
        telemetry.update();
        waitForStart();
        time.reset();

        if (opModeIsActive()){
            int i = 0;
            // Interates through path array to move bot
            while (i < path.length) {
                String cmd = path[i][0];
                float amt = Float.parseFloat(path[i][1]);
                telemetry.addData("step", amt);
                telemetry.update();
                
                // Interpret next step
                if (cmd == "f"){ // front
                    forward(amt);
                } else if (cmd == "r"){ // right rotate
                    rotate(-1 * amt);
                } else if (cmd == "l"){ // left rotate
                    rotate(amt);
                } else if (cmd == "s"){ // strafe
                    strafe(amt);
                } else if (cmd == "p"){ // wait
                    pause(amt);
                } else if (cmd == "c") { // activate the claw
                    // Move claw up
                    int j = 0;
                    while (j < 100000) {
                        clawMotor.setPower(1);
                        j += 1;
                    }
                    // Open, then close claw
                    clawMotor.setPower(0);
                    
                    clawCloseServo.setPower(1);
                    pause(1);
                    clawCloseServo.setPower(-1);
                    
                    // Claw back down
                    while (j > 0) {
                        clawMotor.setPower(-1);
                        j -= 1;
                    }
                    clawMotor.setPower(0);
                }
                
                i += 1; // Next step
            }
        }
    }
}
