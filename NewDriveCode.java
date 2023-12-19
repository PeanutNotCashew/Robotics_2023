//this controls omnidirectional movement and one motor-powered arm
//bot uses mecanum wheels
//left joystick controls movement and strafing, shoulders control rotation
//right joystick controls arm
//servos not coded yet
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class NewDriveCode extends LinearOpMode {

    //wheel motors
    private DcMotor motor_fl;
    private DcMotor motor_fr;
    private DcMotor motor_bl;
    private DcMotor motor_br;
    
    private DcMotor motor_arm;

    void mfl(float speed) { // speed from -1 -> 1
      motor_fl.setPower(speed);
    }

    void mfr(float speed) { // speed from -1 -> 1
      motor_fr.setPower(2f*speed);
    }
    
    void mbr(float speed) { // speed from -1 -> 1
      motor_br.setPower(-2f*speed);
    }
    
    void mbl(float speed) { // speed from -1 -> 1
      motor_bl.setPower(speed);
    }

    
    
    @Override
    public void runOpMode() {
        motor_fl = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_fl");
        motor_fr = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_fr");
        motor_bl = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_bl");
        motor_br = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_br");
        
        motor_arm = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "motor_arm");

        
        motor_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        motor_arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        motor_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        waitForStart();
        
        while (opModeIsActive()) {
          
            //this code sets values to different sticks
            telemetry.addData("Status", "Operating as normal.");
            double y = gamepad1.left_stick_y; 
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double r = this.gamepad1.right_trigger - this.gamepad1.left_trigger;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //the values here make the motors turn and stuff
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
            double frontLeftPower = (y + x + r) / denominator;
            double backLeftPower = (y - x + r) / denominator;
            double frontRightPower = (y - x - r) / denominator;
            double backRightPower = (y + x - r) / denominator;
            
            //this sets the powers of the motors to the stuff we did above
            //adapting for issues with motors  rpm; change when we replace motors
            motor_fl.setPower(frontLeftPower / 2);
            motor_bl.setPower(backLeftPower / 2);
            motor_fr.setPower(frontRightPower);
            motor_br.setPower(-1 * backRightPower);
            
            //arm code
            double armPower = gamepad1.right_stick_y;
            motor_arm.setPower(armPower);

            
        

    }
}
    
}
