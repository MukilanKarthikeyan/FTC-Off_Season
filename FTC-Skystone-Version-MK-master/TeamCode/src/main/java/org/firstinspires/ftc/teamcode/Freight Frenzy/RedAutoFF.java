package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class RedAutoFF extends LinearOpMode {

    DcMotor rightFront, rightBack, leftFront, leftBack;
    DcMotor carousel;

    Servo wobbleGrab;

    @Override
    public void runOpMode() throws InterruptedException{


        rightFront = hardwareMap.get(DcMotor.class, "rf"); // control motor port 0
        rightBack = hardwareMap.get(DcMotor.class, "rb"); // control motor port 1
        leftFront = hardwareMap.get(DcMotor.class, "lf");// control motor port 2
        leftBack = hardwareMap.get(DcMotor.class, "lb");// control motor port 3
/*

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/


        EleDrive robot_drive = new EleDrive(this);

        carousel = hardwareMap.get(DcMotor.class, "ca");
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        

        waitForStart();

        //robot_drive.move(24,0.75);


        robot_drive.move(24, 0.3);
        sleep(3000);

        carousel.setPower(0.5);
        sleep(3000);
        carousel.setPower(0.0);
        while(opModeIsActive()){

        }

    }
}
