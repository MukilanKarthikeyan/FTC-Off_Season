package org.firstinspires.ftc.teamcode.DC;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class DCone extends LinearOpMode {
    double leftPow, rightPow; //using the elevated drive train, 4 wheel drive
    double drive, turn;

    DcMotor rightFront, rightBack, leftFront, leftBack;
    DcMotor slides;


    boolean pad1DriveToggle, pad1DualStickDrive; //monostable circuit for switching between dual stick drive and right stick drive
    boolean pad2DriveToggle, pad2DualStickDrive;//same as above but for pad 2


    public void runOpMode() {

        //initialize all the declared hardware

        rightFront = hardwareMap.get(DcMotor.class, "rf"); // control motor port 0
        rightBack = hardwareMap.get(DcMotor.class, "rb"); // control motor port 1
        leftFront = hardwareMap.get(DcMotor.class, "lf");// control motor port 2
        leftBack = hardwareMap.get(DcMotor.class, "lb");// control motor port 3

        slides = hardwareMap.get(DcMotor.class, "slides");

        //reverse the direction of the motors needed so positive is the same direction for all
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        //flyWheel.setDirection(DcMotor.Direction.REVERSE);


        //establish 0 power behavior for all motors
        //we want the motors to stop running when the input power is 0
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));


        //establish that the power sent will be in the form of a float or a decimal
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        //these are the variables used for driving
        //the left and right pow is calculated based on the drive and turn
        leftPow = 0;
        rightPow = 0;
        drive = 0;
        turn = 0;


        pad1DriveToggle = false;
        pad2DriveToggle = false;

        pad1DualStickDrive = true;
        pad2DualStickDrive = true;

        //the code that runs in loop during the driver control period
        while(!isStopRequested()){

            //flywheel toggler: pressing "a" turns on the flywheel, pressing "a" again turns it off
            // NOTE: tried creating a separate method but does not work as intended


            //toogle for dual stick and right stick drive in game pad 1
            //NOTE: add one for right stick drive
            if(gamepad1.left_stick_button && pad1DriveToggle ){
                if(pad1DualStickDrive){ pad1DualStickDrive = false; }
                else{ pad1DualStickDrive = true;}
                pad1DriveToggle = false;
            }
            else if(!gamepad1.left_stick_button){pad1DriveToggle = true;}

            //same as the drive toggle for game pad 1 driving but implemented for game pad 2
            if( gamepad2.left_stick_button && pad2DriveToggle ){
                if(pad2DualStickDrive){ pad2DualStickDrive = false; }
                else{ pad2DualStickDrive = true; }
                pad2DriveToggle = false;
            }
            else if( !gamepad2.left_stick_button){pad2DriveToggle = true;}


            drive = Range.clip(gamepad1.left_stick_y + gamepad2.left_stick_y, -0.7,0.7);

            if(pad1DualStickDrive){ turn = gamepad1.right_stick_x; }
            else{ turn = gamepad1.left_stick_x; }
            if(pad2DualStickDrive){ turn += gamepad2.right_stick_x; }
            else{ turn = gamepad2.left_stick_x; }

            leftPow = drive - turn;
            rightPow = drive + turn;

            rightFront.setPower(rightPow);
            rightBack.setPower(rightPow);
            leftFront.setPower(leftPow);
            leftBack.setPower(leftPow);

            slides.setPower(gamepad1.right_trigger);
            slides.setPower(-gamepad1.left_trigger);

            telemetry.addData("2", "gamepad1 single stick drive:" + pad1DualStickDrive);
            telemetry.addData("3", "gamepad2 single stick drive:" + pad2DualStickDrive);
            telemetry.update();

        }
    }
}

