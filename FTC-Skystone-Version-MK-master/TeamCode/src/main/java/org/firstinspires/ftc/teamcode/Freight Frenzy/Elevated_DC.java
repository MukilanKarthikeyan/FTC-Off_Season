package org.firstinspires.ftc.teamcode.DC;

        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Elevated_DC extends LinearOpMode {
    double leftPow, rightPow; //using the elevated drive train, 4 wheel drive
    double drive, turn;
    double caroPow;

    DcMotor rightFront, rightBack, leftFront, leftBack;
    DcMotor carousel;
    DcMotor slides;

    Servo clamp;

    boolean pad1DriveToggle, pad1DualStickDrive; //monostable circuit for switching between dual stick drive and right stick drive
    boolean pad2DriveToggle, pad2DualStickDrive;//same as above but for pad 2
    boolean allaiance; // if true then red, if false then blue

    public void runOpMode() {

        //initialize all the declared hardware

        rightFront = hardwareMap.get(DcMotor.class, "rf"); // control motor port
        rightBack = hardwareMap.get(DcMotor.class, "rb"); // control motor port
        leftFront = hardwareMap.get(DcMotor.class, "lf");// control motor port
        leftBack = hardwareMap.get(DcMotor.class, "lb");// control motor port

        //slides = hardwareMap.get(DcMotor.class, "slide");
        clamp = hardwareMap.get(Servo.class, "clamp");

        //reverse the direction of the motors needed so positive is the same direction for all
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        //flyWheel.setDirection(DcMotor.Direction.REVERSE);


        //establish 0 power behavior for all motors
        //we want the motors to stop running when the input power is 0
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //establish that the power sent will be in the form of a float or a decimal
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        carousel = hardwareMap.get(DcMotor.class, "ca");
        carousel.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
        carousel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);





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

            if(gamepad1.back && gamepad1.b){
                allaiance = true;
            }
            if(gamepad1.back && gamepad1.x){
                allaiance = false;
            }


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

/*
            if(gamepad1.left_trigger > 0){
                slides.setPower(-gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger > 0){
                slides.setPower(gamepad1.right_trigger);
            }
*/
            if ((gamepad1.x && !gamepad1.back) || (gamepad2.x && !gamepad2.back)){
                clamp.setPosition(0.8);
            }
            if (gamepad1.y|| gamepad2.y){
                clamp.setPosition(0.2);
            }

            drive = Range.clip(gamepad1.left_stick_y + gamepad2.left_stick_y, -0.9,0.9);
            caroPow = Range.clip(gamepad1.left_trigger + gamepad2.left_trigger, 0.0,0.7);

/*            if(gamepad1.a || gamepad2.a && !(gamepad1.right_trigger > 0)){
                if(allaiance){
                    carousel.setPower(0.5);
                }
                else {
                    carousel.setPower(-0.5);
                }

            }
            else if (gamepad1.right_trigger > 0){
                if(allaiance){
                    carousel.setPower(gamepad1.right_trigger);
                }
                else {
                    carousel.setPower(-gamepad1.right_trigger);
                }
            }

            */

            if(gamepad1.right_bumper){
                carousel.setPower(0.5);
            }
            if (gamepad1.left_bumper ){
                carousel.setPower(-0.5);
            }
            if(!gamepad1.right_bumper && !gamepad2.left_bumper){
                carousel.setPower(0.0);
            }


            if(pad1DualStickDrive){ turn = gamepad1.right_stick_x; }
            else{ turn = gamepad1.left_stick_x; }
            if(pad2DualStickDrive){ turn += gamepad2.right_stick_x; }
            else{ turn = gamepad2.left_stick_x; }

            leftPow = drive + turn;
            rightPow = drive - turn;

            rightFront.setPower(rightPow);
            rightBack.setPower(rightPow);
            leftFront.setPower(leftPow);
            leftBack.setPower(leftPow);

            telemetry.addData("2", "gamepad1 single stick drive:" + pad1DualStickDrive);
            telemetry.addData("3", "gamepad2 single stick drive:" + pad2DualStickDrive);

            if(allaiance){
                telemetry.addData("allaiance", "red");
            }
            else{
                telemetry.addData("allaiance", "blue");
            }
            telemetry.update();

        }
    }
}

