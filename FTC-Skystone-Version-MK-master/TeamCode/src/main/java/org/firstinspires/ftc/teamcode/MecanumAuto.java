package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Arrays;
@Autonomous
public class MecanumAuto extends LinearOpMode {

    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftBack = null;
    DcMotor rightBack = null;

    BNO055IMU imu;
    Orientation angles;

    private final int TICKS = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        //assign each motor
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");

        waitForStart();
        //fwd(1, 0.3);
        //DrivePower(0);
        //strafe(1,0.3);

        //forward
        /*
        freeDrive(0.6,-0.6,-0.6,0.6);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);
        //backward
        freeDrive(-0.6,0.6,0.6,-0.6);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);

        freeDrive(0.6,0.6,-0.6,-0.6);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);

        freeDrive(-0.6,-0.6,0.6,0.6);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);

        freeDrive(-0.6,-0.6,-0.6,-0.6);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);

        freeDrive(0.6,0.6,0.6,0.6);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);

        freeDrive(0.6,0.0,0.6,0.0);
        sleep(1000);

        freeDrive(0.0,0.0,0.0,0.0);
        sleep(500);

        freeDrive(0.0,0.6,0.0,0.6);
        sleep(1000);

        */
        move_Y(1,0.1);
    }

    public void testEncoder(int rev){
        int tics = TICKS*(rev);
        telemetry.addData("1", "motorRightFront: " + String.format("%d", rightFront.getCurrentPosition()));
        telemetry.update();

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);

        rightFront.setTargetPosition(-tics);
        int rfPos = rightFront.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(rfPos) < Math.abs(tics)){
            rightFront.setPower(0.1);
            rfPos = rightFront.getCurrentPosition();
            telemetry.addData("1", "motorRightFront: " + String.format("%d", rightFront.getCurrentPosition()));
            telemetry.update();
        }


    }

    public void move_Y(double rev, double pow){//need to change to a distace parameter
        int tics = (int)rev*TICKS;
        //int tic = (dist/(wheel_diameter))*TICKS;
        //NOTE: rev = dist/(wheel_diameter)

        telemetry.addData("1", "motorRightFront: " + String.format("%d", rightFront.getCurrentPosition()));
        telemetry.addData("2", "motorRightFront: " + String.format("%d", rightBack.getCurrentPosition()));
        telemetry.addData("3", "motorRightFront: " + String.format("%d", leftFront.getCurrentPosition()));
        telemetry.addData("4", "motorRightFront: " + String.format("%d", leftBack.getCurrentPosition()));
        telemetry.update();

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        rightFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RESET_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RESET_ENCODERS);

        rightFront.setTargetPosition(tics);
        rightBack.setTargetPosition(tics);
        leftFront.setTargetPosition(-tics);
        leftBack.setTargetPosition(-tics);

        int rfPos = rightFront.getCurrentPosition();
        int rbPos = rightBack.getCurrentPosition();
        int lfPos = leftFront.getCurrentPosition();
        int lbPos = leftBack.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((Math.abs(rfPos) < Math.abs(tics)) && (Math.abs(rbPos) < Math.abs(tics)) &&
                (Math.abs(lfPos) < Math.abs(tics)) && (Math.abs(lbPos) < Math.abs(tics))){

            freeDrive(-pow, pow, pow, -pow);

            rfPos = rightFront.getCurrentPosition();
            rbPos = rightBack.getCurrentPosition();
            lfPos = leftFront.getCurrentPosition();
            lbPos = leftBack.getCurrentPosition();

            telemetry.addData("1", "motorRightFront: " + String.format("%d", rightFront.getCurrentPosition()));
            telemetry.addData("2", "motorRightFront: " + String.format("%d", rightBack.getCurrentPosition()));
            telemetry.addData("3", "motorRightFront: " + String.format("%d", leftFront.getCurrentPosition()));
            telemetry.addData("4", "motorRightFront: " + String.format("%d", leftBack.getCurrentPosition()));
            telemetry.update();
        }
    }

    public void turn(int tics, boolean turn){
        //rev is the number of revolutions of the motor(need to change it to distance in meters/inches so it will calculate revs)
        //set the distace goal
        rightFront.setTargetPosition(tics);
        leftFront.setTargetPosition(tics);
        leftBack.setTargetPosition(tics);
        rightBack.setTargetPosition(tics);

        //change to run to position mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power to motors
        if(turn){
            freeDrive(0.3,-0.3,-0.3,0.3);}
        else{
            freeDrive(-0.3,0.3,0.3,-0.3);
        }
        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack
                .isBusy()) {
        }
        DrivePower(0.0);
    }

    /*
    sets the power for each wheel
    inputs range form -1.0 to 1.0
     */
    public void DrivePower(double pow) {
        leftFront.setPower(pow);
        leftBack.setPower(pow);
        rightFront.setPower(pow);
        rightBack.setPower(pow);
    }

    public void freeDrive(double rf, double lf,  double lb, double rb){
        rightFront.setPower(rf);
        leftFront.setPower(lf);
        rightBack.setPower(rb);
        leftBack.setPower(lb);
    }
    /*
    the strafe method is for moving on the Horzintal axis reltive to the robot
     */
    public void strafe(int rev, double pow){
        // if dist is positive strafes right, if dist is negative strafes left
        int dist = (int)rev*TICKS;
        //int tic = (dist/(wheel_diameter))*TICKS;
        //NOTE: rev = dist/(wheel_diameter)

        //rev is the number of revolutions of the motor(need to change it to distance in meters/inches so it will calculate revs)
        //set the distace goal
        //a postive rev value strafes right, and a negative rev value strafes left
        leftFront.setTargetPosition(dist);
        leftBack.setTargetPosition(-dist);
        rightFront.setTargetPosition(dist);
        rightBack.setTargetPosition(-dist);

        //change to run to position mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power to motors
        freeDrive(pow,pow,-pow,-pow);

        while (leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack
                .isBusy()) {
        }
        DrivePower(0.0);
    }

    /*
    drivePolar method takes in an input of distance, angle, and boolean mainatin orentaion(mainOr)
    if mainOr is true, then robot does not turn, if false robot turns and moves forward
    NOTE: since its holonomic, assign an integer to each side of the robot (easer to work with in the future
    depending on attachments and use a waterfall conditional)
     */
    public void drivePolar(double dist, double angle, boolean mainOr, double pow){
        //Calculate Horizontal Movement
        double X_move = dist*Math.cos(angle);

        //Calculate Veritcal movement
        double Y_move = dist*Math.sin(angle);
        //initially we need to test the movemnt by breaking down the movement vector into its x and y components

        //no need to check for refernce angle when maintaing Orentaion, works like graphing in Polar coordinates
        if(mainOr){
            drive(X_move, Y_move);
        }
        //else statement deals with if you want to turn and drive forward
        //slightly complicated as we need to accomedate for large turns using reference angles

    }

    /*
    placeholer drive method
    */
    public void drive(double X_move,double Y_move){
        double Omega1 = X_move + Y_move;//rf
        double Omega2 = X_move - Y_move;//lf
        double Omega3 = X_move + Y_move;//lb
        double Omega4 = X_move - Y_move;//rb
    }

    /*
    the folowing driveMethod is for cartesian coordinates
    The logic used in this command to determine the power assigned to each motor is the
    same as the logic for the teleOp code and matches input ranges form controller

    NOTE: Program the TeleOp to be field centric for best use
     */
    public void driveXY(int x, int y, double turn) {
        //calculations for the power to be applied to each wheel
        // NOTE: these are what would be used for the telOP inputs form controller
        double leftFrontVal = y + turn + x;
        double leftBackVal = y + turn - x;
        double rightFrontVal = y - turn - x;
        double rightBackVal = y - turn + x;

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }
}
