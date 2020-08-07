package org.firstinspires.ftc.teamcode;

import com.hardware.bosch.BNO055IMU;
import com.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.robotcore.eventloop.opmode.Autonomous;
import com.robotcore.eventloop.opmode.Disabled;
import com.robotcore.eventloop.opmode.LinearOpMode;
import com.robotcore.hardware.DcMotor;
import com.robotcore.hardware.DcMotorController;
import com.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        fwd(1);
    }
    public void fwd(double rev){//need to change to a distace perameter
        int dist = rev*TICKS;
        //int tic = (dist/(wheel_diameter))*TICKS;
        //NOTE: rev = dist/(wheel_diameter)

        //Stop and Reset the encoders
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rev is the number of revolutions of the motor(need to change it to distance in meters/inches so it will calculate revs)
        //set the distace goal
        leftFront.setTargetPosition(dist);
        leftBack.setTargetPosition(dist);
        rightFront.setTargetPosition(dist);
        rightBack.setTargetPosition(dist);

        //change to run to position mode
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power to motors
        DrivePower(0.3);

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
    /*
    the strafe method is for moving on the Horzintal axis reltive to the robot
     */
    public void strafe(int dist){
        // if dist is positive strafes right, if dist is negative strafes left
    }

    /*
    drivePolar method takes in an input of distance, angle, and boolean mainatin orentaion(mainOr)
    if mainOr is true, then robot does not turn, if false robot turns and moves forward
    NOTE: since its holonomic, assign an integer to each side of the robot (easer to work with in the future
    depending on attachments and use a waterfall conditional)
     */
    public void drivePolar(double dist, double angle, boolean mainOr, double pow){
        //Calculate Horizontal Movement
        double X_move = dist*Cos(angle);

        //Calculate Veritcal movement
        double Y_move = dist*Sin(angle);
        //initially we need to test the movemnt by breaking down the movement vector into its x and y components

        //no need to check for refernce angle when maintaing Orentaion, works like graphing in Polar coordinates
        if(mainOr){
            drive(X_move);
            drive(Y_move;
        }
        //else statement deals with if you want to turn and drive forward
        //slightly complicated as we need to accomedate for large turns using reference angles
        else{
            turn(angle); // placeholder command, but write the turn command
            move(dist);
        }
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


