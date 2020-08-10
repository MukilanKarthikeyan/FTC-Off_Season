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
        drive(0.3, 1, -1, 1, -1);
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

    public void drive(double pow, double rfRev, double rbRev, double lfRev,double lbRev){//need to change to a distace parameter
        int rfTics = (int)rfRev*TICKS;
        int rbTics = (int)rbRev*TICKS;
        int lfTics = (int)lfRev*TICKS;
        int lbTics = (int)lbRev*TICKS;

        double rfPow = (rfRev*pow)/(double)Math.abs(rfRev);
        double rbPow = (rbRev*pow)/(double)Math.abs(rbRev);
        double lfPow = (lfRev*pow)/(double)Math.abs(lfRev);
        double lbPow = (lbRev*pow)/(double)Math.abs(lbRev);
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

        rightFront.setTargetPosition(rfTics);
        rightBack.setTargetPosition(rbTics);
        leftFront.setTargetPosition(lfTics);
        leftBack.setTargetPosition(lbTics);

        int rfPos = rightFront.getCurrentPosition();
        int rbPos = rightBack.getCurrentPosition();
        int lfPos = leftFront.getCurrentPosition();
        int lbPos = leftBack.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while((Math.abs(rfPos) < Math.abs(rfTics)) && (Math.abs(rbPos) < Math.abs(rbTics)) &&
                (Math.abs(lfPos) < Math.abs(lfTics)) && (Math.abs(lbPos) < Math.abs(lbTics))){

            powDrive(rfPow, rbPow, lfPow, lbPow);

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

    public void powDrive(double rf, double lf,  double lb, double rb){
        rightFront.setPower(rf);
        leftFront.setPower(lf);
        rightBack.setPower(rb);
        leftBack.setPower(lb);
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
