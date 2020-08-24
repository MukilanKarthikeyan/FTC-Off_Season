package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.Range;
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
import java.util.*;
@Autonomous
public class MecanumAuto extends LinearOpMode {

    DcMotor rightFront = null, rightBack = null, leftFront = null, leftBack = null;

    BNO055IMU imu;
    Orientation angles;
    double globAng;
    int rfPos, rbPos, lfPos, lbPos;
    double rfPow, rbPow, lfPow, lbPow;
    double rfRev, rbRev, lfRev, lbRev;
    double rfScalPow, rbScalPow, lfScalPow, lbScalPow;
    double initTime, deltaTime;
    double Kp = 0.01, Ki = 0.01, i = 0, Kd = 0.001, d = 0, pre_error = 0, PIDpow;

    private final int TICKS = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        //assign each motor to the port in config
        rightFront = hardwareMap.dcMotor.get("rf");
        rightBack = hardwareMap.dcMotor.get("rb");
        leftFront = hardwareMap.dcMotor.get("lf");
        leftBack = hardwareMap.dcMotor.get("lb");

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // initializes the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();

    }

    /**
     * Mukilan K. 2020.08.11
     * drive allows control over direction and number of rotations of each indivitual motor
     * also fixed limitations form previous drive method which required each motor to spin the same non zero amount of times
     * would always spin the least number of revolution int he parameter (previous method will be replaced once freeDrive is tested)
     *
     * @param pow: takes in the power that each motor will be set to
     * @param rfRev: number of revolutions for the right front motor
     * @param rbRev: number of revolutions for the right back motor
     * @param lfRev: number of revolutions for the left front motor
     * @param lbRev: number of revolutions for the left back motor
     */
    public void drive(double pow, double rfRev, double rbRev, double lfRev,double lbRev, double target){
        //need to change to a distace parameter
        //so for the parameters are the number of revolutions for each motor, but with future testing and aditional methods
        //which will make this freeDrive method a helper one, we can program based on distance rather than revolutions
        //currently distance is not fesibel due to inconsistancies(wheel slipage) and drifting(hardware issue)
        int rfTics = (int)(rfRev*TICKS);
        int rbTics = (int)(rbRev*TICKS);
        int lfTics = (int)(lfRev*TICKS);
        int lbTics = (int)(lbRev*TICKS);

        rfPow = (rfRev*(pow))/(double)Math.abs(rfRev);
        rbPow = (rbRev*(pow))/(double)Math.abs(rbRev);
        lfPow = (lfRev*(pow))/(double)Math.abs(lfRev);
        lbPow = (lbRev*(pow))/(double)Math.abs(lbRev);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initAng = Math.abs(angles.firstAngle);
        //int tic = (dist/(wheel_diameter))*TICKS;
        //NOTE: rev = dist/(wheel_diameter)

        telemetry.addData("1", "motorRightFront: " + String.format("%d", rightFront.getCurrentPosition())
                + " target: " + String.format("%d", rfTics));
        telemetry.addData("2", "motorRightFront: " + String.format("%d", rightBack.getCurrentPosition())
                + " target: "+String.format("%d", rbTics));
        telemetry.addData("3", "motorRightFront: " + String.format("%d", leftFront.getCurrentPosition())
                + " target: "+String.format("%d", lfTics));
        telemetry.addData("4", "motorRightFront: " + String.format("%d", leftBack.getCurrentPosition())
                + " target: "+String.format("%d", lbTics));
        telemetry.addData("5", "intial angle: " + Double.toString(initAng));
        telemetry.update();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initAng = Math.abs(angles.firstAngle);

        //set to RUN_USING_ENCODERS before setting target postion
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rfPos = rightFront.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();
        lfPos = leftFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        //while loop must use || because when using && if a revolution of 0 is given to one motor, the statement
        //short ciruits and stops, but the || allows to run each motor at individual revolutions
        while((Math.abs(rfPos) < Math.abs(rfTics)) || (Math.abs(rbPos) < Math.abs(rbTics))||
                (Math.abs(lfPos) < Math.abs(lfTics))|| (Math.abs(lbPos) < Math.abs(lbTics))){

            // implement constatant drift correction/ collision compensation using the IMU values
            //use this same principal for diagonal movemtn, and also use encoder values to see if the tics move, and reserse to compensate
            //for unnecceray movement
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curAng = angles.firstAngle;

            //initTime = getRuntime();
            //double target = 0;
            double error = curAng - target;
            double correction = (Kp*error);

            rfPos = rightFront.getCurrentPosition();
            rbPos = rightBack.getCurrentPosition();
            lfPos = leftFront.getCurrentPosition();
            lbPos = leftBack.getCurrentPosition();

            if((Math.abs(rfPos) > Math.abs(rfTics))){
                rfScalPow = -1 * (Range.clip(rfPow*((double)Math.abs(rfTics)-(double)(Math.abs(rfPos))/(double)Math.abs(rfTics)),-pow,pow)-correction);
            }
            if((Math.abs(rfPos) < Math.abs(rfTics))){
                rfScalPow = Range.clip(rfPow*((double)Math.abs(rfTics)-(double)(Math.abs(rfPos))/(double)Math.abs(rfTics)),-pow,pow)-correction;
            }else{ rfScalPow = 0.0; }

            if((Math.abs(rbPos) > Math.abs(rbTics))){
                rbScalPow = -1 * (Range.clip(rbPow*((double)Math.abs(rbTics)-(double)(Math.abs(rbPos))/(double)Math.abs(rbTics)),-pow,pow)-correction);
            }
            if((Math.abs(rbPos) < Math.abs(rbTics))){
                rbScalPow = Range.clip(rbPow*((double)Math.abs(rbTics)-(double)(Math.abs(rbPos))/(double)Math.abs(rbTics)),-pow,pow)-correction;
            }else{ rbScalPow = 0.0; }

            if((Math.abs(lfPos) > Math.abs(lfTics))){
                lfScalPow = -1 * (Range.clip(lfPow*((double)Math.abs(lfTics)-(double)(Math.abs(lfPos))/(double)Math.abs(lfTics)),-pow,pow)-correction);
            }
            if((Math.abs(lfPos) < Math.abs(lfTics))){
                lfScalPow = Range.clip(lfPow*((double)Math.abs(lfTics)-(double)(Math.abs(lfPos))/(double)Math.abs(lfTics)),-pow,pow)-correction;
            }else{ lfScalPow = 0.0; }

            if((Math.abs(lbPos) < Math.abs(lbTics))){
                lbScalPow = -1 * (Range.clip(lbPow*((double)Math.abs(lbTics)-(double)(Math.abs(lbPos))/(double)Math.abs(lbTics)),-pow,pow)-correction);
            }
            if((Math.abs(lbPos) < Math.abs(lbTics))){
                lbScalPow = Range.clip(lbPow*((double)Math.abs(lbTics)-(double)(Math.abs(lbPos))/(double)Math.abs(lbTics)),-pow,pow)-correction;
            } else{ lbScalPow = 0.0; }

            powDrive(rfScalPow,rbScalPow,lfScalPow,lbScalPow);

            /*
            deltaTime = getRuntime() - initTime;
            if (Math.abs(error) < 30){ i += Ki * error * deltaTime;}
            if (i > 0.3) { i = 0.3;}
            d = (Kd*(error-pre_error)/deltaTime);
            pre_error = error;*/
            telemetry.addData("1", "motorRightFront: " + String.format("%d", rightFront.getCurrentPosition())
                    + " target: " + String.format("%d", rfTics)
                    + " power: " + Double.toString(Math.round(rfScalPow*100)/100.0));
            telemetry.addData("2", "motorRightFront: " + String.format("%d", rightBack.getCurrentPosition())
                    + " target: "+String.format("%d", rbTics)
                    + " power: " + Double.toString(Math.round(rbScalPow*100)/100.0));
            telemetry.addData("3", "motorRightFront: " + String.format("%d", leftFront.getCurrentPosition())
                    + " target: "+String.format("%d", lfTics)
                    + " power: " + Double.toString(Math.round(lfScalPow*100)/100.0));
            telemetry.addData("4", "motorRightFront: " + String.format("%d", leftBack.getCurrentPosition())
                    + " target: "+String.format("%d", lbTics)
                    + " power: " + Double.toString(Math.round(lbScalPow*100)/100.0));
            telemetry.addData("5", "intial angle: " + Double.toString(initAng));
            telemetry.addData("6", "current angle: " + Double.toString(curAng));
            telemetry.addData("7", "error: " + Double.toString(error));
            telemetry.addData("8", "correction: " + Double.toString(correction));
            telemetry.update();
        }
        //need to reset the mode of the motors so that it can work with other methods such as powDrive
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //stops setting power to the motors after the target position has been reached
        //beffore it used to just stop because the loop was exited
        brake(100);
        //angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //double endAng = Math.abs(angles.firstAngle);
        //turn(endAng-initAng, ((endAng-initAng)<0), 0, 0.3);
        //brake(100);

    }

    //Turn method to be written after studying the IMU, first work on robot motion
    /**
     * if edge is 0 then it pivots about the center
     * for edges 1-4
     * this pivots on the midpoint of the specifide edge
     * however with these cases, if the boolean sideCon is true, it pivots on midpoint of edges,
     * if false pivots on one of the wheels (starting form right front wheel and going clockwise)
     * 1: right front wheel and labled clockwise
     *
     * @param target: if positive: clockwise if negative: counterclockwise
     *             this method is not motor encoder based, must write it based on an IMU PID loop
     * @param edge: 0 is center, side 1 is the front, sides are labled clockwise
     * @param pow: if positive -> anitclockwise, if pow is regative -> clockwise
     */
    public void turn(double target, boolean sideCon, int edge, double pow){
        double  offset = 90;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initialPos = angles.firstAngle;

        double currAng = angles.firstAngle;

        double error = offset - Math.abs(target);
        pow = (target*pow)/(Math.abs(target));

        while (opModeIsActive() && ((Math.abs(target) - Math.abs(currAng)) > 3)) {
            initTime = getRuntime();
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currAng = Math.abs(angles.firstAngle);

            error = currAng - Math.abs(target);
            double correction = (Kp*Math.abs(error)) + i + d;

            //pow = Range.clip(pow*(Math.abs((curAng - Math.abs(target)) / (100.0)) + i), -0.7, .7);


            telemetry.addData("Current Position: ", currAng);
            telemetry.addData("Distance to go: ", error);
            telemetry.update();

            PIDpow = pow + correction;
            if(sideCon) {
                switch (edge) {
                    case 0:
                        powDrive(pow, pow, pow, pow);
                        break;
                    case 1:
                        powDrive(0.0, pow, 0.0, pow);
                        break;
                    case 2:
                        powDrive(0.0, 0.0, pow, pow);
                        break;
                    case 3:
                        powDrive(pow, 0.0, pow, 0.0);
                        break;
                    case 4:
                        powDrive(pow, pow, 0.0, 0.0);
                        break;
                }
            }
            else{
                switch(edge){
                    case 1:
                        powDrive(0.0, pow, pow, pow);
                        break;
                    case 2:
                        powDrive(pow,0.0,pow,pow);
                        break;
                    case 3:
                        powDrive(pow, pow,0.0, pow);
                        break;
                    case 4:
                        powDrive(pow, pow, pow, 0.0);
                        break;
                }
            }
            deltaTime = getRuntime() - initTime;
            if (Math.abs(error) < 30)
                i += Ki * Math.abs(error) * deltaTime;

            if (i > 0.3) {
                i = 0.3;
            }
            d = (Kd*(Math.abs(error)-pre_error));
        }
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brake(100);
    }

    /**
     * takes in a specfic speed for each individutal motor
     * Mukilan K. 2020.08.07
     *
     * @param rf: right front motor power
     * @param rb: right back motor power
     * @param lf: left front motor power
     * @param lb: left back motor power
     */
    public void powDrive(double rf, double rb, double lf,  double lb){
        rightFront.setPower(rf);
        leftFront.setPower(lf);
        rightBack.setPower(rb);
        leftBack.setPower(lb);
    }

    public void brake(int time){
        powDrive(0.0,0.0,0.0,0.0);
        sleep(time);
    }

}
