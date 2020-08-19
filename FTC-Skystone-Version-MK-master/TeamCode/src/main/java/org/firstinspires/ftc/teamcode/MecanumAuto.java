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

import java.util.Arrays;
@Autonomous
public class MecanumAuto extends LinearOpMode {

    DcMotor rightFront = null, rightBack = null, leftFront = null, leftBack = null;

    BNO055IMU imu;
    Orientation angles;
    double globAng;
    int rfPos, rbPos, lfPos, lbPos;
    double rfPow, rbPow, lfPow, lbPow;
    double rfScalPow, rbScalPow, lfScalPow, lbScalPow;
    double initTime, deltaTime;
    double Kp = 0.1, Ki = 0.001, i = 0, Kd = 0.001, d = 0, pre_error = 0, PIDpow;

    private final int TICKS = 1120;

    @Override
    public void runOpMode() throws InterruptedException {
        //assign each motor to the port in config
        rightFront = hardwareMap.dcMotor.get("rf");
        rightBack = hardwareMap.dcMotor.get("rb");
        leftFront = hardwareMap.dcMotor.get("lf");
        leftBack = hardwareMap.dcMotor.get("lb");

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
        /*
        freeDrive(0.3, 1, 1, -1, -1);
        brake(400);

        freeDrive(0.3,-1,1,-1,1);
        brake(400);

        freeDrive(0.3, -1, -1,1,1);
        brake(400);

        freeDrive(0.3, 1,-1,1,-1);
        brake(400);
        */

        //turning algorithm works as intended, hower overshoots but about one or two degrees
        //turn(90, true, 0, 0.3);
        //brake(1000);
        //turn(-5, true, 0, 0.3);
        //brake(1000);
        //turn(20, true, 0, 0.3);

        //freeDrive(0.3, 1, 1, -1, -1);
        //brake(1000);

        //freeDrive(0.3, -1, -1,1,1);
        //brake(1000);

        drive(0.3, 1, 1, -1, -1);
        brake(500);
        drive(0.3, -1, -1,1,1);
        brake(500);
        //freeDrive(0.2, 0.75, 0.75, -0.75, -0.75);
        //brake(400);

    }

    //this method can be used to test the encoders in the future in case encoders go out of phase
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

    /**
     * exactly like the drive method but but
     * the freeDrive method does not have a drifint correction applied to it
     *
     * @param pow: takes in the power that each motor will be set to
     * @param rfRev: number of revolutions for the right front motor
     * @param rbRev: number of revolutions for the right back motor
     * @param lfRev: number of revolutions for the left front motor
     * @param lbRev: number of revolutions for the left back motor
     */
    public void freeDrive(double pow, double rfRev, double rbRev, double lfRev,double lbRev){
        //need to change to a distace parameter
        //so for the parameters are the number of revolutions for each motor, but with future testing and aditional methods
        //which will make this freeDrive method a helper one, we can program based on distance rather than revolutions
        //currently distance is not fesibel due to inconsistancies(wheel slipage) and drifting(hardware issue)
        int rfTics = (int)(rfRev*TICKS);
        int rbTics = (int)(rbRev*TICKS);
        int lfTics = (int)(lfRev*TICKS);
        int lbTics = (int)(lbRev*TICKS);

        //to calculate the sign/diriction of the motor, yes its a long calculation there is posibiltiy for simplicfication
        //but currently we have not found a better solution
        double rfPow = (rfRev*pow)/(double)Math.abs(rfRev);
        double rbPow = (rbRev*pow)/(double)Math.abs(rbRev);
        double lfPow = (lfRev*pow)/(double)Math.abs(lfRev);
        double lbPow = (lbRev*pow)/(double)Math.abs(lbRev);

        double rfScalPow = 0, rbScalPow = 0, lfScalPow = 0, lbScalPow = 0;

        //int tic = (dist/(wheel_diameter))*TICKS;
        //NOTE: rev = dist/(wheel_diameter)
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double initAng = Math.abs(angles.firstAngle);

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

        //set to RUN_USING_ENCODERS before setting target postion
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setTargetPosition(rfTics);
        rightBack.setTargetPosition(rbTics);
        leftFront.setTargetPosition(lfTics);
        leftBack.setTargetPosition(lbTics);

        rfPos = rightFront.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();
        lfPos = leftFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while loop must use || because when using && if a revolution of 0 is given to one motor, the statement
        //short ciruits and stops, but the || allows to run each motor at individual revolutions
        while((Math.abs(rfPos) < Math.abs(rfTics)) || (Math.abs(rbPos) < Math.abs(rbTics))||
                (Math.abs(lfPos) < Math.abs(lfTics))|| (Math.abs(lbPos) < Math.abs(lbTics))){

            //make a scaleing helper method, for simplicity
            if((Math.abs(rfPos) < Math.abs(rfTics))){
                rfScalPow = 0.5*Range.clip(rfPow*((double)Math.abs(rfTics)-(double)(Math.abs(rfPos) )/(double)Math.abs(rfTics)),-pow,pow);
                rightFront.setPower(rfScalPow);
            }
            if((Math.abs(rbPos) < Math.abs(rbTics))){
                rbScalPow = 0.5*Range.clip(rbPow*((double)Math.abs(rbTics)-(double)(Math.abs(rbPos))/(double)Math.abs(rbTics)),-pow,pow);
                rightBack.setPower(rbScalPow);
            }
            if((Math.abs(lfPos) < Math.abs(lfTics))){
                lfScalPow = 0.5*Range.clip(lfPow*((double)Math.abs(lfTics)-(double)(Math.abs(lfPos))/(double)Math.abs(lfTics)),-pow,pow);
                leftFront.setPower(lfScalPow);
            }
            if((Math.abs(lbPos) < Math.abs(lbTics))){
                lbScalPow = 0.5*Range.clip(lbPow*((double)Math.abs(lbTics)-(double)(Math.abs(lbPos))/(double)Math.abs(lbTics)),-pow,pow);
                leftBack.setPower(lbScalPow);
            }

            //unable to use the powDrive method beacue i need to be able to control each of the power of the motors independitly which would
            //require me to write 4 seperate lines of code
            //powDrive(rfPow, rbPow, lfPow, lbPow);

            rfPos = rightFront.getCurrentPosition();
            rbPos = rightBack.getCurrentPosition();
            lfPos = leftFront.getCurrentPosition();
            lbPos = leftBack.getCurrentPosition();

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curAng = Math.abs(angles.firstAngle);

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
    public void drive(double pow, double rfRev, double rbRev, double lfRev,double lbRev){
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

        rightFront.setTargetPosition(rfTics);
        rightBack.setTargetPosition(rbTics);
        leftFront.setTargetPosition(lfTics);
        leftBack.setTargetPosition(lbTics);

        rfPos = rightFront.getCurrentPosition();
        rbPos = rightBack.getCurrentPosition();
        lfPos = leftFront.getCurrentPosition();
        lbPos = leftBack.getCurrentPosition();

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //while loop must use || because when using && if a revolution of 0 is given to one motor, the statement
        //short ciruits and stops, but the || allows to run each motor at individual revolutions
        while((Math.abs(rfPos) < Math.abs(rfTics)) || (Math.abs(rbPos) < Math.abs(rbTics))||
                (Math.abs(lfPos) < Math.abs(lfTics))|| (Math.abs(lbPos) < Math.abs(lbTics))){

            // implement constatant drift correction/ collision compensation using the IMU values
            //use this same principal for diagonal movemtn, and also use encoder values to see if the tics move, and reserse to compensate
            //for unnecceray movement
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double curAng = angles.firstAngle;

            initTime = getRuntime();
            double target = 0;
            double error = curAng - target;
            double correction = (Kp*error);


            if((Math.abs(rfPos) < Math.abs(rfTics))){
                rfScalPow =Range.clip(rfPow*((double)Math.abs(rfTics)-(double)(Math.abs(rfPos))/(double)Math.abs(rfTics)),-pow,pow)-correction;
                rightFront.setPower(rfScalPow);
            }
            if((Math.abs(rbPos) < Math.abs(rbTics))){
                rbScalPow = Range.clip(rbPow*((double)Math.abs(rbTics)-(double)(Math.abs(rbPos))/(double)Math.abs(rbTics)),-pow,pow)-correction;
                rightBack.setPower(rbScalPow);
            }
            if((Math.abs(lfPos) < Math.abs(lfTics))){
                lfScalPow = Range.clip(lfPow*((double)Math.abs(lfTics)-(double)(Math.abs(lfPos))/(double)Math.abs(lfTics)),-pow,pow)-correction;
                leftFront.setPower(lfScalPow);
            }
            if((Math.abs(lbPos) < Math.abs(lbTics))){
                lbScalPow = Range.clip(lbPow*((double)Math.abs(lbTics)-(double)(Math.abs(lbPos))/(double)Math.abs(lbTics)),-pow,pow)-correction;
                leftBack.setPower(lbScalPow);
            }

            rfPos = rightFront.getCurrentPosition();
            rbPos = rightBack.getCurrentPosition();
            lfPos = leftFront.getCurrentPosition();
            lbPos = leftBack.getCurrentPosition();
            /*
            deltaTime = getRuntime() - initTime;
            if (Math.abs(error) < 30)
                i += Ki * error * deltaTime;

            if (i > 0.3) {
                i = 0.3;
            }
            d = (Kd*(error-pre_error)/deltaTime);*/
            pre_error = error;
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
            telemetry.addData("7", "error: ", Double.toString(error));
            //telemetry.addData("8", "p: ", Double.toString((Kp*error)));
            //telemetry.addData("9", "i: ", Double.toString(i));
            //telemetry.addData("10", "d: ", Double.toString(d));
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

    /**
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
            teLogic(X_move, Y_move);
        }
        //else statement deals with if you want to turn and drive forward
        //slightly complicated as we need to accomedate for large turns using reference angles

    }

    /*
    placeholer drive method
    */
    public void teLogic(double X_move,double Y_move){
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
