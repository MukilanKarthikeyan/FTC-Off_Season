package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
public class EleDrive {
    LinearOpMode opmode;

    DcMotor right_FrontM = null;
    DcMotor right_BackM = null;
    DcMotor left_FrontM = null;
    DcMotor left_BackM = null;

    double right_power;
    double left_power;
    double motor_power;

    /*both right motors move are liked and both left motors are linked so each side only have one encoder
    * NOTE: three wheel turns are not possible with the TileRunner*/
    double right_encoder_counts;
    double left_encoder_counts;

    BNO055IMU imu;
    Orientation angles;

    int target_counts;

    //wheel radius is in inches
    double wheel_radius = 2;
    double wheel_circumfrence = 2*Math.PI*wheel_radius;
    private final int TICKS_PER_MOTOR_REV = 1120;
    private final  double wheel_to_motor_ratio = 3.0/2.0;
    private final int TICKS_PER_WHEEL_REV = (int)(TICKS_PER_MOTOR_REV*wheel_to_motor_ratio);


    /**
     * constructor method: initializes all motors on robot
     * sets motor behavior and run modes
     * @param mode
     */
    public EleDrive(LinearOpMode mode){
        opmode = mode;

        right_FrontM = opmode.hardwareMap.get(DcMotor.class, "rf"); // control motor port
        right_BackM = opmode.hardwareMap.get(DcMotor.class, "rb"); // control motor port
        left_FrontM = opmode.hardwareMap.get(DcMotor.class, "lf");// control motor port
        left_BackM = opmode.hardwareMap.get(DcMotor.class, "lb");// control motor port

        right_FrontM.setDirection(DcMotor.Direction.FORWARD);
        right_BackM.setDirection(DcMotor.Direction.FORWARD);
        left_FrontM.setDirection(DcMotor.Direction.REVERSE);
        left_BackM.setDirection(DcMotor.Direction.REVERSE);

        right_FrontM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_BackM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_FrontM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_BackM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        right_FrontM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_BackM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_FrontM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_BackM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        right_FrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_BackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_FrontM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_BackM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setUpIMU();

    }

    /**
     * this method uses
     * @param dist the distance in inches (positive distance moves forward, negative distance moves backward.)
     * @param power the power that is to be applied to the motors
     */
    public void move(double dist, double power){
        int tics = (int)((dist/(wheel_circumfrence*wheel_to_motor_ratio))*TICKS_PER_WHEEL_REV);
        int right_target = Math.abs(right_BackM.getCurrentPosition()) + tics;
        int left_target = Math.abs(left_BackM.getCurrentPosition()) + tics;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current_angle = angles.firstAngle;


        /*(Math.abs(right_BackM.getCurrentPosition())-Math.abs(right_target) > 45) &&
                (Math.abs(left_BackM.getCurrentPosition())-Math.abs(left_target) > 45) &&
                    !opmode.isStopRequested()*/
        if(dist > 0){
            right_power = power;
            left_power = power;
        }
        else{
            right_power = -power;
            left_power = -power;
        }
        while((Math.abs(Math.abs(right_BackM.getCurrentPosition())-Math.abs(right_target)) > 20) &&
                (Math.abs(Math.abs(left_BackM.getCurrentPosition())-Math.abs(left_target)) > 20) &&
                !opmode.isStopRequested()){


            right_FrontM.setPower(right_power);
            right_BackM.setPower(right_power);
            left_FrontM.setPower(left_power);
            left_BackM.setPower(left_power);

            //must add acceleration nd deceleration
            //double rp = right_power *(Math.abs(Math.abs(right_BackM.getCurrentPosition())-Math.abs(right_target))/tics);
            //double lp = right_power *(Math.abs(Math.abs(left_BackM.getCurrentPosition())-Math.abs(left_target))/tics);
            opmode.telemetry.addData("tics", tics);
            opmode.telemetry.addData("angle", current_angle);
            opmode.telemetry.addData("Current Position", Math.abs(right_BackM.getCurrentPosition()));
            opmode.telemetry.addData("Distance to go", Math.abs(Math.abs(right_BackM.getCurrentPosition()) - Math.abs(right_target)));
            opmode.telemetry.update();
        }
        /*
        if(dist > 0){
            while(Math.abs(right_BackM.getCurrentPosition()) < Math.abs(right_target) &&
                    !opmode.isStopRequested()){
                right_FrontM.setPower(power);
                right_BackM.setPower(power);
                left_FrontM.setPower(power);
                left_BackM.setPower(power);

                opmode.telemetry.addData("tics", tics);
                opmode.telemetry.addData("Current Position", Math.abs(right_BackM.getCurrentPosition()));
                opmode.telemetry.addData("Distance to go", Math.abs(Math.abs(right_BackM.getCurrentPosition()) - Math.abs(right_target)));
                opmode.telemetry.update();
            }
        }
        else{
            while(Math.abs(right_BackM.getCurrentPosition()) >  Math.abs(right_target) &&
                    !opmode.isStopRequested()){
                right_FrontM.setPower(-power);
                right_BackM.setPower(-power);
                left_FrontM.setPower(-power);
                left_BackM.setPower(-power);

                opmode.telemetry.addData("tics", tics);
                opmode.telemetry.addData("Current Position", Math.abs(right_BackM.getCurrentPosition()));
                opmode.telemetry.addData("Distance to go", Math.abs(Math.abs(right_BackM.getCurrentPosition()) - Math.abs(right_target)));
                opmode.telemetry.update();
            }

        }*/
        go(0.0, 0.0);
    }
    public void moveMotor(DcMotor motor, double dist, double power){
        int tics = (int)((dist/(wheel_circumfrence*wheel_to_motor_ratio))*TICKS_PER_WHEEL_REV);

        int motor_target = Math.abs(motor.getCurrentPosition()) + tics;

        if(dist > 0){
            motor_power = power;
        }
        else{
            motor_power = -power;
        }
        while((Math.abs(Math.abs(right_BackM.getCurrentPosition())-Math.abs(motor_target)) > 5) &&
                !opmode.isStopRequested()){
            motor.setPower(motor_power);

            //must add acceleration nd deceleration
            //double rp = right_power *(Math.abs(Math.abs(right_BackM.getCurrentPosition())-Math.abs(right_target))/tics);
            //double lp = right_power *(Math.abs(Math.abs(left_BackM.getCurrentPosition())-Math.abs(left_target))/tics);
            opmode.telemetry.addData("tics", tics);
            opmode.telemetry.addData("Current Position", Math.abs(right_BackM.getCurrentPosition()));
            opmode.telemetry.addData("Distance to go", Math.abs(Math.abs(right_BackM.getCurrentPosition()) - Math.abs(motor_target)));
            opmode.telemetry.update();
        }
        motor.setPower(0.0);
    }
    public void move_using_tics(int tics, double power){

        int right_target = Math.abs(right_BackM.getCurrentPosition()) + tics;
        int left_target = Math.abs(left_BackM.getCurrentPosition()) + tics;

        while(Math.abs(right_BackM.getCurrentPosition()) < Math.abs(right_target) &&
                Math.abs(left_BackM.getCurrentPosition()) < Math.abs(left_target) && !opmode.isStopRequested()){
            right_FrontM.setPower(power);
            right_BackM.setPower(power);
            left_FrontM.setPower(power);
            left_BackM.setPower(power);

            opmode.telemetry.addData("right target", right_target);
            opmode.telemetry.addData("Current Position", Math.abs(right_BackM.getCurrentPosition()));
            opmode.telemetry.addData("Distance to go", Math.abs(Math.abs(right_BackM.getCurrentPosition()) - Math.abs(right_target)));
            opmode.telemetry.update();
        }
        go(0.0, 0.0);
    }


    public void turn_relative_robot(double turn_angle, double power){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current_angle = angles.firstAngle;

        //adds the turning degrees to the robot angle so that the turn starts form where the robot is facing
        double target_angle = current_angle + turn_angle;
        double error;


        double pow;
        double c;


        while(Math.abs(target_angle) - Math.abs(current_angle) > 10 && !opmode.isStopRequested()){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            current_angle = angles.firstAngle;


            error = target_angle - current_angle;
            //c = 1+(1/error);
            /*pow = power*Math.pow(Math.E,
                    (
                            (-1/(2*turn_angle)) * Math.pow(turn_angle-current_angle , 2.0)
                    )
            );
*/
            pow = power;

            if(target_angle>0){
                right_FrontM.setPower(pow);
                right_BackM.setPower(pow);
                left_FrontM.setPower(-pow);
                left_BackM.setPower(-pow);

            }
            else{
                right_FrontM.setPower(-pow);
                right_BackM.setPower(-pow);
                left_FrontM.setPower(pow);
                left_BackM.setPower(pow);

            }
            opmode.telemetry.addData("Cangle", current_angle);
            opmode.telemetry.addData("Tangle", turn_angle);
            opmode.telemetry.addData("Gangle", target_angle);
            opmode.telemetry.addData("error", error);
            opmode.telemetry.addData("pow", pow);
            opmode.telemetry.update();
        }
        go(0.0 , 0.0);
    }

    public void turn_relative_field(double target_angle, double power){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double current_angle = angles.firstAngle % 360;
        boolean clockwise = true;
        //if clockwise else counter clockwise
        if(current_angle > target_angle){
            right_power = power;
            left_power = -power;
            clockwise = true;
        }
        else{
            right_power = -power;
            left_power = power;
            clockwise = false;
        }

        while(Math.abs(Math.abs(current_angle) - Math.abs(target_angle)) > 5 && !opmode.isStopRequested()){
            right_FrontM.setPower(right_power);
            right_BackM.setPower(right_power);
            left_FrontM.setPower(left_power);
            left_BackM.setPower(left_power);
        }
        go(0.0,0.0);
    }

    public void go(double rpower, double lpower){
        right_FrontM.setPower(rpower);
        right_BackM.setPower(rpower);
        left_FrontM.setPower(lpower);
        left_BackM.setPower(lpower);
    }

    public void setUpIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // initializes the IMU
        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


    }
}
