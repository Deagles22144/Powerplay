package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public abstract class RobotNew extends LinearOpMode {
    int high = 1650;
    int mid = 1000;
    int low = 160;
    int ground = 0;
    double clawClose = 0.05;
    double clawOpen = 0.03;


    public BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    Orientation angle = new Orientation();
    double globalAngle = 0, power = .30, correction;


    public SampleMecanumDrive drive;
    public DcMotor lf, rf, lb, rb, elevator,elevatorr;
    public Servo claw, tilt, arm;
    public ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415926);


    boolean close = false;
    boolean wasPressed = true;
    boolean timerBrake = false;
    ElapsedTime elapsedTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        elevator = hardwareMap.dcMotor.get("elevator");
        elevatorr = hardwareMap.dcMotor.get("elevator");

        claw = hardwareMap.servo.get("claw");
        tilt = hardwareMap.servo.get("tilt");

        arm = hardwareMap.servo.get("arm");


        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorr.setDirection(DcMotorSimple.Direction.REVERSE);

        tilt.setDirection(Servo.Direction.FORWARD);


        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm.setDirection(Servo.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
       /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);*/


        claw.setPosition(clawClose);
        arm.setPosition(0.00);
        tilt.setPosition(0.0);


    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    public double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = (angles.firstAngle - lastAngles.firstAngle);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }


    public void RotateP(int degrees, double power, double timeoutR, double KP) {

        runtime.reset();

        if (getAngle() < degrees) {
            while ((getAngle() < degrees) &&  (runtime.seconds() < timeoutR)) {
                double error = degrees - getAngle();

                lf.setPower(-power * error * KP);
                lb.setPower(-power * error * KP);
                rf.setPower(power * error * KP);
                rb.setPower(power * error * KP);

                telemetry.addData("heading", getAngle());
                telemetry.update();

            }
        } else if (getAngle() > degrees) {
            while ((getAngle() > degrees) && (runtime.seconds() < timeoutR)) {
                double error = getAngle() - degrees;

                lf.setPower(power * error * KP);
                lb.setPower(power * error * KP);
                rf.setPower(-power * error * KP);
                rb.setPower(-power * error * KP);

            }
        } else return;
        // turn the motors off.
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);
    }


    public void encoderDriveP(double speed, double LeftFrontCM, double LeftBackCM, double RightFrontCM, double RightBackCM, double KP, double TimeOut) {

        runtime.reset();

        double error = LeftFrontCM - lf.getCurrentPosition();
        int newLeftFrontTarget = 0;
        int newLeftBackTarget = 0;
        int newRightFrontTarget = 0;
        int newRightBackTarget = 0;
        double Pnumber = 0.019;
        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        newLeftFrontTarget = lf.getCurrentPosition() + (int) (LeftFrontCM * COUNTS_PER_CM);
        newLeftBackTarget = lb.getCurrentPosition() + (int) (LeftBackCM * COUNTS_PER_CM);
        newRightFrontTarget = rf.getCurrentPosition() + (int) (RightFrontCM * COUNTS_PER_CM);
        newRightBackTarget = rb.getCurrentPosition() + (int) (RightBackCM * COUNTS_PER_CM);

        lf.setTargetPosition(newLeftFrontTarget);
        lb.setTargetPosition(newLeftBackTarget);
        rf.setTargetPosition(newRightFrontTarget);
        rb.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.

        while ((newLeftFrontTarget >= lf.getCurrentPosition() && runtime.seconds() < TimeOut) && (newLeftBackTarget >= lb.getCurrentPosition() && runtime.seconds() < TimeOut)
                && (newRightFrontTarget >= rf.getCurrentPosition() && runtime.seconds() < TimeOut) && (newRightBackTarget >= rb.getCurrentPosition() && runtime.seconds() < TimeOut)
                || (newLeftFrontTarget <= lf.getCurrentPosition() && runtime.seconds() < TimeOut) && (newLeftBackTarget <= lb.getCurrentPosition() && runtime.seconds() < TimeOut)
                && (newRightFrontTarget <= rf.getCurrentPosition() && runtime.seconds() < TimeOut) && (newRightBackTarget <= rb.getCurrentPosition() && runtime.seconds() < TimeOut)) {

            error = newLeftFrontTarget - lf.getCurrentPosition();


            lf.setPower(Math.abs(speed * error * KP));
            lb.setPower(Math.abs(speed * error * KP));
            rf.setPower(Math.abs(speed * error * KP));
            rb.setPower(Math.abs(speed * error * KP));

        }

        // Stop all motion;
        lf.setPower(0);
        lb.setPower(0);
        rf.setPower(0);
        rb.setPower(0);

        // Turn off RUN_TO_POSITION
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void elevatorHigh() {
        elevator.setTargetPosition(high);
        arm.setPosition(0.1);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.7);
    }

    public void elevatorMid() {
        elevator.setTargetPosition(mid);
        arm.setPosition(0.09);
        elevator.setPower(0.7);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void elevatorLow() {
        elevator.setTargetPosition(low);
        arm.setPosition(0.09);
        elevator.setPower(0.7);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void elevatorGround() {
        elevator.setTargetPosition(ground);
        arm.setPosition(0.0);
        elevator.setPower(0.7);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void tiltControl() {
        tilt.setPosition(arm.getPosition() - 0.04);
    }


    public void ToggleClaw() {
        if (wasPressed) {
            close = !close;
            wasPressed = false;
            timerBrake = true;
            elapsedTime.reset();
        }

        if (close) {
            claw.setPosition(clawClose);
        } else {
            claw.setPosition(clawOpen);
        }
    }
}