package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
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

@Config
public class RobotNew extends LinearOpMode {
    public static int elevatoeHighPos = 1650;
    public static int elevatorMiddlePos = 850;
    public static int elevatorLowPos = 160;
    public static int elevatorGroundPos = 0;

    int[] cones = {250, 140, 30, 100, 0};

    // cone 1 = 700  cone 2 = 600 cone 3 = 400 cone 4 = 200 cone 5 = 0

    public static double clawClose = 0.2;
    public static double clawOpen = 0;

    public static double tiltHigh = 0.1;
    public static double tiltMid = 0.1;
    public static double tiltLow = 0.0;
    public static double tiltGround = 0.0;

    public static double armHigh = 0.8;
    public static double armMid = 0.8;
    public static double armLow = 0.37;
    public static double armGround = 0.0;



    public static double tiltAuto = 0;

    boolean isGround = false;

    public BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    Orientation angle = new Orientation();
    double globalAngle = 0, power = .30, correction;


    public SampleMecanumDrive drive;
    public DcMotor elevator0,elevator1;
    public Servo claw, tilt, arm0, arm1;
    public ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415926);


    boolean close = false;
    boolean wasPressed = true;
    boolean timerBrake = false;
    boolean timerBrake1 = false;

    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        elevator0 = hardwareMap.dcMotor.get("elevator0");
        elevator1 = hardwareMap.dcMotor.get("elevator1");

        claw = hardwareMap.servo.get("claw");
        tilt = hardwareMap.servo.get("tilt");

        arm0 = hardwareMap.servo.get("arm0");
        arm1 = hardwareMap.servo.get("arm1");


        elevator0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator0.setDirection(DcMotorSimple.Direction.REVERSE);

        tilt.setDirection(Servo.Direction.FORWARD);

        arm1.setDirection(Servo.Direction.REVERSE);


        elevator0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
       /* BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);*/


        claw.setPosition(clawClose);
        armPos(armGround);
        tilt. setPosition(tiltGround);


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




    public void elevatorPower(double power) {
        elevator0.setPower(power);
        elevator1.setPower(power);
    }
    public void elevatorTargetPosition(int TargetPosition){
        elevator0.setTargetPosition(TargetPosition);
        elevator1.setTargetPosition(TargetPosition);
    }
    public void elevatorSetMode(){
        elevator0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void armPos(double setPos) {
        arm0.setPosition(setPos);
        arm1.setPosition(setPos);
    }

    public void elevatorAuto(int Position )
    {
        elevatorTargetPosition(Position);
        armPos(0.1);
        elevatorPower(0.7);
        elevatorSetMode();
        tilt.setPosition(tiltGround);
    }
    public void elevatorAfterColloctAuto() {
        elevatorTargetPosition(1150);
        elevatorPower(0.7);
        elevatorSetMode();
    }


    public void elevatorHigh() {
     //   elapsedTime.reset();
        elevatorTargetPosition(elevatoeHighPos);
        armPos(armHigh);
        elevatorPower(0.7);
        elevatorSetMode();
        tilt.setPosition(tiltHigh);
        /*if (elapsedTime.seconds() >= 0.5) {
            tilt.setPosition(tiltHigh);
        }*/
    }

    public void elevatorMid() {
       // elapsedTime.reset();
        elevatorTargetPosition(elevatorMiddlePos);
        armPos(armMid);
        elevatorPower(0.7);
        elevatorSetMode();
        tilt.setPosition(tiltMid);
    }

    public void elevatorLow() {
        elevatorTargetPosition(elevatorLowPos);
        armPos(armLow);
        elevatorPower(0.7);
        elevatorSetMode();
        tilt.setPosition(tiltLow);
    }


    public void elevatorGround() {
        armPos(armGround);
        elevatorTargetPosition(elevatorGroundPos);
        elevatorPower(0.7);
        elevatorSetMode();
        tilt.setPosition(tiltGround);
    }

    public void tiltControl() {
        tilt.setPosition((arm0.getPosition() * 5) + 0.15);
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