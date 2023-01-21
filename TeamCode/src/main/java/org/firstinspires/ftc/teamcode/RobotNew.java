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
    public static int elevatoeHighPos = 1500;
    public static int elevatorMiddlePos = 975;
    public static int elevatorLowPos = 0;
    public static int elevatorGroundPos = 0;

    public static double powerDownElevator = 0.8;

    int[] cones = {425, 275, 175, 95, 0};

    // cone 1 = 544  cone 2 = 460 cone 3 = 340 cone 4 = 200 cone 5 = 0

    public static double clawClose = 0.15;
    public static double clawOpen = 0.05;




    /*public static double tiltHigh = 0.8;
    //public static double tiltMid = 0.75;
    //public static double tiltLow = 0.1;
    public static double tiltGround = 0.1; */


    public static double armHighAuto = 0.7;
    public static double armHigh = 0.57;
    public static double armMid = 0.62;
    public static double armLow = 0.6;
    public static double armReset = 0.0;
    public static double armGround = armReset + 0.03;
    public static double armPreRelease = armMid + 0.09;
    public static double armLoadCone = 0.45;


    public static double tiltAuto = 0;

    boolean isGround = true;

    public BNO055IMU imu;

    Orientation lastAngles = new Orientation();
    Orientation angle = new Orientation();
    double globalAngle = 0, power = .30, correction;


    public SampleMecanumDrive drive;
    public DcMotor elevator0,elevator1;
    public Servo claw , arm0, arm1/*,tilt0,tilt1*/;
    public ElapsedTime runtime = new ElapsedTime();


    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 9.6;     // For figuring circumference
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415926);


    boolean close = false;
    boolean wasPressed = true;
    boolean timerBrake = false;
    boolean timerBrake1 = false;
    boolean timerBrake2 = true;
    boolean timerBrakeMid = false;
    boolean isLow = false,isMid = false, isHigh = false;
    ElapsedTime elapsedTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        drive = new SampleMecanumDrive(hardwareMap);

        elevator0 = hardwareMap.dcMotor.get("elevator0");
        elevator1 = hardwareMap.dcMotor.get("elevator1");

        claw = hardwareMap.servo.get("claw");

        //tilt0 = hardwareMap.servo.get("tilt0");
        //tilt1 = hardwareMap.servo.get("tilt1");

        arm0 = hardwareMap.servo.get("arm0");
        arm1 = hardwareMap.servo.get("arm1");


        elevator0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elevator0.setDirection(DcMotorSimple.Direction.REVERSE);
//        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);

        //tilt1.setDirection(Servo.Direction.REVERSE);
        //claw.setDirection(Servo.Direction.REVERSE);


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
        //tiltPos(tiltGround);


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
        elevator0.setTargetPosition(TargetPosition+40);
        elevator1.setTargetPosition(TargetPosition+40);
    }
    public void elevatorSetMode(){
        elevator0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void armPos(double setPos) {
        arm0.setPosition(setPos);
        arm1.setPosition(setPos);
    }

    public void tiltPos(double setPos) {
        //tilt0.setPosition(setPos);
        //tilt1.setPosition(setPos);

    }

    public void elevatorAuto(int Position )
    {
        elevatorTargetPosition(Position);
        armPos(armGround);
        elevatorPower(0.7);
        elevatorSetMode();
       // tiltPos(tiltGround);
    }
    public void elevatorAfterColloctAuto() {
        elevatorTargetPosition(1150);
        elevatorPower(0.7);
        elevatorSetMode();
    }

    public void elevatorHighAuto() {
        elevatorTargetPosition(elevatoeHighPos);
       // armPos(armHigh);
        elevatorPower(0.7);
        elevatorSetMode();
       // tiltPos(tiltHigh);

    }


    public void elevatorHigh() {
        elevatorTargetPosition(elevatoeHighPos);
//        armPos(armHigh);
        elevatorPower(1);
        elevatorSetMode();
      //  tiltPos(tiltHigh);

    }

    public void elevatorMidAuto() {
        elevatorTargetPosition(elevatorMiddlePos);
        // armPos(armHigh);
        elevatorPower(1);
        elevatorSetMode();
        //tiltPos(tiltMid);

    }


    public void elevatorMid() {
        elevatorTargetPosition(elevatorMiddlePos);
       // armPos(armMid);
        elevatorPower(1);
        elevatorSetMode();
    //    tiltPos(tiltMid);
    }

    public void elevatorLow() {
        elevatorTargetPosition(elevatorLowPos);
        armPos(armLow);
        elevatorPower(1);
        elevatorSetMode();
       // tiltPos(tiltLow);
    }


    public void elevatorGround() {
       /* armPos(armGround);
        elevatorTargetPosition(elevator0.getCurrentPosition() + 100);
        elevatorPower(0.1);
        elevatorSetMode();*/
        armPos(armGround);
        elevatorTargetPosition(elevatorGroundPos);
        elevatorPower(powerDownElevator);
        elevatorSetMode();
       // tiltPos(tiltGround);
        claw.setPosition(clawClose);
        close = false;
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