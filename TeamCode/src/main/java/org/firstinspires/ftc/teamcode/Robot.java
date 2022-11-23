package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class Robot extends LinearOpMode {
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor rf;
    public DcMotor lf;
    public Servo claw;
    public Servo tilt;
    public Servo arm;
    public DcMotor elevator;
    @Override
    public void runOpMode() {
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");
        rf = hardwareMap.dcMotor.get("rf");
        lf = hardwareMap.dcMotor.get("lf");


        claw = (Servo) hardwareMap.servo.get("claw");
        tilt = (Servo) hardwareMap.servo.get("tilt");
        arm = (Servo)hardwareMap.servo.get("arm");

        claw.setPosition(0.0);

        lb.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator = hardwareMap.dcMotor.get("elevator");

        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}