package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Nothing", group="Robot")
public class Nothing extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        while (!isStarted() && !isStopRequested()) {
            QrScan();
        }


        waitForStart();

        armPos(0.02);
        }
    }