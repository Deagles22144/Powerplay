package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="ResetOpMode", group="Robot")
public class ResetOpMode extends RobotNew {

    @Override
    public void runOpMode() {
        super.runOpMode();

        waitForStart();
        elapsedTime.reset();
        while (opModeIsActive() && !isStopRequested()) {

            armPos(armReset);
            claw.setPosition(clawOpen);


        }
    }
}
