package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="opal", group="Robot")
public class opal extends Robot {

    @Override
    public void runOpMode() {
        super.runOpMode();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            float x = gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y;
            float rx = gamepad1.right_stick_x;
            float pad2lsy = gamepad2.left_stick_y;
            float pad2lt = gamepad2.right_trigger;

            float pad2rt = gamepad2.left_trigger;
            boolean HighDpadU = gamepad2.dpad_up;
            boolean midDpadL = gamepad2.dpad_left;
            boolean lowDpaD = gamepad2.dpad_down;

            float lfPower = y + x + rx;
            float lbPower = y - x + rx;
            float rfPower = y - x - rx;
            float rbPower = y + x - rx;

            lf.setPower(lfPower);
            lb.setPower(lbPower);
            rf.setPower(rfPower);
            rb.setPower(rbPower);
            arm.setPosition(pad2lsy);
            telemetry.addLine("LSX: " + x);
            telemetry.addLine("LSY: " + y);
            telemetry.addLine("RSX: " + rx);
            telemetry.addLine(String.valueOf(lfPower));
            telemetry.addLine(String.valueOf(lbPower));
            telemetry.addLine(String.valueOf(rfPower));
            telemetry.addLine(String.valueOf(rbPower));
            telemetry.update();

//            if (pad2lt) {
//                s2.setPosition(0.05);
//            }
//            else  {
//                s2.setPosition(0);
//            }
//            if (gamepad2) {
//                s1.setPosition(0.3);
//            }
//            else {
//                s1.setPosition(0);
//           }

              if (lowDpaD) {
                  elevator.setPower(0.7);
                  elevator.setTargetPosition(2000);
                  elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


              }
              if (midDpadL) {
                  elevator.setPower(0.7);
                  elevator.setTargetPosition(5000);
                  elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);


              }

              if (HighDpadU) {
                  elevator.setPower(0.7);
                  elevator.setTargetPosition(8000);
                  elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

              }

                  if (!elevator.isBusy()) {
                      elevator.setPower(0);
              }
        }
    }
}
