package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
@TeleOp
public class MainTeleOp extends LinearOpMode {
    org.firstinspires.ftc.teamcode.Robot robot = new org.firstinspires.ftc.teamcode.Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if(opModeIsActive()){
            robot.runTeleOp();
        }
    }
}
