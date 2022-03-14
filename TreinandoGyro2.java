/*
package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous
public class TreinandoGyro2 extends LinearOpMode {

    HardwarePushbot robot = new HardwarePushbot();
    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        turn(90);

        sleep(3000);

        turnTo(-90);




    }

    public void resetAngle(){
        //lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle(){
        Orientation orientation= robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        if(deltaAngle > 180){
            deltaAngle -= 360;
        } else if(deltaAngle <= -180){
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("Gyro:", orientation.firstAngle);
        return  currAngle;
    }

    public void turn(double degress){

        resetAngle();

        double error = degress;

        while (opModeIsActive() && Math.abs(error) > 2){
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            error = degress - getAngle();
            telemetry.addData("Error:", error);
            telemetry.update();
        }

        robot.setAllPower(0);

    }

    public void turnTo(double degress){
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double error = degress - orientation.firstAngle;

        if(error > 180){
            error -= 360;
        } else if(error < -180){
            error += 360;
        }

        turn(error);

    }



}
*/
