package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Locale;

@Autonomous
public class Autonomo2 extends LinearOpMode {

    HardwarePushbot         robot   = new HardwarePushbot();


    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

//    private DcMotor LMF;
//    private DcMotor LMB;
//    private DcMotor RMF;
//    private DcMotor RMB;
    private DcMotor Esteira;
    double valorCarregado[][] = new double[500][4];
    double timer = 0;
    double timer2 = 0;
    double timer3 = 0;
    double difTimer = 0;
    double jGravacao = 0;
    boolean objeto = false;
    boolean controle = false;
    int i =0;
    String nivel = "";
    double pointBraco =0;
    double powerBraco = 0.6;

    double test =0;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
//
//        RMF = hardwareMap.get(DcMotor.class, "RMF");
//        LMF = hardwareMap.get(DcMotor.class, "LMF");
//        RMB = hardwareMap.get(DcMotor.class, "RMB");
//        LMB = hardwareMap.get(DcMotor.class, "LMB");
        Esteira = hardwareMap.get(DcMotor.class, "MMT");
//        SC = hardwareMap.get(Servo.class, "SC");

        sensorColor = hardwareMap.get(ColorSensor.class,"sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class,"sensor_color_distance");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 225;
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//
//        // Set Direction
//        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
//        LMB.setDirection(DcMotorSimple.Direction.REVERSE);
//        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
//        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
//        SC.setDirection(Servo.Direction.REVERSE);
//
//        RMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LMF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LMB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        resetStartTime();
        objeto=false;
        nivel = "Coletar";
        while(opModeIsActive()) {

            timer = getRuntime();
            setNivel(nivel);

            telemetry.addData("Cor - ", sensorColor.blue());
            telemetry.addData("Distancia -\t", String.format(Locale.US, "%02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Objeto -\t", objeto);
            telemetry.addData("Nivel -\t", nivel);
            telemetry.addData("Timer -\t", timer);
            telemetry.addData("Timer2 -\t", timer2);
            telemetry.addData("Timer3 -\t", timer3);
            telemetry.addData("DifTimer -\t", difTimer);
            telemetry.addData("setNivel -\t", pointBraco);

            telemetry.update();

            if(!objeto) {
                rlPower(-1, 1);
                sleep(480);
                turnTo(0);
            }



            while (!objeto) {
                if(controle){
                    timer3 = timer;
                    controle = false;
                }

                allPower(0.2);
                timer2 = getRuntime() - timer3;
//                difTimer = timer2 - timer3;

                if (sensorDistance.getDistance(DistanceUnit.CM) <= 30 || timer2 >=2.4) {
                    objeto = true;
                }

            }

//          Movimentação ate o ShipHub


            if(objeto){
                robot.SC.setPosition(1);
                nivelObjet(timer2);
                setNivel(nivel);

                if(i == 0) {
                    if (nivel == "1") {

                        allPower(0.5);
                        sleep(370);

                    } else if (nivel == "2") {

                        allPower(0.5);
                        sleep(620);

                    } else if (nivel == "3") {

                        allPower(0.5);
                        sleep(150);

                    }
                }

                turnTo(90);

                if(i == 0) {
                    telemetry.addLine("Só1");
                    telemetry.update();
                    allPower(0.4);
                    sleep(200);

                    //Falta add Paleta


                }

                if(i == 0){
                    nivel = "Coletar";
                    setNivel(nivel);

                    rlPower(-1, 1);
                    sleep(1250);

                    turnTo(90);

                    allPower(0.2);
                    sleep(50);


                    robot.SPR.setPower(1);
                    robot.MP.setPower(1);
                    sleep(2000);
                    i++;

                }

            }


            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor((Color.HSVToColor(0xff, values)));
                }
            });

            telemetry.update();

        }
        relativeLayout.post(new Runnable() {
            @Override
            public void run() {
                relativeLayout.setBackgroundColor((Color.WHITE));
            }
        });

//        allPower(0.5);
//        sleep(1000);
//        powerDouble
//        sleep(800);
//        allPower(0.1);
//        sleep(100);
//
//        sleep(1000);

    }

    public String nivelObjet(double timer2){

        if(timer2 <= 1.4){
            return nivel = "2";
        }
        else if(timer2>1.4 && timer2<=2.38){
            return nivel = "1";
        }
        return nivel = "3";

    }

    public void setNivel(String nivel){
        powerBraco = 0.6;
        pointBraco = nivel == "Coletar" ? 10 : nivel == "1" ? -950 : nivel == "2" ? -800 : nivel == "3" ? -650 : pointBraco;

        runNivel(pointBraco, powerBraco);
    }

    public void runNivel(double pointBraco, double powerBraco){

        if(robot.MB.getCurrentPosition() != pointBraco) {

//            if (nivel == "Coleta") {
//                powerBraco = 0.2;
//                robot.MB.setTargetPosition(-80);
//                robot.MB.setPower(powerBraco);
//                robot.MB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                sleep(500);
//            }

            robot.MB.setTargetPosition((int) pointBraco);
            robot.MB.setPower(powerBraco);
            robot.MB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void allPower(double force){
        robot.RMF.setPower(force);
        robot.LMF.setPower(force);
        robot.RMB.setPower(force);
        robot.LMB.setPower(force);
    }
    public void rlPower(double forceD, double forceE){
        robot.RMF.setPower(forceD);
        robot.LMF.setPower(forceE);
        robot.RMB.setPower(forceE);
        robot.LMB.setPower(forceD);
    }

    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);


            telemetry.addData("Nivel -\t", nivel);

            telemetry.update();
        }

        robot.setAllPower(0);
    }

    public void turnTo(double degrees){

        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        turn(error);
    }

    public double getAbsoluteAngle() {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPower(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
    }

}
