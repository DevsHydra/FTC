package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Date;
import java.util.Locale;

@Autonomous
public class VermelhoDoisNivelPatoArm2 extends LinearOpMode {
    private DcMotor LMF;
    private DcMotor LMB;
    private DcMotor RMF;
    private DcMotor RMB;
    private DcMotor Esteira;
    private DcMotor MB;      //Motor - Braço - Ticks 1120
    private DcMotor MC;      //Motor - Coletar
    private DcMotor ML;

    private Servo SL;
    private Servo SC;       //Sero - coleta
    private CRServo SPR;
    private CRServo SPL;

    HardwarePushbot robot = new HardwarePushbot();

    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;

    DigitalChannel digitalTouch;  // Hardware Device Object

    DistanceSensor sensorDistance;


    double valorCarregado[][] = new double[500][4];
    double timer = 0;
    double timer2 = 0;
    double timer3 = 0;
    double difTimer = 0;
    boolean objeto = false, temosObjeto = false;
    boolean controle = false;
    int i = 0;
    String nivel = "";
    double pointBraco = 0;
    double proporcional = 0, integral = 0, derivada = 0, ultErro = 0, kP = 0, erro = 0;
    int posicion = 0;
    double MIN_PALETA = 0.2;
    double MAX_PALETA = 0.45;


    BNO055IMU imu;

    Orientation angles;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        boolean qqw = true;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch");
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        LMF = hardwareMap.get(DcMotor.class, "LMF");
        LMB = hardwareMap.get(DcMotor.class, "LMB");
        RMF = hardwareMap.get(DcMotor.class, "RMF");
        RMB = hardwareMap.get(DcMotor.class, "RMB");
        Esteira = hardwareMap.get(DcMotor.class, "MMT");
        MC = hardwareMap.get(DcMotor.class, "MC");
        MB = hardwareMap.get(DcMotor.class, "MB");
        ML = hardwareMap.get(DcMotor.class, "ML");

        SC = hardwareMap.get(Servo.class, "SC");
        SL = hardwareMap.get(Servo.class, "SL");

        SPR = hardwareMap.crservo.get("SPR");
        SPL = hardwareMap.crservo.get("SPL");

        LMF.setDirection(DcMotorSimple.Direction.REVERSE);
        LMB.setDirection(DcMotorSimple.Direction.FORWARD);
        RMF.setDirection(DcMotorSimple.Direction.FORWARD);
        RMB.setDirection(DcMotorSimple.Direction.FORWARD);
        Esteira.setDirection(DcMotorSimple.Direction.REVERSE);
        MB.setDirection(DcMotorSimple.Direction.FORWARD);
        MC.setDirection(DcMotorSimple.Direction.FORWARD);
        ML.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setDirection(Servo.Direction.FORWARD);
        SC.setDirection(Servo.Direction.REVERSE);

        ML.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ML.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        resetStartTime();
        objeto = false;
        temosObjeto = false;
        nivel = "Coletar";

        timer = getRuntime();
        setNivel(nivel);

        telemetry.addData("Distancia -\t", String.format(Locale.US, "%02f", sensorDistance.getDistance(DistanceUnit.CM)));
        telemetry.addData("Objeto -\t", objeto);
        telemetry.addData("Nivel -\t", nivel);
        telemetry.addData("Timer -\t", timer);
        telemetry.addData("Timer2 -\t", timer2);
        telemetry.addData("Timer3 -\t", timer3);
        telemetry.addData("DifTimer -\t", difTimer);
        telemetry.addData("setNivel -\t", pointBraco);

        telemetry.update();

        //  Vai para a identificação


        SL.setPosition(MAX_PALETA);
        temosObjeto = true;


        turnTo(0);
        rlPower(0.75, -0.7);
        MC.setPower(-1);
        sleep(650);
        turnTo(0);

        MC.setPower(0);

        //  Identifica
        while (!objeto) {
            if (controle) {
                timer3 = timer;
                controle = false;
            }

            allPower(0.3);
            timer2 = getRuntime() - timer3;

            telemetry.addData("Distancia -\t", String.format(Locale.US, "%02f", sensorDistance.getDistance(DistanceUnit.CM)));
telemetry.update();

            if (sensorDistance.getDistance(DistanceUnit.CM) <= 30 || timer2 >= 2.4) {
                nivelObjet(timer2);
                objeto = true;
            }
        }
        allPower(0);


//          Alinhar com ShipHub
        if(nivel == "2"){
            turnTo(230);
        }else if(nivel == "1"){
            turnTo(250);
        }else if(nivel == "3"){
            turnTo(270);
        }

//  Soltar carga
        if(nivel == "2"){
            allPower(0.2);
            sleep(700);
            allPower(0);

        }else if(nivel == "1"){
            allPower(0.2);
            sleep(900);
            allPower(0);

        }else if(nivel == "3"){
            allPower(0.2);
            sleep(1700);
            allPower(0);
        }

        setNivel(nivel);



        while (objeto){
            telemetry.addData("Nivel -", nivel);
            telemetry.addData("Timer2 - ",timer2);
            telemetry.update();
        }




//            if (objeto) {
//
//                if (nivel == "1") {
//
//                    allPower(1);
//                    sleep(200);
//
//                } else if (nivel == "2") {
//
//                    allPower(1);
//                    sleep(400);
//
//                } else if (nivel == "3") {
//
//                    allPower(1);
//                    sleep(150);
//
//                }
//
//
//                //  Se alinha ao ShipHub
//
//                turnTo(270);
//
        //            if(nivel == "1") {
        //                allPower(0.4);
        //                sleep(200);
        //            }else if(nivel == "2"){
        //                allPower(0.4);
        //                sleep(600);
        //            }else if(nivel == "3"){
        //                allPower(0.4);
        //                sleep(50);
        //            }


        //  Leva o braço até os níveis

        //setNivel(nivel);


        //            nivel = "Coletar";
        //            setNivel(nivel);
        //
        //            rlPower(1, -1);
        //            sleep(1250);
        //
        //            turnTo(90);
        //
        //            allPower(0.2);
        //            sleep(50);
        //
        //
        //            //robot.SPR.setPower(1);
        //            ML.setPower(1);
        //            sleep(7000);
        //            i++;
        //

    }



    public void allPower(double forca){
        RMF.setPower(forca);
        LMF.setPower(forca);
        RMB.setPower(forca);
        LMB.setPower(forca);
        Esteira.setPower(forca);
    }

    public void rlPower(double forceD, double forceE){
        RMF.setPower(forceD);
        LMF.setPower(forceE);
        RMB.setPower(forceE);
        LMB.setPower(forceD);
    }

    public void setAllPower(double p){     //Metodo para definir a mesma força para todos os motores
        setMotorPower(p,p,p,p,p);
    }

    public void setMotorPower(double lF, double rF, double lB, double rB, double eS){
        LMF.setPower(lF);
        LMB.setPower(lB);
        RMB.setPower(rB);
        RMF.setPower(rF);
        Esteira.setPower(eS);
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
        pointBraco = nivel == "Coletar" ? 0 : nivel == "3" ? 860 : nivel == "2" ? 795 : nivel == "1" ? 650 : pointBraco;

        runNivel(pointBraco, nivel);
    }

    public void runNivel(double pointBraco, String nivel){

        int i=0;
        double forca =-0.4;
        while (temosObjeto){

            telemetry.addData("Erro - ", erro);
            telemetry.addData("i - ", i);
            telemetry.update();

            posicion = nivel == "Coletar" ? 0 : nivel != "Coletar" && MB.getCurrentPosition() > 200 ? 70: posicion;

            erro = (pointBraco - MB.getCurrentPosition());

            kP = nivel == "Coletar" ? 0.0004 : 0.0008;

            proporcional = (erro * kP);

            proporcional = proporcional > 0.4 ? 0.4 : proporcional < -0.6 ? -0.6 : proporcional;

            derivada = (erro - ultErro) * 0.008;

            ultErro = erro;

            MB.setPower(proporcional + derivada );

            ML.setTargetPosition(posicion);
            ML.setPower(1);
            ML.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(erro >= -6 && erro <= 8 && i > 500){

                allPower(forca);
                sleep(300);

                allPower(0);

                SL.setPosition(MIN_PALETA);
                sleep(3000);

                temosObjeto = false;

            }
            i++;

        }


    }


    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
            setMotorPower(-motorPower, motorPower, -motorPower, motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);


            telemetry.addData("Nivel -\t", nivel);

            telemetry.update();
        }

        setAllPower(0);
    }

    public void turnTo(double degrees){

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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
        return imu.getAngularOrientation(
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
