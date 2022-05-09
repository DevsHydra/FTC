package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import java.io.File;
import java.util.Arrays;
import java.util.Date;

@TeleOp
public class Gravacao extends LinearOpMode {

    double speed =1;
    double angulofinalE;
    double angulofinalD;
    double angulorealE = 0;
    double angulorealD = 0;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;
    long timer = System.currentTimeMillis();
    int iGravacao = 0;
    boolean gravar = false;
    boolean gravar2 = false;
    double valorArmazenado[][] = new double[500][4];
    double valorCarregado[][] = new double[500][4];
    double timer5 = 0;
    double[] forcaMotor = {0, 0}; // 0 ESQUERDA, E 1 DIREITA EM RELAÇÃO A MECANUM.
    double[] forcax = {0, 0}; // 0 ESQUERDA E 1 DIREITA
    long timer2 = System.currentTimeMillis();
    boolean executar = false;
    ElapsedTime timer3 = new ElapsedTime();
    ElapsedTime timer4 = new ElapsedTime();
    int jGravacao = 0;
    File arquivo = new File("/C:/Users/Ramon/Documents");

//    PrintWriter gravarArq = new PrintWriter(arquivoGravacao);


    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        waitForStart();
        timer = System.currentTimeMillis();
        timer3.time();
        while(opModeIsActive()){
//            telemetry.addData("tempo: ", System.currentTimeMillis());
//            telemetry.addData("tempo - timer", System.currentTimeMillis() - timer);
//            telemetry.addData("Força y", gamepad1.left_stick_y);
//            if(System.currentTimeMillis() - timer < 20) {
//                continue;
//            }
            timer = System.currentTimeMillis();
            telemetry.addData("TIMER: ", timer3.time() - timer5);
            telemetry.update();
//            telemetry.addLine("Oi dps do timer");
            gravacao();

            if (gamepad1.dpad_up) {
                setPowerGravado();
            }
            timer5 = timer3.time();
            sleep(20);
        }
    }

    public void gravacao() {
        if (gamepad1.y) {
            if (gravar) {
                gravar2 = !gravar2;
            }
            gravar = false;
        } else {
            gravar = true;
        }
        if (gravar2) {
            telemetry.addLine("GRAVANDO VALOR");
            // GRAVAR
            valorArmazenado[iGravacao][0] = (gamepad1.left_stick_y) * speed; // RMF
            valorArmazenado[iGravacao][1] = (gamepad1.left_stick_y) * speed; // LMF
            valorArmazenado[iGravacao][2] = (gamepad1.right_stick_y) * speed; // RMB
            valorArmazenado[iGravacao][3] = (gamepad1.right_stick_y) * speed; // LMB
            iGravacao++;
            jGravacao = iGravacao;
        } else {
            iGravacao = 0;
        }
    }
    public void setPowerGravado() {

//        timer2 = System.currentTimeMillis();
//        while (System.currentTimeMillis() - timer2 < 10000) {
//            if (System.currentTimeMillis() - timer < 20) {
//                continue;
//            }
        timer = System.currentTimeMillis();
        for (int i = 0; i < jGravacao; i++) {
            telemetry.addLine("SET POWER GRAVADOR");
            telemetry.addData("valor armazenado: ", valorArmazenado[i][0]);
            RobotLog.d("valor_armazenado_RMF: " + i + " Valor" + valorArmazenado[i][0]);
            telemetry.update();
//                RMF.setPower(valorArmazenado[i][0]);
//                LMF.setPower(valorArmazenado[i][1]);
//                RMB.setPower(valorArmazenado[i][2]);
//                LMB.setPower(valorArmazenado[i][3]);
            sleep(20);
        }
        try {
            this.saveGravacao(valorArmazenado);
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    private void saveGravacao(double [][] storedValue) throws IOException {
        System.out.println("SAVE_GRAVACAO_WORKING");
        StringBuffer builder = new StringBuffer();
        for(int i = 0; i < jGravacao; i++) {
            System.out.println("SAVE_GRAVACAO_LOOP_ONE");
            for(int j = 0; j < 4; j++) {
                System.out.println("SAVE_GRAVACAO_LOOP_TWO");
                builder.append(storedValue[i][j] + "");
                if(j < storedValue.length - 1)
                    builder.append(",");
            }
            builder.append("\n");
        }
        Date date = new Date();
        BufferedWriter writer = new BufferedWriter(new FileWriter("/storage/emulated/0/FIRST/RecordedGravacao/gravacao.txt"));
        writer.write((builder.toString()));
        writer.close();
    }

    private void loadGravacao(String pathName) throws IOException {
        BufferedReader reader = new BufferedReader(new FileReader(pathName));
        String line = "";
        int row = 0;
        while((line = reader.readLine()) != null) {
            String[] cols = line.split(",");
            int col = 0;
            for(String c : cols) {
                valorCarregado[row][col] = Double.parseDouble(c);
                col++;
            }
            row++;
        }
        reader.close();
    }
}