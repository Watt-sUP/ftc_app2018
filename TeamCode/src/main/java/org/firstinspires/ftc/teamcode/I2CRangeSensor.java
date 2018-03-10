package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

public class I2CRangeSensor
{
    private I2cDevice sensor;
    private I2cDeviceSynch reader;
    private int addr;
    byte[] cache;

    public double aParam = 5.11595056535567;
    public double bParam = 457.048400147437;
    public double cParam = -0.8061002068394054;
    public double dParam = 0.004048820370701007;
    public int    rawOpticalMinValid = 3;

    public int cmUltrasonicMax = 255;

    I2CRangeSensor(I2cDevice _sensor, int _addr)
    {
        sensor = _sensor;
        addr = _addr;
        reader = new I2cDeviceSynchImpl(sensor, I2cAddr.create8bit(addr), false);
        reader.engage();
    }

    private double getOpticalCM(int optical)
    {
        return (dParam + Math.log((-aParam + optical)/bParam))/cParam;
    }

    public double getDistanceCM()
    {
        cache = reader.read(0x04, 2);
        int ultrasonic = cache[0] & 0xFF;
        int optical = cache[1] & 0xFF;

        double cm = 0.0;

        if(optical >= rawOpticalMinValid)
        {
            cm = getOpticalCM(optical);
        }
        else
        {
            cm = ultrasonic;
        }

        return cm;
    }
}
