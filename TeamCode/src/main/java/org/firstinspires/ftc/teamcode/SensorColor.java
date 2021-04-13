/* Copyright (c) 2017-2020 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "At Color", group = "Sensor")
public class SensorColor extends OpMode {
  /** The colorSensor field will contain a reference to our color sensor hardware object */
  ColorRangeSensor colorSensor;
  Robot rb = new Robot(telemetry);
  boolean leftWasDown = false;
  boolean rightWasDown = false;
  boolean aWasDown = false;
  double servoPosition = .5;

  @Override public void init() {
    rb.init(hardwareMap);
    colorSensor = rb.colorSensor;
  }

  @Override
  public void loop() {
    final float[] hsvValues = new float[3];
    //Get the actual colors
    double distance = colorSensor.getDistance(DistanceUnit.INCH);
    //Convert them into HSV values, which are more useful for computer vision tasks.
    int color = colorSensor.getNormalizedColors().toColor();
    Color.colorToHSV(color, hsvValues);
    //Add all the values to telemetry
    telemetry.addLine()
            .addData("Red", Color.red(color))
            .addData("Green", Color.green(color))
            .addData("Blue", Color.blue(color));
    telemetry.addLine()
            .addData("Hue", hsvValues[0])
            .addData("Saturation", hsvValues[1])
            .addData("Value", hsvValues[2]);
    telemetry.addData("Alpha", Color.alpha(color));
    telemetry.addData("Distance", distance);
    telemetry.update();
    telemetry.addData("Servo position:", servoPosition);
    telemetry.update();

    if (gamepad1.a) {
      if (!aWasDown) {
        aWasDown = true;
        rb.sensor_servo.setPosition(servoPosition);
      }
    } else {
      aWasDown = false;
    }
    //Use the bumpers to change the angle. If the b button is pressed, change it slowly.
    if (gamepad1.left_bumper) {
      if (!leftWasDown) {
        leftWasDown = true;
        if (gamepad1.b) {
          servoPosition -= .01;
        } else {
          servoPosition -= ServoTest.INCREMENT;
        }
      }
    } else {
      leftWasDown = false;
    }
    if (gamepad1.right_bumper) {
      if (!rightWasDown) {
        rightWasDown = true;
        if (gamepad1.b) {
          servoPosition += .01;
        } else {
          servoPosition += ServoTest.INCREMENT;
        }
      }
    } else {
      rightWasDown = false;
    }
  }
}
