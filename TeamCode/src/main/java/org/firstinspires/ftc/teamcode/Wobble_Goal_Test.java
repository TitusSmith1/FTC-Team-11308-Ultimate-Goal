/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Wobble Goal Test", group="Linear Opmode")
public class Wobble_Goal_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor liftMotor;
    private Servo rightServo;
    private Servo leftServo;
    //private DigitalChannel button;

    //constants
    private final double LIFT_POWER = 0.8;
    private final double CLOSED_LEFT_SERVO = 0.28;
    private final double CLOSED_RIGHT_SERVO = 0.8;
    private final double OPEN_LEFT_SERVO = 0.8;
    private final double OPEN_RIGHT_SERVO = 0.28;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        leftServo = hardwareMap.get(Servo.class,"leftServo");
        rightServo = hardwareMap.get(Servo.class,"rightServo");

        // Most robots need the motor on one side to be reversed to drive forward
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        //boolean isInMode2 = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for the motor power
            double motorPower;

            //set the motor power based on the user input
            if(gamepad1.left_bumper == true){
                motorPower = LIFT_POWER;
            }
            else if(gamepad1.right_bumper == true){
                motorPower = -LIFT_POWER;
            }
            else{
                motorPower = 0.0;
            }

            //safety check on the motor power
            motorPower    = Range.clip(motorPower, -1.0, 1.0) ;

            //if we are in mode 1 use A andB buttons to toggle open and close
            //if(isInMode2 == false){
            if(gamepad1.a == true){
                leftServo.setPosition(OPEN_LEFT_SERVO);
                rightServo.setPosition(OPEN_RIGHT_SERVO);
            }
            else if(gamepad1.b == true){
                leftServo.setPosition(CLOSED_LEFT_SERVO);
                rightServo.setPosition(CLOSED_RIGHT_SERVO);
            }
            //}
            //If we are in mode 2 set the default position to closed. Allow the user to open the hand
            //by holding the A button.
            /*else {
                if (gamepad1.a == true) {
                    leftServoPos = OPEN_LEFT_SERVO;
                    rightServoPos = OPEN_RIGHT_SERVO;
                }
                else {
                    leftServoPos = CLOSED_LEFT_SERVO;
                    rightServoPos = CLOSED_RIGHT_SERVO;
                }
            }
                */
            //If the wobble goal button is pressed go to mode 2
            /*if(button.getState() == false){
                isInMode2 = true;
            }*/
            //Allow the user to reset to Mode 1 when the X button is pressed
            //if(gamepad1.x == true){
            //    isInMode2 = false;
            //}

            // Send the power to the motor
            liftMotor.setPower(motorPower);

            // Show the elapsed game time
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("GamePad A:"+gamepad1.a);
            telemetry.addLine("GamePad B:"+gamepad1.b);
            //telemetry.addLine("Button state:"+button.getState());
            telemetry.update();
        }
    }
}
