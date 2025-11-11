package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Gamepad;

public class asmGamepadEx {
    private final Gamepad gamepad;

    private ButtonsState currentState = new ButtonsState();
    private ButtonsState previousState = new ButtonsState();

    private static class ButtonsState {
        boolean a, b, x, y;
        boolean dpad_up, dpad_down, dpad_left, dpad_right;
        boolean left_bumper, right_bumper;
        boolean left_stick_button, right_stick_button;
        boolean back, start, guide;
        boolean touchpad;
        float left_trigger;
        float right_trigger;
    }

    public asmGamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        previousState = copyState(currentState);

        currentState.a = gamepad.a;
        currentState.b = gamepad.b;
        currentState.x = gamepad.x;
        currentState.y = gamepad.y;

        currentState.dpad_up = gamepad.dpad_up;
        currentState.dpad_down = gamepad.dpad_down;
        currentState.dpad_left = gamepad.dpad_left;
        currentState.dpad_right = gamepad.dpad_right;

        currentState.left_bumper = gamepad.left_bumper;
        currentState.right_bumper = gamepad.right_bumper;

        currentState.left_stick_button = gamepad.left_stick_button;
        currentState.right_stick_button = gamepad.right_stick_button;

        currentState.back = gamepad.back;
        currentState.start = gamepad.start;
        currentState.guide = gamepad.guide;

        currentState.left_trigger = gamepad.left_trigger;
        currentState.right_trigger = gamepad.right_trigger;

        try {
            currentState.touchpad = gamepad.touchpad;
        } catch (Exception e) {
            currentState.touchpad = false;
        }
    }

    private ButtonsState copyState(ButtonsState source) {
        ButtonsState copy = new ButtonsState();
        copy.a = source.a;
        copy.b = source.b;
        copy.x = source.x;
        copy.y = source.y;

        copy.dpad_up = source.dpad_up;
        copy.dpad_down = source.dpad_down;
        copy.dpad_left = source.dpad_left;
        copy.dpad_right = source.dpad_right;

        copy.left_bumper = source.left_bumper;
        copy.right_bumper = source.right_bumper;

        copy.left_stick_button = source.left_stick_button;
        copy.right_stick_button = source.right_stick_button;

        copy.back = source.back;
        copy.start = source.start;
        copy.guide = source.guide;

        copy.touchpad = source.touchpad;

        copy.left_trigger = source.left_trigger;
        copy.right_trigger = source.right_trigger;

        return copy;
    }

    public boolean isAPressed() { return currentState.a && !previousState.a; }
    public boolean isAReleased() { return !currentState.a && previousState.a; }
    public boolean isADown() { return currentState.a; }

    public boolean isBPressed() { return currentState.b && !previousState.b; }
    public boolean isBReleased() { return !currentState.b && previousState.b; }
    public boolean isBDown() { return currentState.b; }

    public boolean isXPressed() { return currentState.x && !previousState.x; }
    public boolean isXReleased() { return !currentState.x && previousState.x; }
    public boolean isXDown() { return currentState.x; }

    public boolean isYPressed() { return currentState.y && !previousState.y; }
    public boolean isYReleased() { return !currentState.y && previousState.y; }
    public boolean isYDown() { return currentState.y; }

    public boolean isDpadUpPressed() { return currentState.dpad_up && !previousState.dpad_up; }
    public boolean isDpadUpReleased() { return !currentState.dpad_up && previousState.dpad_up; }
    public boolean isDpadUpDown() { return currentState.dpad_up; }

    public boolean isDpadDownPressed() { return currentState.dpad_down && !previousState.dpad_down; }
    public boolean isDpadDownReleased() { return !currentState.dpad_down && previousState.dpad_down; }
    public boolean isDpadDownDown() { return currentState.dpad_down; }

    public boolean isDpadLeftPressed() { return currentState.dpad_left && !previousState.dpad_left; }
    public boolean isDpadLeftReleased() { return !currentState.dpad_left && previousState.dpad_left; }
    public boolean isDpadLeftDown() { return currentState.dpad_left; }

    public boolean isDpadRightPressed() { return currentState.dpad_right && !previousState.dpad_right; }
    public boolean isDpadRightReleased() { return !currentState.dpad_right && previousState.dpad_right; }
    public boolean isDpadRightDown() { return currentState.dpad_right; }

    public boolean isLeftBumperPressed() { return currentState.left_bumper && !previousState.left_bumper; }
    public boolean isLeftBumperReleased() { return !currentState.left_bumper && previousState.left_bumper; }
    public boolean isLeftBumperDown() { return currentState.left_bumper; }

    public boolean isRightBumperPressed() { return currentState.right_bumper && !previousState.right_bumper; }
    public boolean isRightBumperReleased() { return !currentState.right_bumper && previousState.right_bumper; }
    public boolean isRightBumperDown() { return currentState.right_bumper; }

    public boolean isLeftStickButtonPressed() { return currentState.left_stick_button && !previousState.left_stick_button; }
    public boolean isLeftStickButtonReleased() { return !currentState.left_stick_button && previousState.left_stick_button; }
    public boolean isLeftStickButtonDown() { return currentState.left_stick_button; }

    public boolean isRightStickButtonPressed() { return currentState.right_stick_button && !previousState.right_stick_button; }
    public boolean isRightStickButtonReleased() { return !currentState.right_stick_button && previousState.right_stick_button; }
    public boolean isRightStickButtonDown() { return currentState.right_stick_button; }

    public boolean isBackPressed() { return currentState.back && !previousState.back; }
    public boolean isBackReleased() { return !currentState.back && previousState.back; }
    public boolean isBackDown() { return currentState.back; }

    public boolean isStartPressed() { return currentState.start && !previousState.start; }
    public boolean isStartReleased() { return !currentState.start && previousState.start; }
    public boolean isStartDown() { return currentState.start; }

    public boolean isGuidePressed() { return currentState.guide && !previousState.guide; }
    public boolean isGuideReleased() { return !currentState.guide && previousState.guide; }
    public boolean isGuideDown() { return currentState.guide; }

    public boolean isTouchpadPressed() { return currentState.touchpad && !previousState.touchpad; }
    public boolean isTouchpadReleased() { return !currentState.touchpad && previousState.touchpad; }
    public boolean isTouchpadDown() { return currentState.touchpad; }

    public double getLeftStickX() { return gamepad.left_stick_x; }
    public double getLeftStickY() { return gamepad.left_stick_y; }
    public double getRightStickX() { return gamepad.right_stick_x; }
    public double getRightStickY() { return gamepad.right_stick_y; }

    public double getLeftTrigger() { return gamepad.left_trigger; }
    public double getRightTrigger() { return gamepad.right_trigger; }

    public boolean isLeftTriggerPressed(double threshold) {
        return currentState.left_trigger > threshold && previousState.left_trigger <= threshold;
    }

    public boolean isRightTriggerPressed(double threshold) {
        return currentState.right_trigger > threshold && previousState.right_trigger <= threshold;
    }

    public boolean isLeftTriggerDown() { return currentState.left_trigger > 0.1; }
    public boolean isRightTriggerDown() { return currentState.right_trigger > 0.1; }
}