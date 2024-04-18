#include <opencv2/opencv.hpp>
#include <iostream>
#include <wiringPi.h>
#include <softPwm.h>

// Define GPIO pins
#define LEFT_MOTOR_PIN 1
#define RIGHT_MOTOR_PIN 2
#define STEPPER_PIN 3

// PID Constants
const double Kp = 0.1;
const double Ki = 0.01;
const double Kd = 0.05;

// PID Variables
double setpoint = 0.0; // Setpoint for the center of the lane
double integral = 0.0;
double last_error = 0.0;

// Initialize Pi Camera
cv::VideoCapture setup_camera() {
    cv::VideoCapture cap;
    cap.open(0); // Open the default camera
    if (!cap.isOpened()) {
        std::cerr << "Error opening the camera" << std::endl;
        exit(1);
    }
    return cap;
}

// Process frame and detect lane
double detect_lane(cv::Mat &frame) {
    cv::Mat edges;
    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(frame, frame, cv::Size(7, 7), 1.5, 1.5);
    cv::Canny(frame, edges, 0, 30, 3);

    // Assuming the lane detection and center calculation logic here
    double detected_center = frame.cols / 2; // Dummy value

    return detected_center;
}

// Calculate PID
double calculate_pid(double detected_center, double actual_center) {
    double error = detected_center - actual_center;
    integral += error;
    double derivative = error - last_error;
    last_error = error;
    return Kp * error + Ki * integral + Kd * derivative;
}

// Setup GPIO
void setup_gpio() {
    wiringPiSetup();
    softPwmCreate(LEFT_MOTOR_PIN, 0, 100);
    softPwmCreate(RIGHT_MOTOR_PIN, 0, 100);
    softPwmCreate(STEPPER_PIN, 0, 100);
}

// Control Motors
void control_motors(double pid_value) {
    // Control logic for motors
    softPwmWrite(STEPPER_PIN, static_cast<int>(pid_value));
    softPwmWrite(LEFT_MOTOR_PIN, 50); // Set a constant speed
    softPwmWrite(RIGHT_MOTOR_PIN, 50);
}

int main() {
    setup_gpio();
    cv::VideoCapture cap = setup_camera();
    cv::Mat frame;

    while (true) {
        cap >> frame; // Capture a new frame
        if (frame.empty()) break;

        double detected_center = detect_lane(frame);
        double pid_output = calculate_pid(detected_center, frame.cols / 2);
        control_motors(pid_output);

        // Display the frame
        cv::imshow("Frame", frame);
        if (cv::waitKey(30) >= 0) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
