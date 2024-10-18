#include <iostream>
#include <cmath>

// Function to calculate h
double calculateH(double x_start, double y_start, double x_end, double y_end) {
    return std::sqrt(std::pow(x_start - x_end, 2) + std::pow(y_start - y_end, 2));
}

// Parametric functions for x(t) and y(t)
double parametricX(double t, double x_start, double x1, double x_end) {
    return (1 - t) * ((1 - t) * x_start + t * x1) + t * ((1 - t) * x1 + t * x_end);
}

double parametricY(double t, double y_start, double y1, double y_end) {
    return (1 - t) * ((1 - t) * y_start + t * y1) + t * ((1 - t) * y1 + t * y_end);
}

// Function to calculate arc length, x1, and y1
double calculateArcLength(double x_start, double y_start, double x_end,
                          double y_end, double theta_end, double d_lead,
                          int n) {
    double h = calculateH(x_start, y_start, x_end, y_end);
    double x1 = x_end - h * std::sin(theta_end) * d_lead;
    double y1 = y_end - h * std::cos(theta_end) * d_lead;

    double totalLength = 0.0;
    double dt = 1.0 / n;

    for (int i = 0; i < n; i++) {
        double t1 = i * dt;
        double t2 = (i + 1) * dt;

        double x1_val = parametricX(t1, x_start, x1, x_end);
        double y1_val = parametricY(t1, y_start, y1, y_end);
        double x2_val = parametricX(t2, x_start, x1, x_end);
        double y2_val = parametricY(t2, y_start, y1, y_end);

        double segmentLength = std::sqrt(std::pow(x2_val - x1_val, 2) + std::pow(y2_val - y1_val, 2));
        totalLength += segmentLength;
    }

    return totalLength;
}

int main() {
    for (float i = 0; i < 1.1; i+=0.1) {
        std::cout << i << ' ' <<  calculateArcLength(0, 0, 100, 100, 232*M_PI/180, i, 250) << std::endl;
    }
}