#include "main.h"
#include "generate_jerryio.h"

#include <fstream>
#include <iostream>
#include <random>


// Initialize the random number generator (seeded once with time)
std::string generateRandomUID(int length = 10) {
    static std::mt19937 rng; // Random number generator, static to keep state between calls
    const std::string alphanumeric = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    const int charactersLength = alphanumeric.size();

    std::string uid;

    // Use the random number generator `rng` to pick random characters from the alphabet
    std::uniform_int_distribution<int> dist(0, charactersLength - 1); // Uniform distribution

    for (int i = 0; i < length; ++i) {
        uid += alphanumeric[dist(rng)];
    }

    return uid;
}

std::vector<lemlib::Pose> changeUnits(const std::vector<lemlib::Pose> &poses) {
    std::vector<lemlib::Pose> newPoses;
    for (int i = 0; i < poses.size(); ++i) {
        newPoses.emplace_back(poses[i]*2.54);
    }
    return newPoses;
}

void generatePathFile(const std::string &filename) {
    std::vector<lemlib::Pose> newPoses=changeUnits(movements);
    // Open the file for writing
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return;
    }

    // Start the JSON structure
    file << "#PATH.JERRYIO-DATA";
    file << "{";
    file << "\"appVersion\":\"0.8.3\",";
    file << "\"format\":\"Move-to-Point Code Gen v0.1\",";
    file << "\"gc\":{";
    file << "\"robotWidth\":30,";
    file << "\"robotHeight\":30,";
    file << "\"showRobot\":false,";
    file << "\"uol\":1,";
    file << "\"pointDensity\":2,";
    file << "\"controlMagnetDistance\":5,";
    file << "\"fieldImage\":{";
    if(skills_field) {
        file << "\"displayName\":\"V5RC 2025 - High Stakes (Skills)\",";
        file << "\"signature\":\"V5RC 2025 - High Stakes (Skills)\",";
    }else {
        file << "\"displayName\":\"V5RC 2025 - High Stakes\",";
        file << "\"signature\":\"V5RC 2025 - High Stakes\",";
    }
    file << "\"origin\":{\"__type\":\"built-in\"}";
    file << "},";
    file << "\"coordinateSystem\":\"VEX Gaming Positioning System\",";
    file <<
            "\"outputTemplate\":\"path:`//${name}\\n\\n${code}\\n`\\nmoveToPoint:`moveToPoint(${x}, ${y}, ${heading}, ${speed});`\"";
    file << "},";
    file << "\"paths\":[";

    file << "{";
    file << "\"segments\":[";

    // Create the controls in pairs: point1 -> point2, point2 -> point3, etc.
    for (size_t i = 0; i < newPoses.size() - 1; ++i) {
        file << "{";
        file << "\"controls\":[";

        // Control for points[i] and points[i+1]
        for (int j = 0; j < 2; ++j) {
            float x = newPoses[i + j].x;
            float y = newPoses[i + j].y;
            float theta = newPoses[i + j].theta;

            file << "{";
            file << "\"uid\":\"" << generateRandomUID() << "\","; // Random UID
            file << "\"x\":" << x << ",";
            file << "\"y\":" << y << ",";
            file << "\"heading\":" << theta << ",";
            file << "\"lock\":false,";
            file << "\"visible\":true,";
            file << "\"__type\":\"end-point\"";
            file << "}";

            // If it's not the last control, add a comma
            if (j == 0) {
                file << ",";
            }
        }

        file << "],"; // Close controls array

        file << "\"speedProfiles\":[],";
        file << "\"lookaheadKeyframes\":[],";
        file << "\"uid\":\"" << generateRandomUID() << "\""; // Random UID for this segment
        file << "}";

        // Add a comma after the segment if it's not the last one
        if (i < newPoses.size() - 2) {
            file << ",";
        }
    }

    file << "],"; // End of segments array
    file << "\"pc\":{";
    file << "\"speed\":30";
    file << "},";
    file << "\"name\":\"Path\",";
    file << "\"uid\":\"" << generateRandomUID() << "\",";
    file << "\"lock\":false,";
    file << "\"visible\":true";
    file << "}";

    file << "]}";

    // Close the file
    file.close();
    std::cout << "Path file generated: " << filename << std::endl;
}
