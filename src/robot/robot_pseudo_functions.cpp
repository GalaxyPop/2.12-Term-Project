#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>

// Global atomic flag to communicate between the vision thread and the drive sequence
std::atomic<bool> obstacle_detected(false);

// ---------------------------------------------------------
// Subsystem & Navigation Placeholder Functions
// ---------------------------------------------------------

// Vision & Targeting
void findAndAlignAprilTag(int tag_id);
void scanForFoodTrayPrepTag();
void scanForDinnerTableTag();
void scanForDishwasherTrayTag();
void scanForDishwasherTag();

// Basic Movement (Non-Ramp)
void navigateForwardUntil(std::string condition);
void moveForwardPresetDistance();
void reversePresetDistance();
void turnRight90();
void turnLeft90();
void makeInformedAdjustments();

// Ramp Traversals (Empty)
void traverseEmptySlopeA_Up();
void traverseEmptySlopeB_Down();
void reverseEmptySlopeB_Up();
void reverseEmptySlopeA_Down();

// Ramp Traversals (Loaded - Includes IMU Balancing)
void traverseLoadedSlopeA_Up();
void reverseLoadedSlopeA_Down();
void reverseLoadedSlopeB_Up();
void traverseLoadedSlopeB_Down();

// Manipulation
void positionChassisForGrip();
void grabTray();
void depositTray();

// Path Planning
void executePathPlanTo(std::string destination);

// ---------------------------------------------------------
// Concurrent Obstacle Detection Thread
// ---------------------------------------------------------
void scanForMovingObstacles() {
    std::cout << "[THREAD] Obstacle detection active on RGB/Depth camera..." << std::endl;
    while (true) {
        // Implement your object detection inference here (e.g., YOLOv8 or Realsense depth check)
        bool obstacle_in_path = false; // Replace with actual camera inference
        
        if (obstacle_in_path) {
            obstacle_detected = true;
            // Send emergency brake command to chassis here
        } else {
            obstacle_detected = false;
        }
        
        // Brief sleep to yield CPU cycles
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

// Helper wrapper to ensure movement commands check the obstacle flag
void safeExecute(void (*movementFunction)()) {
    while (obstacle_detected) {
        std::cout << "[HALT] Moving obstacle detected! Waiting..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    movementFunction();
}


// ---------------------------------------------------------
// Main Execution Sequence
// ---------------------------------------------------------
int main() {
    std::cout << "Starting Mobile Robot Autonomous Sequence..." << std::endl;

    // Step 3: Initialize the concurrent obstacle scanning thread
    std::thread vision_thread(scanForMovingObstacles);
    vision_thread.detach(); 

    // Steps 1-2: Initial Ramp Approach
    findAndAlignAprilTag(1); // Assuming 1 is the ramp tag
    safeExecute([](){ navigateForwardUntil("base_of_slope_A"); });

    // Steps 4-6: Empty Traversal 1
    safeExecute(traverseEmptySlopeA_Up);
    safeExecute(turnRight90);
    safeExecute(traverseEmptySlopeB_Down);

    // Steps 7-9: Empty Traversal 2 (Reverse)
    safeExecute(reverseEmptySlopeB_Up);
    safeExecute(turnLeft90); // Doing a reverse turn at the peak
    safeExecute(reverseEmptySlopeA_Down);

    // Steps 10-12: Empty Traversal 3
    safeExecute(traverseEmptySlopeA_Up);
    safeExecute(turnRight90);
    safeExecute(traverseEmptySlopeB_Down);

    // Steps 13-14: Approach Preset Tray
    safeExecute([](){ navigateForwardUntil("preset_tray_tag_visible"); });
    safeExecute(moveForwardPresetDistance);

    // Steps 15-18: Grab Food Tray
    safeExecute(turnRight90);
    scanForFoodTrayPrepTag();
    safeExecute(positionChassisForGrip);
    grabTray();

    // Steps 19-22: Prep for Loaded Ramp Traversal
    safeExecute(reversePresetDistance);
    safeExecute(turnRight90);
    findAndAlignAprilTag(2); // Assuming 2 is the return ramp tag
    executePathPlanTo("base_of_slope_A");

    // Steps 23-25: Loaded Slope A (Max Points Loop)
    for(int i = 0; i < 3; i++) {
        safeExecute(traverseLoadedSlopeA_Up);   
        safeExecute(reverseLoadedSlopeA_Down);  
    }

    // Step 26: Cross the peak to Slope B
    safeExecute(traverseLoadedSlopeA_Up);
    safeExecute(turnRight90);
    safeExecute(traverseLoadedSlopeB_Down);

    // Steps 27-29: Loaded Slope B (Max Points Loop)
    for(int i = 0; i < 3; i++) {
        safeExecute(reverseLoadedSlopeB_Up);    
        safeExecute(traverseLoadedSlopeB_Down); 
    }

    // Steps 30-33: Dinner Table Delivery
    executePathPlanTo("dinner_table_vicinity");
    scanForDinnerTableTag();
    safeExecute(makeInformedAdjustments);
    depositTray();

    // Steps 34-36: Retrieve Dishwasher Tray
    executePathPlanTo("dishwasher_tray_vicinity");
    scanForDishwasherTrayTag();
    safeExecute(makeInformedAdjustments);
    grabTray();

    // Steps 37-39: Dishwasher Delivery
    executePathPlanTo("dishwasher_vicinity");
    scanForDishwasherTag();
    safeExecute(makeInformedAdjustments);
    depositTray();

    std::cout << "Mobile Robot Sequence Complete. Total tasks executed." << std::endl;

    return 0;
}