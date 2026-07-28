#include <string>
#include <openvr.h>
#include <chrono> 
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>

#include "VRUtils.hpp"
#include "json.hpp" 
#include "server.hpp"

// VR Input Configuration Constants
namespace VRInputConfig {
    constexpr double DEFAULT_PUBLISH_FREQUENCY = 50.0;
    constexpr double MIN_PUBLISH_FREQUENCY = 0.1;
    constexpr double MAX_PUBLISH_FREQUENCY = 1000.0;
    constexpr int TRIGGER_FEEDBACK_STEPS = 6;
    constexpr int HAPTIC_FEEDBACK_DURATION = 200;
    constexpr int TRIGGER_HAPTIC_MULTIPLIER = 3000;
    constexpr int WARNING_HAPTIC_DURATION = 20;
    constexpr float POSITION_THRESHOLD = 0.05f;
    constexpr float DISTANCE_THRESHOLD = 0.1f;
    constexpr int NO_CONTROLLER_SLEEP_MS = 50;
    constexpr int DATA_COLLECTION_SLEEP_MS = 10;  
    constexpr int CONTROLLER_DELAY_MS = 1;
    constexpr int LOG_INTERVAL_SECONDS = 1;
    constexpr double Y_OFFSET = 0.6;
    
    constexpr int RIGHT_CONTROLLER_INDEX = 0;
    constexpr int LEFT_CONTROLLER_INDEX = 1;
    constexpr int MAX_CONTROLLERS = 16;
}

class ViveInput {
public:
    ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data, double publish_freq = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY);
    ~ViveInput();
    void runVR();
    void setPublishFrequency(double freq);

private:
    vr::IVRSystem *pHMD = nullptr;
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

    std::mutex &data_mutex;
    std::condition_variable &data_cv;
    VRControllerData &shared_data;
    VRControllerData local_data;
    
    bool controller_detected[VRInputConfig::MAX_CONTROLLERS] = {false};
    Eigen::Vector3d prev_position[VRInputConfig::MAX_CONTROLLERS];  
    std::chrono::steady_clock::time_point prev_time[VRInputConfig::MAX_CONTROLLERS];
    bool first_run[VRInputConfig::MAX_CONTROLLERS] = {true};
    
    VRControllerData right_controller_data; 
    VRControllerData left_controller_data;  
    bool right_controller_updated = false;
    bool left_controller_updated = false;
    
    double controllerPublishFrequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
    std::chrono::steady_clock::time_point last_publish_time[VRInputConfig::MAX_CONTROLLERS]; 
    bool publish_time_initialized[VRInputConfig::MAX_CONTROLLERS] = {false};

    bool initVR();
    bool shutdownVR();
};

ViveInput::ViveInput(std::mutex &mutex, std::condition_variable &cv, VRControllerData &data, double publish_freq) 
    : data_mutex(mutex), data_cv(cv), shared_data(data), controllerPublishFrequency(publish_freq) {
    if (!initVR()) {
        shutdownVR();
        throw std::runtime_error("Failed to initialize VR");
    }
    for (int i = 0; i < VRInputConfig::MAX_CONTROLLERS; i++) {
        first_run[i] = true;
        publish_time_initialized[i] = false;
    }
}

ViveInput::~ViveInput() {
    shutdownVR();
}

void ViveInput::setPublishFrequency(double freq) {
    if (freq > VRInputConfig::MIN_PUBLISH_FREQUENCY && freq <= VRInputConfig::MAX_PUBLISH_FREQUENCY) {
        controllerPublishFrequency = freq;
        logMessage(Info, "Controller publish frequency updated to: " + std::to_string(freq) + " Hz");
        for(int i = 0; i < VRInputConfig::MAX_CONTROLLERS; i++) {
            publish_time_initialized[i] = false;
        }
    }
}

void ViveInput::runVR() {
    logMessage(Info, "Starting VR loop");
    logMessage(Info, "Controller publish frequency set to: " + std::to_string(controllerPublishFrequency) + " Hz");
    
    static int previousStep[VRInputConfig::MAX_CONTROLLERS] = {-1, -1}; 

    VRControllerData tracker_data_array[8];
    bool tracker_updated_array[8] = {false, false, false, false, false, false, false, false};

    while (true) {
        
        for (int r = 0; r < 8; r++) {
            controller_detected[r] = false;
            tracker_updated_array[r] = false;
        }
        
        right_controller_updated = false;
        left_controller_updated = false;
        
        
        pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, trackedDevicePose, vr::k_unMaxTrackedDeviceCount);


        for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            if (trackedDevicePose[i].bDeviceIsConnected) {
                if (VRUtils::controllerIsConnected(pHMD, i) || pHMD->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_GenericTracker) {
                    vr::ETrackedControllerRole controllerRole = VRUtils::controllerRoleCheck(pHMD, i);

                    if (controllerRole == vr::TrackedControllerRole_LeftHand || 
                        controllerRole == vr::TrackedControllerRole_RightHand || 
                        pHMD->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_GenericTracker) {

                        char buf[128];
                        pHMD->GetStringTrackedDeviceProperty(i, vr::Prop_SerialNumber_String, buf, sizeof(buf));
                        std::string tracker_sn(buf);
                        int role_index = -1;

                        if (tracker_sn == "LHR-D6B44530") { role_index = 0; } 
                        else if (tracker_sn == "LHR-481141A3") { role_index = 1; } 
                        else if (tracker_sn == "LHR-2E8F88BF") { role_index = 2; }
                        else if (tracker_sn == "LHR-27D31A08") { role_index = 3; }
                        else if (tracker_sn == "LHR-0D0FABD7") { role_index = 4; }
                        else if (tracker_sn == "LHR-AF2E6E8C") { role_index = 5; }
                        else if (tracker_sn == "LHR-D6B44530") { role_index = 6; }
                        else if (tracker_sn == "LHR-TRACKER8") { role_index = 7; }
                        else { continue; }

                        controller_detected[role_index] = true;


                        if (trackedDevicePose[i].bPoseIsValid && trackedDevicePose[i].eTrackingResult == vr::TrackingResult_Running_OK) {
                            
                            vr::HmdMatrix34_t steamVRMatrix = trackedDevicePose[i].mDeviceToAbsoluteTracking;
                            Eigen::Vector3d position = VRTransforms::getPositionFromVRMatrix(steamVRMatrix);
                            Eigen::Quaterniond quaternion = VRTransforms::getQuaternionFromVRMatrix(steamVRMatrix);

                            VRUtils::resetJsonData(local_data);
                            local_data.time = Server::getCurrentTimeWithMilliseconds();
                            local_data.role = role_index; 
                            local_data.setPosition(position - Eigen::Vector3d(0, VRInputConfig::Y_OFFSET, 0)); 
                            local_data.setQuaternion(quaternion);

                            // 處理按鍵狀態
                            vr::VRControllerState_t controllerState;
                            pHMD->GetControllerState(i, &controllerState, sizeof(controllerState));

                            if ((1LL << vr::k_EButton_ApplicationMenu) & controllerState.ulButtonPressed) {
                                first_run[role_index] = true; 
                                local_data.menu_button = true;
                                VRUtils::HapticFeedback(pHMD, i, 200);
                            }
                            if ((1LL << vr::k_EButton_SteamVR_Trigger) & controllerState.ulButtonPressed) {
                                local_data.trigger_button = true;
                            }
                            if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonPressed) {
                                local_data.trackpad_button = true;
                                VRUtils::HapticFeedback(pHMD, i, 200);
                            }
                            if ((1LL << vr::k_EButton_Grip) & controllerState.ulButtonPressed) {
                                local_data.grip_button = true;
                            }
                            if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonTouched) {
                                local_data.trackpad_x = controllerState.rAxis[0].x;
                                local_data.trackpad_y = controllerState.rAxis[0].y;
                                local_data.trackpad_touch = true;
                            }
                                    
                            float triggerValue = controllerState.rAxis[1].x;
                            int currentStep = static_cast<int>(triggerValue / (1.0f / 6));
                            local_data.trigger = triggerValue;
                                    
                            if (currentStep != previousStep[role_index]) {
                                VRUtils::HapticFeedback(pHMD, i, static_cast<int>(triggerValue * 3000));
                                previousStep[role_index] = currentStep;
                            }

                            // 速度合理性防線（跳躍時強刷）
                            auto current_time = std::chrono::steady_clock::now();
                            if (!first_run[role_index]) {
                                if (!VRTransforms::isPositionChangeReasonable(position, prev_position[role_index], 0.05)) {
                                    logMessage(Warning, "[CONTROLLER " + std::to_string(role_index) + "] Unreasonable distance. Resync.");
                                    prev_position[role_index] = position; 
                                }
                            } else {
                                first_run[role_index] = false; 
                            }

                            prev_position[role_index] = position;
                            prev_time[role_index] = current_time;

                            tracker_data_array[role_index] = local_data;
                            tracker_updated_array[role_index] = true;
                        }
                    } 
                } 
            }
        } 


        auto currentTime = std::chrono::steady_clock::now();
        double interval_ms = 1000.0 / controllerPublishFrequency;
        auto interval_duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double, std::milli>(interval_ms));
            
        for (int role = 0; role < 8; role++) {
            if (tracker_updated_array[role]) {
                if (!publish_time_initialized[role]) {
                    last_publish_time[role] = currentTime;
                    publish_time_initialized[role] = true;
                }
                    
                auto time_since_last_publish = currentTime - last_publish_time[role];
                if (time_since_last_publish >= interval_duration) {
                    {
                        std::lock_guard<std::mutex> lock(data_mutex);
                        shared_data = tracker_data_array[role];
                        shared_data.time = Server::getCurrentTimeWithMilliseconds();
                        shared_data.role = role;
                        data_cv.notify_one(); 
                    }
                        
                    last_publish_time[role] = currentTime;
                    logMessage(Debug, "[Tracker " + std::to_string(role + 1) + "] Data sent to server");
                }
            }
        }
            

        std::this_thread::sleep_for(std::chrono::milliseconds(1)); 
            
        // 重設旗標
        right_controller_updated = false;
        left_controller_updated = false;


        std::string detected_str = "Detected trackers: ";
        bool any_detected = false;
        for (int i = 0; i < 8; i++) {
            if (controller_detected[i]) {
                detected_str += "Tracker" + std::to_string(i + 1) + " ";
                any_detected = true;
            }
        }
        if (any_detected) {
            logMessage(Debug, detected_str.c_str());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(11));
    } 
}

bool ViveInput::initVR() {
    eError = vr::VRInitError_None;
    pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);
    if (eError != vr::VRInitError_None) {
        pHMD = NULL;
        std::string error_msg = vr::VR_GetVRInitErrorAsEnglishDescription(eError);
        logMessage(Error, "Unable to init VR runtime: " + error_msg);
        return false;
    } else {
        logMessage(Info, "VR runtime initialized");
    }
    return true;
}

bool ViveInput::shutdownVR() {
    if (pHMD) {
        logMessage(Info, "Shutting down VR runtime");
        vr::VR_Shutdown();
    }
    return true;
}

int main(int argc, char **argv) {
    Server::setupSignalHandlers();

    std::mutex data_mutex;
    std::condition_variable data_cv;
    VRControllerData shared_data; 
    
    double publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
    
    if (argc > 1) {
        try {
            publish_frequency = std::stod(argv[1]);
            if (publish_frequency <= VRInputConfig::MIN_PUBLISH_FREQUENCY || publish_frequency > VRInputConfig::MAX_PUBLISH_FREQUENCY) {
                publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
            }
        } catch (const std::exception& e) {
            publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
        }
    }

    Server server(12345, data_mutex, data_cv, shared_data);
    
    std::thread serverThread(&Server::start, &server);
    serverThread.detach(); 

    ViveInput vive_input(data_mutex, data_cv, shared_data, publish_frequency);
    vive_input.runVR();

    return 0;
}
