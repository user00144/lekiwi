// Independent USB monocular runner. No robot, ZeroMQ, radar or IMU access.
#include <System.h>
#include <Tracking.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) { interrupted = 1; }
double seconds(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(a - b).count();
}
const char* state_name(int state) {
    switch (state) {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY: return "SYSTEM_NOT_READY";
    case ORB_SLAM3::Tracking::NO_IMAGES_YET: return "NO_IMAGES_YET";
    case ORB_SLAM3::Tracking::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case ORB_SLAM3::Tracking::OK: return "OK";
    case ORB_SLAM3::Tracking::RECENTLY_LOST: return "RECENTLY_LOST";
    case ORB_SLAM3::Tracking::LOST: return "LOST";
    default: return "OTHER";
    }
}

int main(int argc, char** argv) {
    const char* usage = "Usage: mono_usb VOCABULARY SETTINGS --output NEW_DIRECTORY "
                        "[--device /dev/v4l/by-id/...] [--seconds 120] [--no-viewer]\n";
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << usage;
        return 0;
    }
    if (argc < 5) { std::cerr << usage; return 2; }
    std::string device = "/dev/v4l/by-id/usb-Arducam_Arducam_5MP_USB_Camera_YL20240927V0-video-index0";
    fs::path output;
    double duration = 120;
    bool viewer = true;
    cv::VideoCapture cap;
    std::unique_ptr<ORB_SLAM3::System> slam;
    std::ofstream trajectory, metrics, session;
    std::size_t frames = 0, valid_poses = 0;
    int exit_code = 0;
    std::string end_reason = "duration_complete";
    try {
        for (int i = 3; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--no-viewer") viewer = false;
            else if ((option == "--output" || option == "--device" || option == "--seconds") && i + 1 < argc) {
                const std::string value = argv[++i];
                if (option == "--output") output = value;
                else if (option == "--device") device = value;
                else {
                    std::size_t parsed = 0;
                    duration = std::stod(value, &parsed);
                    if (parsed != value.size()) throw std::runtime_error("Invalid duration");
                }
            } else throw std::runtime_error("Unknown or incomplete option: " + option);
        }
        if (output.empty() || !std::isfinite(duration) || duration <= 0)
            throw std::runtime_error("Specify --output and a positive duration");
        if (viewer && (!std::getenv("DISPLAY") || !*std::getenv("DISPLAY")))
            throw std::runtime_error("DISPLAY is empty. Use the Orin Desktop terminal or --no-viewer.");
        if (!fs::is_regular_file(argv[1]) || !fs::is_regular_file(argv[2]))
            throw std::runtime_error("Vocabulary or settings file not found");
        const std::string vocabulary_path = fs::absolute(argv[1]).string();
        const std::string settings_path = fs::absolute(argv[2]).string();
        if (!fs::exists(device)) throw std::runtime_error("Camera device not found: " + device);
        cv::FileStorage config(argv[2], cv::FileStorage::READ);
        if (!config.isOpened() || static_cast<int>(config["Camera.width"]) != 640 ||
            static_cast<int>(config["Camera.height"]) != 480 || config["Camera.RGB"].empty() ||
            static_cast<int>(config["Camera.RGB"]) != 0)
            throw std::runtime_error("This runner requires calibrated raw BGR 640x480 settings");
        config.release();
        if (!fs::create_directories(output))
            throw std::runtime_error("Output directory already exists; choose a new one");
        output = fs::absolute(output);
        fs::copy_file(argv[2], output / "OV5648.yaml");
        session.open(output / "session.txt");
        trajectory.open(output / "CameraTrajectoryOnlineTUM.txt");
        metrics.open(output / "tracking.csv");
        if (!session || !trajectory || !metrics) throw std::runtime_error("Cannot open output files");
        trajectory << std::fixed << std::setprecision(9);
        metrics << "timestamp_s,state,state_name,track_ms,loop_fps,keypoints,pose_valid,big_map_change\n";
        session << "device=" << device << "\nvocabulary=" << fs::absolute(argv[1])
                << "\nsettings=" << fs::absolute(argv[2]) << "\nviewer=" << viewer
                << "\nduration_s=" << duration
                << "\ntimestamp=host steady_clock seconds since capture start, after frame read; not exposure timestamp"
                << "\npose=Twc camera-to-world; translation has arbitrary monocular scale"
                << "\nonline_trajectory=only Tracking::OK; not retrospectively optimized; map resets may change world frame"
                << "\nkeyframe_trajectory=ORB-SLAM3 final TUM export; may contain multiple maps"
                << "\n";
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
        // The upstream Viewer's Stop action also exports files relative to cwd.
        fs::current_path(output);
        // Load vocabulary before opening camera, avoiding stale startup frames.
        slam = std::make_unique<ORB_SLAM3::System>(vocabulary_path, settings_path, ORB_SLAM3::System::MONOCULAR, viewer);
        if (!cap.open(device, cv::CAP_V4L2)) throw std::runtime_error("Camera open failed");
        for (const auto& setting : {
            std::pair<int, double>{cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G')},
            {cv::CAP_PROP_FRAME_WIDTH, 640}, {cv::CAP_PROP_FRAME_HEIGHT, 480},
            {cv::CAP_PROP_FPS, 30}, {cv::CAP_PROP_BUFFERSIZE, 1}}) {
            if (!cap.set(setting.first, setting.second))
                std::cerr << "Capture property not accepted: " << setting.first << '\n';
        }
        session << "reported_fps=" << cap.get(cv::CAP_PROP_FPS)
                << "\nreported_fourcc=" << cap.get(cv::CAP_PROP_FOURCC)
                << "\nreported_autofocus=" << cap.get(cv::CAP_PROP_AUTOFOCUS)
                << "\nreported_focus=" << cap.get(cv::CAP_PROP_FOCUS) << std::endl;
        if (cap.get(cv::CAP_PROP_AUTOFOCUS) > 0.5)
            throw std::runtime_error("Autofocus enabled: restore the calibrated fixed-focus condition first");
        const auto started = Clock::now();
        auto changed = started, logged = started;
        cv::Mat previous;
        std::cout << "Camera started. Move slowly sideways past textured objects to initialize.\n"
                  << "Stop with Ctrl+C in this terminal; default duration is 120 seconds.\n"
                  << "Results: " << output << std::endl;
        while (!interrupted && !slam->isShutDown()) {
            const auto loop_start = Clock::now();
            if (seconds(loop_start, started) >= duration) break;
            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) throw std::runtime_error("Camera frame read failed");
            if (frame.cols != 640 || frame.rows != 480 || frame.type() != CV_8UC3)
                throw std::runtime_error("Actual frame format differs from calibrated BGR 640x480");
            const auto received = Clock::now();
            if (!previous.empty() && cv::norm(previous, frame, cv::NORM_INF) == 0) {
                if (seconds(received, changed) >= 3)
                    throw std::runtime_error("Identical pixels repeated for 3 seconds; capture stopped");
                continue;
            }
            previous = frame.clone();
            changed = received;
            const double timestamp = seconds(received, started);
            const auto track_start = Clock::now();
            const Sophus::SE3f Tcw = slam->TrackMonocular(frame, timestamp);
            const auto tracked = Clock::now();
            if (slam->isShutDown()) { end_reason = "viewer_stop"; break; }
            const int state = slam->GetTrackingState();
            const bool valid = state == ORB_SLAM3::Tracking::OK && Tcw.matrix().allFinite();
            if (valid) {
                const Sophus::SE3f Twc = Tcw.inverse();
                const Eigen::Vector3f position = Twc.translation();
                Eigen::Quaternionf q(Twc.rotationMatrix());
                q.normalize();
                trajectory << timestamp << ' ' << position.x() << ' ' << position.y() << ' ' << position.z()
                           << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
                ++valid_poses;
            }
            ++frames;
            const double track_ms = seconds(tracked, track_start) * 1000;
            const double fps = 1.0 / std::max(seconds(tracked, loop_start), 1e-9);
            metrics << std::fixed << std::setprecision(6) << timestamp << ',' << state << ',' << state_name(state)
                    << ',' << track_ms << ',' << fps << ',' << slam->GetTrackedKeyPointsUn().size()
                    << ',' << valid << ',' << slam->MapChanged() << '\n';
            if (seconds(tracked, logged) >= 1) {
                std::cout << state_name(state) << " | tracking " << track_ms << " ms | loop " << fps
                          << " FPS | valid poses " << valid_poses << std::endl;
                trajectory.flush(); metrics.flush();
                logged = tracked;
            }
        }
        if (interrupted) end_reason = "signal";
        else if (slam->isShutDown()) end_reason = "viewer_stop";
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << std::endl;
        end_reason = error.what(); exit_code = 1;
    }
    cap.release();
    if (slam) {
        std::cout << "Waiting for SLAM workers to finish before export..." << std::endl;
        slam->Shutdown();
        // Shutdown must include the supplied worker-wait fix.
        if (valid_poses && !output.empty())
            slam->SaveKeyFrameTrajectoryTUM((output / "KeyFrameTrajectoryTUM.txt").string());
    }
    if (slam && valid_poses == 0 && exit_code == 0) {
        end_reason = "no_valid_tracking_pose";
        exit_code = 1;
    }
    if (session) session << "end_reason=" << end_reason << "\nprocessed_frames=" << frames
                         << "\nvalid_poses=" << valid_poses << "\nexit_code=" << exit_code << std::endl;
    if (slam && valid_poses == 0) std::cerr << "No valid tracking poses; initialization/tracking was not verified.\n";
    std::cout << "Finished: " << frames << " frames, " << valid_poses << " valid poses.\n";
    return exit_code;
}
