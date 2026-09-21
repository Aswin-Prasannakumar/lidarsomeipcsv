// ============================================================================
// lidar_subscriber_clean.cpp
//
// Subscribes to the LiDAR SOME/IP event published by the SCANeR SOME/IP plugin,
// deserialises each frame, writes all rays to a CSV file, and prints only a
// compact summary every N frames.
//
// This avoids flooding the terminal and reduces vSomeIP dispatcher overload.
//
// Binary payload layout:
//   [uint32  ray_count ]   4 bytes   bytes 0–3
//   [double  frame_ts  ]   8 bytes   bytes 4–11
//   per ray (28 bytes):
//     [double  ray_ts  ]   8 bytes
//     [uint32  ray_id  ]   4 bytes
//     [float   x       ]   4 bytes
//     [float   y       ]   4 bytes
//     [float   z       ]   4 bytes
//     [float   intensity]  4 bytes
//
// Output:
//   - Console: compact one-line summary every SUMMARY_EVERY_N_FRAMES frames
//   - CSV: lidar_points.csv in the folder where this .exe is launched
// ============================================================================

#include <vsomeip/vsomeip.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

// ?? SOME/IP identifiers (must be identical to the adapter/plugin) ?????????????
static constexpr vsomeip::service_t    SERVICE = 0x1234;
static constexpr vsomeip::instance_t   INSTANCE = 0x0001;
static constexpr vsomeip::event_t      EVENT = 0x8001;
static constexpr vsomeip::eventgroup_t EVENTGROUP = 0x0001;

// ?? Payload layout constants ?????????????????????????????????????????????????
static constexpr size_t OFF_RC = 0;
static constexpr size_t OFF_TS = sizeof(uint32_t);
static constexpr size_t HEADER_BYTES = sizeof(uint32_t) + sizeof(double);

// Offsets within a single 28-byte ray record
static constexpr size_t RAY_OFF_TS = 0;
static constexpr size_t RAY_OFF_ID = 8;
static constexpr size_t RAY_OFF_X = 12;
static constexpr size_t RAY_OFF_Y = 16;
static constexpr size_t RAY_OFF_Z = 20;
static constexpr size_t RAY_OFF_INTENSITY = 24;
static constexpr size_t RAY_BYTES = 28;

// ?? Output settings ??????????????????????????????????????????????????????????
static constexpr uint32_t SUMMARY_EVERY_N_FRAMES = 10;
static constexpr uint32_t CSV_FLUSH_EVERY_N_FRAMES = 20;
static const std::string CSV_FILENAME = "lidar_points.csv";

// ?? Signal handling ??????????????????????????????????????????????????????????
static std::atomic<bool> g_running{ true };
void on_signal(int) { g_running = false; }

// ?? Global counters/statistics ????????????????????????????????????????????????
static std::atomic<uint64_t> g_frames_received{ 0 };
static std::atomic<uint64_t> g_rays_received{ 0 };
static std::atomic<uint64_t> g_hit_rays_received{ 0 };

// ?? Thread-safe output ????????????????????????????????????????????????????????
static std::ofstream g_csv;
static std::mutex g_csv_mutex;
static std::mutex g_cout_mutex;

// ============================================================================
// LidarRay
// ============================================================================
struct LidarRay
{
    double   ray_ts = 0.0;
    uint32_t ray_id = 0;
    float    x = 0.0f;
    float    y = 0.0f;
    float    z = 0.0f;
    float    intensity = 0.0f;
};

// ============================================================================
// is_hit
//
// In the current plugin, no-hit rays are encoded as x=y=z=intensity=0.
// ============================================================================
static bool is_hit(const LidarRay& ray)
{
    return !(ray.x == 0.0f &&
        ray.y == 0.0f &&
        ray.z == 0.0f &&
        ray.intensity == 0.0f);
}

// ============================================================================
// distance_of
// ============================================================================
static float distance_of(const LidarRay& ray)
{
    return std::sqrt(ray.x * ray.x + ray.y * ray.y + ray.z * ray.z);
}

// ============================================================================
// write_csv_header
// ============================================================================
static void write_csv_header()
{
    std::lock_guard<std::mutex> lock(g_csv_mutex);

    if (g_csv.is_open()) {
        g_csv << "frame,frame_timestamp,ray_timestamp,ray_index,ray_id,"
            << "x,y,z,intensity,distance,hit\n";
        g_csv.flush();
    }
}

// ============================================================================
// deserialise_frame
//
// Reads the binary payload, writes all rays to CSV, and prints a compact
// summary every SUMMARY_EVERY_N_FRAMES frames.
// ============================================================================
static bool deserialise_frame(const std::vector<vsomeip::byte_t>& data)
{
    if (data.size() < HEADER_BYTES) {
        std::lock_guard<std::mutex> lock(g_cout_mutex);
        std::cerr << "[sub] payload too short (" << data.size() << " bytes)\n";
        return false;
    }

    // ?? Read frame header ?????????????????????????????????????????????????????
    uint32_t ray_count = 0;
    double frame_ts = 0.0;

    std::memcpy(&ray_count, data.data() + OFF_RC, sizeof(ray_count));
    std::memcpy(&frame_ts, data.data() + OFF_TS, sizeof(frame_ts));

    // ?? Validate total payload size ???????????????????????????????????????????
    const size_t expected = HEADER_BYTES + static_cast<size_t>(ray_count) * RAY_BYTES;

    if (data.size() < expected) {
        std::lock_guard<std::mutex> lock(g_cout_mutex);
        std::cerr << "[sub] payload truncated: expected "
            << expected << " bytes, got "
            << data.size() << " bytes\n";
        return false;
    }

    if (data.size() > expected) {
        std::lock_guard<std::mutex> lock(g_cout_mutex);
        std::cerr << "[sub] payload has extra bytes: expected "
            << expected << " bytes, got "
            << data.size() << " bytes. Decoding expected portion only.\n";
    }

    // Increment only after validation succeeds.
    const uint64_t frame_no = ++g_frames_received;

    // ?? Decode rays and compute compact stats ?????????????????????????????????
    std::vector<LidarRay> rays;
    rays.reserve(ray_count);

    uint32_t hit_count = 0;
    float min_distance = std::numeric_limits<float>::max();
    float max_distance = 0.0f;
    float sum_distance = 0.0f;

    const uint8_t* base = data.data() + HEADER_BYTES;

    for (uint32_t i = 0; i < ray_count; ++i) {
        const uint8_t* ray_ptr = base + static_cast<size_t>(i) * RAY_BYTES;

        LidarRay ray;

        std::memcpy(&ray.ray_ts, ray_ptr + RAY_OFF_TS, sizeof(ray.ray_ts));
        std::memcpy(&ray.ray_id, ray_ptr + RAY_OFF_ID, sizeof(ray.ray_id));
        std::memcpy(&ray.x, ray_ptr + RAY_OFF_X, sizeof(ray.x));
        std::memcpy(&ray.y, ray_ptr + RAY_OFF_Y, sizeof(ray.y));
        std::memcpy(&ray.z, ray_ptr + RAY_OFF_Z, sizeof(ray.z));
        std::memcpy(&ray.intensity, ray_ptr + RAY_OFF_INTENSITY, sizeof(ray.intensity));

        const bool hit = is_hit(ray);
        const float distance = distance_of(ray);

        if (hit) {
            ++hit_count;
            min_distance = std::min(min_distance, distance);
            max_distance = std::max(max_distance, distance);
            sum_distance += distance;
        }

        rays.push_back(ray);
    }

    g_rays_received.fetch_add(ray_count);
    g_hit_rays_received.fetch_add(hit_count);

    const float avg_distance =
        (hit_count > 0) ? (sum_distance / static_cast<float>(hit_count)) : 0.0f;

    if (hit_count == 0) {
        min_distance = 0.0f;
    }

    // ?? Write full frame to CSV ???????????????????????????????????????????????
    {
        std::lock_guard<std::mutex> lock(g_csv_mutex);

        if (g_csv.is_open()) {
            for (uint32_t i = 0; i < ray_count; ++i) {
                const auto& ray = rays[i];
                const bool hit = is_hit(ray);
                const float distance = distance_of(ray);

                g_csv << frame_no << ","
                    << std::fixed << std::setprecision(6)
                    << frame_ts << ","
                    << ray.ray_ts << ","
                    << i << ","
                    << ray.ray_id << ","
                    << ray.x << ","
                    << ray.y << ","
                    << ray.z << ","
                    << ray.intensity << ","
                    << distance << ","
                    << (hit ? 1 : 0)
                    << "\n";
            }

            if (frame_no % CSV_FLUSH_EVERY_N_FRAMES == 0) {
                g_csv.flush();
            }
        }
    }

    // ?? Print only compact summaries ??????????????????????????????????????????
    if (frame_no == 1 || frame_no % SUMMARY_EVERY_N_FRAMES == 0) {
        std::lock_guard<std::mutex> lock(g_cout_mutex);

        std::cout << "[frame " << frame_no << "]"
            << " t=" << std::fixed << std::setprecision(3) << frame_ts << "s"
            << " | rays=" << ray_count
            << " | hits=" << hit_count << "/" << ray_count
            << " | min=" << std::setprecision(2) << min_distance << "m"
            << " | avg=" << avg_distance << "m"
            << " | max=" << max_distance << "m"
            << " | payload=" << data.size() << "B"
            << "\n";
    }

    return true;
}

// ============================================================================
// main
// ============================================================================
int main()
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    // ?? Open CSV file ?????????????????????????????????????????????????????????
    g_csv.open(CSV_FILENAME, std::ios::out);

    if (!g_csv.is_open()) {
        std::cerr << "[sub] WARNING: could not open " << CSV_FILENAME
            << ". Console summary will still run, but CSV will not be saved.\n";
    }
    else {
        write_csv_header();
        std::cout << "[sub] writing decoded LiDAR rays to " << CSV_FILENAME << "\n";
    }

    // ?? Create vsomeip application ????????????????????????????????????????????
    auto app = vsomeip::runtime::get()->create_application("lidar_subscriber");
    app->init();

    // ?? Step 1: Availability handler ??????????????????????????????????????????
    app->register_availability_handler(
        SERVICE, INSTANCE,
        [&app](vsomeip::service_t svc, vsomeip::instance_t inst, bool available) {
            std::lock_guard<std::mutex> lock(g_cout_mutex);

            std::cout << "[sub] service 0x" << std::hex << svc
                << "/0x" << inst << std::dec
                << (available ? " is AVAILABLE - subscribing\n"
                    : " went OFFLINE\n");

            if (available) {
                app->subscribe(SERVICE, INSTANCE, EVENTGROUP);
            }
        }
    );

    // ?? Step 2: Message / event handler ??????????????????????????????????????
    app->register_message_handler(
        SERVICE, INSTANCE, EVENT,
        [](const std::shared_ptr<vsomeip::message>& msg) {
            auto payload = msg->get_payload();

            if (!payload) {
                std::lock_guard<std::mutex> lock(g_cout_mutex);
                std::cerr << "[sub] received message with null payload\n";
                return;
            }

            // This build uses raw-pointer payload data, so copy into a vector.
            std::vector<vsomeip::byte_t> data(
                payload->get_data(),
                payload->get_data() + payload->get_length());

            deserialise_frame(data);
        }
    );

    // ?? Step 3: Request the service ???????????????????????????????????????????
    app->request_service(SERVICE, INSTANCE);

    // ?? Step 4: Request the event ?????????????????????????????????????????????
    std::set<vsomeip::eventgroup_t> groups{ EVENTGROUP };
    app->request_event(
        SERVICE,
        INSTANCE,
        EVENT,
        groups,
        vsomeip::event_type_e::ET_FIELD
    );

    // ?? Step 5: Start vsomeip event loop on its own thread ????????????????????
    std::thread vsomeip_thread([&] { app->start(); });

    {
        std::lock_guard<std::mutex> lock(g_cout_mutex);
        std::cout << "[sub] started - waiting for service 0x"
            << std::hex << SERVICE << "/0x" << INSTANCE << std::dec << "\n";
        std::cout << "[sub] console prints frame 1 and then every "
            << SUMMARY_EVERY_N_FRAMES << " frames\n";
    }

    // ?? Main thread: idle until Ctrl-C ????????????????????????????????????????
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // ?? Shutdown ??????????????????????????????????????????????????????????????
    {
        std::lock_guard<std::mutex> lock(g_cout_mutex);

        std::cout << "\n[sub] shutting down\n"
            << "[sub] total frames received : " << g_frames_received.load() << "\n"
            << "[sub] total rays received   : " << g_rays_received.load() << "\n"
            << "[sub] total hit rays        : " << g_hit_rays_received.load() << "\n";

        if (g_csv.is_open()) {
            std::cout << "[sub] CSV saved to         : " << CSV_FILENAME << "\n";
        }
    }

    app->stop();

    if (vsomeip_thread.joinable()) {
        vsomeip_thread.join();
    }

    {
        std::lock_guard<std::mutex> lock(g_csv_mutex);
        if (g_csv.is_open()) {
            g_csv.flush();
            g_csv.close();
        }
    }

    return 0;
}
