// ============================================================================
// LidarCSV_SOMEIP_Publisher.cpp
// CSV format:
//   time,rayId,X,Y,Z,intensity
//
// Each unique timestamp represents one LiDAR frame.
// Expected frame:
//   36 rays
//
// SOME/IP:
//   Service    0x1234
//   Instance   0x0001
//   Event      0x8001
//   Eventgroup 0x0001
//
// Application payload:
//
//   uint32  ray_count
//   double  frame_timestamp
//
//   For every ray:
//       double  ray_timestamp
//       uint32  ray_id
//       float   x
//       float   y
//       float   z
//       float   intensity
//
// For 36 rays:
//
//   4 + 8 + (36 * 28) = 1020 bytes
//
// ============================================================================

#include <vsomeip/vsomeip.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>


// ============================================================================
// CONFIGURATION
// ============================================================================

// Change this.
//
// D:\Work_CARS\Lidar\data\lidar_data.csv
//
static const std::string CSV_FILE =
R"(D:\Work_CARS\Lidar\data\lidar_data.csv)";


// Expected number of rays per LiDAR frame.
static constexpr std::size_t EXPECTED_RAYS_PER_FRAME = 36;


// If true, reproduce the timing contained in the CSV.
//
// Example:
//
// frame 1 = 0.100426
// frame 2 = 0.209934
//
// publisher waits:
// 0.209934 - 0.100426
// = 0.109508 s
//
static constexpr bool REPLAY_TIMING = true;


// ============================================================================
// SOME/IP IDENTIFIERS
// ============================================================================

static constexpr vsomeip::service_t
LIDAR_SERVICE = 0x1234;

static constexpr vsomeip::instance_t
LIDAR_INSTANCE = 0x0001;

static constexpr vsomeip::event_t
LIDAR_EVENT = 0x8001;

static constexpr vsomeip::eventgroup_t
LIDAR_EVENTGROUP = 0x0001;


// ============================================================================
// PAYLOAD CONSTANTS
// ============================================================================

static constexpr std::size_t
RAY_BYTES =
sizeof(double) +      // ray timestamp
sizeof(uint32_t) +    // ray ID
sizeof(float) +       // X
sizeof(float) +       // Y
sizeof(float) +       // Z
sizeof(float);        // intensity


static constexpr std::size_t
HEADER_BYTES =
sizeof(uint32_t) +    // ray count
sizeof(double);       // frame timestamp


// ============================================================================
// LiDAR RAY
// ============================================================================

struct LidarRay
{
    double timestamp;
    uint32_t ray_id;

    float x;
    float y;
    float z;
    float intensity;
};


// ============================================================================
// FRAME
// ============================================================================

struct LidarFrame
{
    double timestamp;
    std::vector<LidarRay> rays;
};


// ============================================================================
// APPEND RAW BYTES
//
// This intentionally matches the serialization used by your SCANeR plugin.
//
// Your existing plugin uses the same raw-byte approach with push_bytes().
// ============================================================================

template <typename T>
static void push_bytes(
    std::vector<uint8_t>& buffer,
    const T& value)
{
    const uint8_t* ptr =
        reinterpret_cast<const uint8_t*>(&value);

    buffer.insert(
        buffer.end(),
        ptr,
        ptr + sizeof(T));
}


// ============================================================================
// BUILD APPLICATION PAYLOAD
// ============================================================================
//
// Payload:
//
// byte 0-3:
//     uint32 ray_count
//
// byte 4-11:
//     double frame_timestamp
//
// byte 12 onward:
//
//     Ray 0:
//         double timestamp
//         uint32 ray_id
//         float x
//         float y
//         float z
//         float intensity
//
//     Ray 1:
//         ...
//
// ============================================================================

static std::vector<uint8_t> build_payload(
    const LidarFrame& frame)
{
    const uint32_t ray_count =
        static_cast<uint32_t>(frame.rays.size());

    const std::size_t expected_size =
        HEADER_BYTES +
        frame.rays.size() * RAY_BYTES;

    std::vector<uint8_t> payload;

    payload.reserve(expected_size);

    // --------------------------------------------------------
    // Payload header
    // --------------------------------------------------------

    push_bytes(
        payload,
        ray_count);

    push_bytes(
        payload,
        frame.timestamp);


    // --------------------------------------------------------
    // Ray data
    // --------------------------------------------------------

    for (const auto& ray : frame.rays)
    {
        push_bytes(
            payload,
            ray.timestamp);

        push_bytes(
            payload,
            ray.ray_id);

        push_bytes(
            payload,
            ray.x);

        push_bytes(
            payload,
            ray.y);

        push_bytes(
            payload,
            ray.z);

        push_bytes(
            payload,
            ray.intensity);
    }


    // --------------------------------------------------------
    // Sanity check
    // --------------------------------------------------------

    if (payload.size() != expected_size)
    {
        std::cerr
            << "[ERROR] Payload size mismatch. Expected "
            << expected_size
            << " bytes, generated "
            << payload.size()
            << " bytes.\n";
    }

    return payload;
}


// ============================================================================
// PARSE CSV LINE
// ============================================================================
//
// Expected:
//
// time,rayId,X,Y,Z,intensity
//
// Example:
//
// 0.100426,0,7.09341,-0.30971,-0.31,0.013885
//
// ============================================================================

static bool parse_csv_line(
    const std::string& line,
    LidarRay& ray)
{
    if (line.empty())
        return false;


    std::stringstream stream(line);

    std::string field;


    try
    {
        // ----------------------------------------------------
        // time
        // ----------------------------------------------------

        if (!std::getline(stream, field, ','))
            return false;

        ray.timestamp =
            std::stod(field);


        // ----------------------------------------------------
        // rayId
        // ----------------------------------------------------

        if (!std::getline(stream, field, ','))
            return false;

        ray.ray_id =
            static_cast<uint32_t>(
                std::stoul(field));


        // ----------------------------------------------------
        // X
        // ----------------------------------------------------

        if (!std::getline(stream, field, ','))
            return false;

        ray.x =
            std::stof(field);


        // ----------------------------------------------------
        // Y
        // ----------------------------------------------------

        if (!std::getline(stream, field, ','))
            return false;

        ray.y =
            std::stof(field);


        // ----------------------------------------------------
        // Z
        // ----------------------------------------------------

        if (!std::getline(stream, field, ','))
            return false;

        ray.z =
            std::stof(field);


        // ----------------------------------------------------
        // intensity
        // ----------------------------------------------------

        if (!std::getline(stream, field, ','))
            return false;

        ray.intensity =
            std::stof(field);
    }
    catch (const std::exception&)
    {
        return false;
    }


    return true;
}


// ============================================================================
// READ CSV INTO FRAMES
// ============================================================================
//
// Every unique timestamp becomes one frame.
//
// Example:
//
// 0.100426
//   ray 0
//   ray 1
//   ...
//   ray 35
//
// becomes:
//
// Frame {
//     timestamp = 0.100426
//     rays = 36
// }
//
// ============================================================================

static bool load_csv(
    const std::string& filename,
    std::vector<LidarFrame>& frames)
{
    std::ifstream file(filename);

    if (!file.is_open())
    {
        std::cerr
            << "[ERROR] Could not open CSV:\n"
            << filename
            << "\n";

        return false;
    }


    std::cout
        << "[INFO] Reading CSV:\n"
        << filename
        << "\n";


    std::string line;


    // --------------------------------------------------------
    // Read header
    // --------------------------------------------------------

    if (!std::getline(file, line))
    {
        std::cerr
            << "[ERROR] CSV file is empty.\n";

        return false;
    }


    std::cout
        << "[INFO] CSV header: "
        << line
        << "\n";


    // --------------------------------------------------------
    // Read rows
    // --------------------------------------------------------

    LidarFrame current_frame;

    bool have_frame = false;


    std::size_t line_number = 1;


    while (std::getline(file, line))
    {
        ++line_number;


        if (line.empty())
            continue;


        LidarRay ray;


        if (!parse_csv_line(line, ray))
        {
            std::cerr
                << "[WARNING] Could not parse CSV line "
                << line_number
                << ". Skipping.\n";

            continue;
        }


        // ----------------------------------------------------
        // First frame
        // ----------------------------------------------------

        if (!have_frame)
        {
            current_frame.timestamp =
                ray.timestamp;

            current_frame.rays.clear();

            current_frame.rays.push_back(ray);

            have_frame = true;

            continue;
        }


        // ----------------------------------------------------
        // Same timestamp -> same frame
        // ----------------------------------------------------

        if (ray.timestamp == current_frame.timestamp)
        {
            current_frame.rays.push_back(ray);
        }

        // ----------------------------------------------------
        // New timestamp -> finish previous frame
        // ----------------------------------------------------

        else
        {
            frames.push_back(
                std::move(current_frame));


            current_frame =
                LidarFrame{};

            current_frame.timestamp =
                ray.timestamp;

            current_frame.rays.push_back(ray);
        }
    }


    // --------------------------------------------------------
    // Add final frame
    // --------------------------------------------------------

    if (have_frame &&
        !current_frame.rays.empty())
    {
        frames.push_back(
            std::move(current_frame));
    }


    file.close();


    // --------------------------------------------------------
    // Check result
    // --------------------------------------------------------

    if (frames.empty())
    {
        std::cerr
            << "[ERROR] No LiDAR frames found.\n";

        return false;
    }


    std::cout
        << "[INFO] Loaded "
        << frames.size()
        << " LiDAR frames.\n";


    // --------------------------------------------------------
    // Check each frame
    // --------------------------------------------------------

    for (std::size_t i = 0;
        i < frames.size();
        ++i)
    {
        if (frames[i].rays.size() !=
            EXPECTED_RAYS_PER_FRAME)
        {
            std::cerr
                << "[WARNING] Frame "
                << i
                << " timestamp "
                << std::fixed
                << std::setprecision(6)
                << frames[i].timestamp
                << " contains "
                << frames[i].rays.size()
                << " rays. Expected "
                << EXPECTED_RAYS_PER_FRAME
                << ".\n";
        }
    }


    return true;
}


// ============================================================================
// PRINT PAYLOAD HEX
// ============================================================================
//
// Useful for debugging.
//
// Only prints the first max_bytes bytes.
// ============================================================================

static void print_payload_hex(
    const std::vector<uint8_t>& payload,
    std::size_t max_bytes = 64)
{
    const std::size_t count =
        std::min(
            payload.size(),
            max_bytes);


    std::cout
        << "[PAYLOAD] "
        << payload.size()
        << " bytes\n";


    std::cout
        << "[PAYLOAD HEX] ";


    for (std::size_t i = 0;
        i < count;
        ++i)
    {
        std::cout
            << std::hex
            << std::setw(2)
            << std::setfill('0')
            << static_cast<unsigned int>(
                payload[i])
            << " ";
    }


    if (payload.size() > count)
    {
        std::cout
            << "...";
    }


    std::cout
        << std::dec
        << "\n";
}


// ============================================================================
// PUBLISH ONE FRAME
// ============================================================================

static void publish_frame(
    const std::shared_ptr<vsomeip::application>& app,
    const LidarFrame& frame,
    bool print_hex)
{
    // --------------------------------------------------------
    // Build application payload
    // --------------------------------------------------------

    const std::vector<uint8_t> payload =
        build_payload(frame);


    // --------------------------------------------------------
    // Create SOME/IP payload
    // --------------------------------------------------------

    std::shared_ptr<vsomeip::payload>
        someip_payload =
        vsomeip::runtime::get()
        ->create_payload();


    someip_payload->set_data(
        payload);


    // --------------------------------------------------------
    // Send SOME/IP event
    // --------------------------------------------------------

    app->notify(
        LIDAR_SERVICE,
        LIDAR_INSTANCE,
        LIDAR_EVENT,
        someip_payload);


    // --------------------------------------------------------
    // Console output
    // --------------------------------------------------------

    std::cout
        << "[TX] timestamp="
        << std::fixed
        << std::setprecision(6)
        << frame.timestamp
        << " | rays="
        << frame.rays.size()
        << " | payload="
        << payload.size()
        << " bytes\n";


    if (print_hex)
    {
        print_payload_hex(
            payload,
            64);
    }
}


// ============================================================================
// MAIN
// ============================================================================

int main()
{
    std::cout
        << "============================================================\n"
        << "        CSV -> SOME/IP LiDAR Publisher\n"
        << "============================================================\n\n";


    std::cout
        << "Service    : 0x"
        << std::hex
        << LIDAR_SERVICE
        << "\n";

    std::cout
        << "Instance   : 0x"
        << LIDAR_INSTANCE
        << "\n";

    std::cout
        << "Event      : 0x"
        << LIDAR_EVENT
        << "\n";

    std::cout
        << "Eventgroup : 0x"
        << LIDAR_EVENTGROUP
        << "\n"
        << std::dec;


    std::cout
        << "Expected rays/frame : "
        << EXPECTED_RAYS_PER_FRAME
        << "\n";

    std::cout
        << "Expected payload    : "
        << HEADER_BYTES +
        EXPECTED_RAYS_PER_FRAME * RAY_BYTES
        << " bytes\n\n";


    // ========================================================================
    // Load CSV
    // ========================================================================

    std::vector<LidarFrame> frames;


    if (!load_csv(
        CSV_FILE,
        frames))
    {
        return 1;
    }


    // ========================================================================
    // Create vSomeIP application
    // ========================================================================

    auto app =
        vsomeip::runtime::get()
        ->create_application(
            "csv_lidar_publisher");


    if (!app)
    {
        std::cerr
            << "[ERROR] Failed to create "
            "vSomeIP application.\n";

        return 1;
    }


    // ========================================================================
    // Initialise vSomeIP
    // ========================================================================

    if (!app->init())
    {
        std::cerr
            << "[ERROR] vSomeIP initialization failed.\n";

        return 1;
    }


    std::cout
        << "[INFO] vSomeIP initialized.\n";


    // ========================================================================
    // Offer SOME/IP service
    // ========================================================================

    app->offer_service(
        LIDAR_SERVICE,
        LIDAR_INSTANCE);


    // ========================================================================
    // Offer LiDAR event
    // ========================================================================

    std::set<vsomeip::eventgroup_t> groups;

    groups.insert(
        LIDAR_EVENTGROUP);


    app->offer_event(
        LIDAR_SERVICE,
        LIDAR_INSTANCE,
        LIDAR_EVENT,
        groups,
        vsomeip::event_type_e::ET_FIELD);


    std::cout
        << "[INFO] SOME/IP service offered.\n";


    // ========================================================================
    // Start vSomeIP event loop
    // ========================================================================

    std::thread someip_thread(
        [&app]()
        {
            app->start();
        });


    // Give vSomeIP a short amount of time to initialise its
    // routing/service-discovery processing before transmitting.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(500));


    // ========================================================================
    // Replay frames
    // ========================================================================

    std::cout
        << "\n[INFO] Starting LiDAR replay...\n\n";


    double previous_timestamp =
        frames.front().timestamp;


    for (std::size_t i = 0;
        i < frames.size();
        ++i)
    {
        const LidarFrame& frame =
            frames[i];


        // ----------------------------------------------------
        // Reproduce CSV timing
        // ----------------------------------------------------

        if (REPLAY_TIMING && i > 0)
        {
            const double delta_seconds =
                frame.timestamp -
                previous_timestamp;


            if (delta_seconds > 0.0)
            {
                const auto delay =
                    std::chrono::duration<double>(
                        delta_seconds);


                std::this_thread::sleep_for(
                    delay);
            }
        }


        // ----------------------------------------------------
        // Publish
        // ----------------------------------------------------

        publish_frame(
            app,
            frame,
            i == 0);


        previous_timestamp =
            frame.timestamp;
    }


    // ========================================================================
    // Finished
    // ========================================================================

    std::cout
        << "\n============================================================\n"
        << "CSV replay completed.\n"
        << "============================================================\n";


    // Give the final SOME/IP notification a moment to leave.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(200));


    // ========================================================================
    // Stop
    // ========================================================================

    app->stop();


    if (someip_thread.joinable())
    {
        someip_thread.join();
    }


    std::cout
        << "[INFO] Publisher stopped.\n";


    return 0;
}
