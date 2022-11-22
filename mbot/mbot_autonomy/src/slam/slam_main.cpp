#include <slam/slam.hpp>
#include <common_utils/getopt.h>
#include <lcm/lcm-cpp.hpp>
#include <mbot_lcm_msgs/mbot_system_reset_t.hpp>
#include <mbot/mbot_channels.h>
#include <thread>
#include <fstream>
#include <csignal>

using UniqueSlamPtr = std::unique_ptr<OccupancyGridSLAM>;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

using SlamMode = OccupancyGridSLAM::Mode;
class SystemResetHandler
{
public:
    bool reset_requested = false;

    SystemResetHandler(int numParticles,
                       int hitOdds,
                       int missOdds,
                       lcm::LCM& lcmConnection,
                       bool useOptitrack,
                       bool mappingOnly,
                       bool localizationOnly,
                       bool actionOnly,
                       std::string& mapFile,
                       bool randomInitialPos)
        : numParticles_(numParticles)
        , hitOdds_(hitOdds)
        , missOdds_(missOdds)
        , useOptitrack_(useOptitrack)
        , mappingOnly_(mappingOnly)
        , localizationOnly_(localizationOnly)
        , actionOnly_(actionOnly)
        , mapFile_(mapFile)
        , randomInitialPos_(randomInitialPos)
        , retainPose_(false)
    {
        lcmConnection.subscribe(MBOT_SYSTEM_RESET_CHANNEL, &SystemResetHandler::handle_system_reset, this);
    }

    void handle_system_reset(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const mbot_lcm_msgs::mbot_system_reset_t* reset_msg)
    {
        auto reset_mode = static_cast<SlamMode>(reset_msg->slam_mode);
        mappingOnly_ = reset_mode == SlamMode::mapping_only;
        localizationOnly_ = reset_mode == SlamMode::localization_only;
        actionOnly_ = reset_mode == SlamMode::action_only;

        retainPose_ = reset_msg->retain_pose;

        // Check if file exists.
        std::ifstream f(reset_msg->slam_map_location.c_str());
        if (f.good()) mapFile_ = reset_msg->slam_map_location;
        reset_requested = true;

        std::cout << "INFO: Slam reset requested with mode " << reset_mode << ". ";
        std::cout << "Saving to file: " << mapFile_ << std::endl;
    }

    UniqueSlamPtr get_reset_slam_ptr(lcm::LCM& lcmConnection, const UniqueSlamPtr& slam)
    {
        if (retainPose_)
        {
            std::cout << "[SLAM] Resetting SLAM. Retaining pose." << std::endl;
            auto pose = slam->getCurrentPose();
            return UniqueSlamPtr(
                new OccupancyGridSLAM(
                    numParticles_, hitOdds_, missOdds_, lcmConnection, useOptitrack_,
                    mappingOnly_, localizationOnly_, actionOnly_, mapFile_, false, pose)
            );
        }

        std::cout << "[SLAM] Resetting SLAM." << std::endl;

        return get_reset_slam_ptr(lcmConnection);
    }

    UniqueSlamPtr get_reset_slam_ptr(lcm::LCM& lcmConnection)
    {
        return UniqueSlamPtr(
            new OccupancyGridSLAM(
                numParticles_, hitOdds_, missOdds_, lcmConnection, useOptitrack_,
                mappingOnly_, localizationOnly_, actionOnly_, mapFile_, randomInitialPos_)
        );
    }

    void reset_complete()
    {
        reset_requested = false;
    }

private:
    int numParticles_;
    int hitOdds_;
    int missOdds_;
    bool useOptitrack_;
    bool mappingOnly_;
    bool localizationOnly_;
    bool actionOnly_;
    std::string mapFile_;
    bool randomInitialPos_;
    bool retainPose_;
};

int main(int argc, char** argv)
{
    const char* kNumParticlesArg = "num-particles";
    const char* kHitOddsArg = "hit-odds";
    const char* kMissOddsArg = "miss-odds";
    const char* kUseOptitrackArg = "use-optitrack";
    const char* kMappingOnlyArg = "mapping-only";
    const char* kActionOnlyArg = "action-only";
    const char* kLocalizationOnlyArg = "localization-only";
    const char* kRandomParticleInitialization = "random-initial-pos";
    const char* kListeningMode = "listen-for-mode";
    const char* kMapFile = "map";

    // Handle Options
    getopt_t *gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Show this help");
    getopt_add_int(gopt, '\0', kNumParticlesArg, "1000", "Number of particles to use in the particle filter");
    getopt_add_int(gopt, '\0', kHitOddsArg, "4", "Amount to increase log-odds when a cell is hit by a laser ray");
    getopt_add_int(gopt, '\0', kMissOddsArg, "2", "Amount to decrease log-odds when a cell is passed through by a laser ray");
    getopt_add_bool(gopt, '\0', kUseOptitrackArg, 0, "Flag indicating if the map reference frame should be set to the Optitrack reference frame.");
    getopt_add_bool(gopt, '\0', kMappingOnlyArg, 0, "Flag indicating if mapping-only mode should be run");
    getopt_add_bool(gopt, '\0', kActionOnlyArg, 0, "Flag indicating if action-only mode should be run");
    getopt_add_bool(gopt, '\0', kLocalizationOnlyArg, 0, "Localization only mode should be run.");
    getopt_add_bool(gopt, '\0', kListeningMode, 0, "Given this flag, the system will listen for an lcm mode message.");
    getopt_add_string(gopt, '\0', kMapFile, "current.map", "Map to load if localization only, output map file if mapping mode.");

    getopt_add_bool(gopt, '\0', kRandomParticleInitialization, 0, "Initial particles should be randomly distributed along the map.");

    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }

    int numParticles = getopt_get_int(gopt, kNumParticlesArg);
    int hitOdds = getopt_get_int(gopt, kHitOddsArg);
    int missOdds = getopt_get_int(gopt, kMissOddsArg);
    bool useOptitrack = getopt_get_bool(gopt, kUseOptitrackArg);
    bool mappingOnly = getopt_get_bool(gopt, kMappingOnlyArg);
    bool actionOnly = getopt_get_bool(gopt, kActionOnlyArg);
    bool localizationOnly = getopt_get_bool(gopt, kLocalizationOnlyArg);
    bool randomInitialPos = getopt_get_bool(gopt, kRandomParticleInitialization);
    bool listeningMode = getopt_get_bool(gopt, kListeningMode);
    std::string mapFile = getopt_get_string(gopt, kMapFile);

    ctrl_c_pressed = false;
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);
    lcm::LCM lcmConnection(MULTICAST_URL);
    SystemResetHandler systemResetHandler(numParticles, hitOdds, missOdds, lcmConnection, useOptitrack,
                                          mappingOnly, localizationOnly, actionOnly, mapFile, randomInitialPos);

    UniqueSlamPtr slam;
    std::shared_ptr<std::thread> slamThreadPtr;

    if(listeningMode)
    {
        std::cout << "SLAM started in IDLE mode, waiting for reset..." << std::endl;
        while(!systemResetHandler.reset_requested)
        {
            if (ctrl_c_pressed)
            {
                std::cout << "Quitting SLAM in IDLE mode." << std::endl;
                return 0;
            }
            lcmConnection.handleTimeout(1000);  // Timeout so we don't get stuck here on termination.
        }
        slam = systemResetHandler.get_reset_slam_ptr(lcmConnection);
        slamThreadPtr = std::unique_ptr<std::thread>(new std::thread([&slam]() { slam->runSLAM(); }));
        systemResetHandler.reset_complete();
    }
    else
    {
        slam = UniqueSlamPtr(
                    new OccupancyGridSLAM(
                        numParticles, hitOdds, missOdds, lcmConnection, useOptitrack,
                        mappingOnly, localizationOnly, actionOnly, mapFile, randomInitialPos)
                );
        slamThreadPtr = std::unique_ptr<std::thread >(new std::thread([&slam]() { slam->runSLAM(); }));
    }

    std::cout << "Starting SLAM..." << std::endl;
    while(true)
    {
        lcmConnection.handle();

        if (ctrl_c_pressed)
        {
            slam->stopSLAM();
            break;
        }
        else if (systemResetHandler.reset_requested)
        {
            slam->stopSLAM();
            slamThreadPtr->join();
            slam = systemResetHandler.get_reset_slam_ptr(lcmConnection, slam);
            slamThreadPtr = std::unique_ptr<std::thread>(new std::thread([&slam]() { slam->runSLAM(); }));
            systemResetHandler.reset_complete();
        }
    }

    slamThreadPtr->join();

    return 0;
}
