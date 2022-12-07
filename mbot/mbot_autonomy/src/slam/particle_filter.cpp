#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>


ParticleFilter::ParticleFilter(int numParticles, bool randomInitialPos)
: kNumParticles_ (numParticles)
, randomInitialPos_(randomInitialPos)
, samplingAugmentation(0.5, 0.9, numParticles)
, randomPoseGen()
, distribution_quality(1)
, quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ////////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    // Initialize the particles in the particle filter
    double sampleWeight = 1.0 / kNumParticles_;
    std::random_device rd;
    std::mt19937 generator(rd());
    std::normal_distribution<float> dist(0.0, 0.01);

    posteriorPose_ = pose;

    for (auto &&p : posterior_)
    {
        p.pose = pose;
        p.pose.x += dist(generator);
        p.pose.y += dist(generator);
        p.pose.theta += dist(generator);
        p.pose.utime = pose.utime;

        p.parent_pose = p.pose;

        p.weight = sampleWeight;
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ////////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    randomPoseGen.update_map(&map);
    double sampleWeight = 1.0 / kNumParticles_;
    
    posteriorPose_ = randomPoseGen.get_pose();
    
    for (auto &&p : posterior_)
    {
        p = randomPoseGen.get_particle();
        p.weight = sampleWeight;
    }
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if (hasRobotMoved)
    {
        // If the robot has moved, the new prior is the previous posterior
        // resample using weights
        auto prior = resamplePosteriorDistribution(&map);

        // Compute the proposal distribution using the motion model
        // apply action model
        auto proposal = computeProposalDistribution(prior);

        // Compute the posterior by correcting with the laser and normalizing particle weights
        // apply sensor model
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        // OPTIONAL TODO: Add reinvigoration step

        // Estimate the pose using the weights of each particle
        // weighted average
        posteriorPose_ = estimatePosteriorPose(posterior_);
    }
    posteriorPose_.utime = odometry.utime;
    
    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        // If the robot has moved, the new prior is the previous posterior
        auto prior = resamplePosteriorDistribution();
        
        // Compute the proposal distribution using the motion model
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    // Low variance sampler from Probabilistic Robotics
    ParticleList prior;
    if (randomInitialPos_)
        randomPoseGen.update_map(map);

    // random number
    std::random_device rd;
    std::default_random_engine generator(rd());
    auto uniform_distribution = std::uniform_real_distribution<double>(0.0, 1.0 / (static_cast<double>(kNumParticles_)));
    double r = uniform_distribution(generator);
    double c = posterior_[0].weight;
    int i = 0;
    double U;
    for (int m = 0; m < kNumParticles_; m++){
        if (randomInitialPos_ && samplingAugmentation.sample_randomly()){
            prior.push_back(randomPoseGen.get_particle());
        } else {
            U = r + m * 1.0 / (static_cast<double>(kNumParticles_));
            while (U > c){
                i += 1;
                c += posterior_[i].weight;
            }
            prior.push_back(posterior_[i]);
        }
    }

    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    // Create the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
     // Take the proposal, and apply the actions
    for (auto &&p : prior)
    {
        proposal.push_back(actionModel_.applyAction(p));
    }
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    //////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the particles in the proposal distribution
    ParticleList posterior;

    double sumWeights = 0.0;
    double averageWeights = 0.0;
    double M = 1.0 / static_cast<double>(kNumParticles_);

    for (auto &&p : proposal)
    {
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight = sensorModel_.likelihood(weighted, laser, map);
        sumWeights += weighted.weight;
        averageWeights += weighted.weight / M;
        posterior.push_back(weighted);
    }

    // Renormalize
    for (auto &&p : posterior)
    {
        p.weight /= sumWeights;
    }
    samplingAugmentation.insert_average_weight(averageWeights);
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;

    ParticleList posterior_copy(posterior.begin(), posterior.end());
    std::sort(posterior_copy.begin(), posterior_copy.end(), [ ]( const mbot_lcm_msgs::particle_t& lhs, const mbot_lcm_msgs::particle_t& rhs )
    {
        return lhs.weight > rhs.weight;
    });
    int best_pct_idx = posterior_copy.size() / 10 - 1;
    ParticleList posterior_best(posterior_copy.begin(), posterior_copy.begin()+best_pct_idx);

    pose = computeParticlesAverage(posterior_best);
    // pose = computeParticlesAverage(posterior);
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &&p : particles_to_average)
    {
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
