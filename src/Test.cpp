#include "PathFollower.hpp"
#include <cstdio>
#include <Eigen/Geometry>
#include <iostream>

Path getPath()
{
    std::vector<Eigen::Isometry2d> cp;
    cp.reserve(112);
    cp.emplace_back(Eigen::Translation2d(0.343997, 0.942623) * Eigen::Rotation2Dd(-1.43288));
    cp.emplace_back(Eigen::Translation2d(0.352861, 0.947252) * Eigen::Rotation2Dd(-1.43288));
    cp.emplace_back(Eigen::Translation2d(0.361725, 0.951881) * Eigen::Rotation2Dd(-1.43219));
    cp.emplace_back(Eigen::Translation2d(0.370779, 0.956123) * Eigen::Rotation2Dd(-1.4294));
    cp.emplace_back(Eigen::Translation2d(0.379881, 0.960266) * Eigen::Rotation2Dd(-1.42325));
    cp.emplace_back(Eigen::Translation2d(0.388982, 0.964409) * Eigen::Rotation2Dd(-1.41311));
    cp.emplace_back(Eigen::Translation2d(0.398084, 0.968552) * Eigen::Rotation2Dd(-1.39907));
    cp.emplace_back(Eigen::Translation2d(0.407185, 0.972695) * Eigen::Rotation2Dd(-1.38148));
    cp.emplace_back(Eigen::Translation2d(0.416287, 0.976837) * Eigen::Rotation2Dd(-1.36064));
    cp.emplace_back(Eigen::Translation2d(0.425388, 0.98098) * Eigen::Rotation2Dd(-1.3368));
    cp.emplace_back(Eigen::Translation2d(0.43449, 0.985123) * Eigen::Rotation2Dd(-1.31016));
    cp.emplace_back(Eigen::Translation2d(0.443591, 0.989266) * Eigen::Rotation2Dd(-1.28088));
    cp.emplace_back(Eigen::Translation2d(0.452693, 0.993409) * Eigen::Rotation2Dd(-1.24911));
    cp.emplace_back(Eigen::Translation2d(0.461794, 0.997552) * Eigen::Rotation2Dd(-1.21497));
    cp.emplace_back(Eigen::Translation2d(0.470896, 1.00169) * Eigen::Rotation2Dd(-1.17814));
    cp.emplace_back(Eigen::Translation2d(0.479997, 1.00584) * Eigen::Rotation2Dd(-1.13809));
    cp.emplace_back(Eigen::Translation2d(0.489098, 1.00998) * Eigen::Rotation2Dd(-1.09473));
    cp.emplace_back(Eigen::Translation2d(0.4982, 1.01412) * Eigen::Rotation2Dd(-1.04807));
    cp.emplace_back(Eigen::Translation2d(0.507301, 1.01827) * Eigen::Rotation2Dd(-0.998112));
    cp.emplace_back(Eigen::Translation2d(0.516403, 1.02241) * Eigen::Rotation2Dd(-0.944848));
    cp.emplace_back(Eigen::Translation2d(0.525504, 1.02655) * Eigen::Rotation2Dd(-0.88828));
    cp.emplace_back(Eigen::Translation2d(0.534606, 1.03069) * Eigen::Rotation2Dd(-0.828408));
    cp.emplace_back(Eigen::Translation2d(0.543707, 1.03484) * Eigen::Rotation2Dd(-0.765233));
    cp.emplace_back(Eigen::Translation2d(0.552809, 1.03898) * Eigen::Rotation2Dd(-0.698755));
    cp.emplace_back(Eigen::Translation2d(0.56191, 1.04312) * Eigen::Rotation2Dd(-0.628974));
    cp.emplace_back(Eigen::Translation2d(0.571012, 1.04727) * Eigen::Rotation2Dd(-0.555889));
    cp.emplace_back(Eigen::Translation2d(0.580113, 1.05141) * Eigen::Rotation2Dd(-0.479501));
    cp.emplace_back(Eigen::Translation2d(0.589215, 1.05555) * Eigen::Rotation2Dd(-0.399809));
    cp.emplace_back(Eigen::Translation2d(0.598316, 1.05969) * Eigen::Rotation2Dd(-0.316814));
    cp.emplace_back(Eigen::Translation2d(0.607418, 1.06384) * Eigen::Rotation2Dd(-0.230516));
    cp.emplace_back(Eigen::Translation2d(0.616519, 1.06798) * Eigen::Rotation2Dd(-0.140914));
    cp.emplace_back(Eigen::Translation2d(0.625621, 1.07212) * Eigen::Rotation2Dd(-0.0480095));
    cp.emplace_back(Eigen::Translation2d(0.634722, 1.07627) * Eigen::Rotation2Dd(0.0480472));
    cp.emplace_back(Eigen::Translation2d(0.643824, 1.08041) * Eigen::Rotation2Dd(0.143952));
    cp.emplace_back(Eigen::Translation2d(0.652925, 1.08455) * Eigen::Rotation2Dd(0.236554));
    cp.emplace_back(Eigen::Translation2d(0.662027, 1.08869) * Eigen::Rotation2Dd(0.325852));
    cp.emplace_back(Eigen::Translation2d(0.671128, 1.09284) * Eigen::Rotation2Dd(0.411847));
    cp.emplace_back(Eigen::Translation2d(0.68023, 1.09698) * Eigen::Rotation2Dd(0.494539));
    cp.emplace_back(Eigen::Translation2d(0.689331, 1.10112) * Eigen::Rotation2Dd(0.573927));
    cp.emplace_back(Eigen::Translation2d(0.698433, 1.10527) * Eigen::Rotation2Dd(0.650012));
    cp.emplace_back(Eigen::Translation2d(0.707534, 1.10941) * Eigen::Rotation2Dd(0.722794));
    cp.emplace_back(Eigen::Translation2d(0.716635, 1.11355) * Eigen::Rotation2Dd(0.792272));
    cp.emplace_back(Eigen::Translation2d(0.725737, 1.11769) * Eigen::Rotation2Dd(0.858447));
    cp.emplace_back(Eigen::Translation2d(0.734838, 1.12184) * Eigen::Rotation2Dd(0.921319));
    cp.emplace_back(Eigen::Translation2d(0.74394, 1.12598) * Eigen::Rotation2Dd(0.980887));
    cp.emplace_back(Eigen::Translation2d(0.753041, 1.13012) * Eigen::Rotation2Dd(1.03715));
    cp.emplace_back(Eigen::Translation2d(0.762143, 1.13427) * Eigen::Rotation2Dd(1.09011));
    cp.emplace_back(Eigen::Translation2d(0.771244, 1.13841) * Eigen::Rotation2Dd(1.13977));
    cp.emplace_back(Eigen::Translation2d(0.780346, 1.14255) * Eigen::Rotation2Dd(1.18676));
    cp.emplace_back(Eigen::Translation2d(0.789447, 1.14669) * Eigen::Rotation2Dd(1.23172));
    cp.emplace_back(Eigen::Translation2d(0.798549, 1.15084) * Eigen::Rotation2Dd(1.27467));
    cp.emplace_back(Eigen::Translation2d(0.80765, 1.15498) * Eigen::Rotation2Dd(1.3155));
    cp.emplace_back(Eigen::Translation2d(0.816752, 1.15912) * Eigen::Rotation2Dd(1.35412));
    cp.emplace_back(Eigen::Translation2d(0.825853, 1.16326) * Eigen::Rotation2Dd(1.39039));
    cp.emplace_back(Eigen::Translation2d(0.834955, 1.16741) * Eigen::Rotation2Dd(1.42419));
    cp.emplace_back(Eigen::Translation2d(0.844056, 1.17155) * Eigen::Rotation2Dd(1.45533));
    cp.emplace_back(Eigen::Translation2d(0.853158, 1.17569) * Eigen::Rotation2Dd(1.48361));
    cp.emplace_back(Eigen::Translation2d(0.862259, 1.17984) * Eigen::Rotation2Dd(1.50878));
    cp.emplace_back(Eigen::Translation2d(0.871361, 1.18398) * Eigen::Rotation2Dd(1.5305));
    cp.emplace_back(Eigen::Translation2d(0.880507, 1.18802) * Eigen::Rotation2Dd(1.54819));
    cp.emplace_back(Eigen::Translation2d(0.889969, 1.19124) * Eigen::Rotation2Dd(1.56054));
    cp.emplace_back(Eigen::Translation2d(0.89971, 1.19349) * Eigen::Rotation2Dd(1.5674));
    cp.emplace_back(Eigen::Translation2d(0.909628, 1.19473) * Eigen::Rotation2Dd(1.5702));
    cp.emplace_back(Eigen::Translation2d(0.919621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.929621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.939621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.949621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.959621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.969621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.979621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.989621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(0.999621, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.00962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.01962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.02962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.03962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.04962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.05962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.06962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.07962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.08962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.09962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.10962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.11962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.12962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.13962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.14962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.15962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.16962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.17962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.18962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.19962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.20962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.21962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.22962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.23962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.24962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.25962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.26962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.27962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.28962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.29962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.30962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.31962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.32962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.33962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.34962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.35962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.36962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.37962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.38962, 1.195) * Eigen::Rotation2Dd(1.5708));
    cp.emplace_back(Eigen::Translation2d(1.394, 1.195) * Eigen::Rotation2Dd(1.5708));

    return Path{.poses = std::move(cp)};
};

int main()
{
    Path path(getPath());

    PathFollower pf(path, 1.1, {0.6, 0.4});

    size_t maxIterations = 10000;

    std::chrono::time_point<std::chrono::system_clock> curTime = std::chrono::system_clock::now();

    Eigen::Isometry2d curPose(path.poses.front());

    Eigen::Isometry2d world2End(path.poses.back().inverse());

    const double accuracy = 1e-4;

    auto computePoseInDt = [](std::chrono::milliseconds dt, const Eigen::Isometry2d& curPose,
                              const Twist2d& cmd) -> Eigen::Isometry2d {
        Eigen::Isometry2d ret(curPose);

        double dTsec = std::chrono::duration_cast<std::chrono::duration<double>>(dt).count();

        // compute next position from the cmd assuming perfect execution
        // note, this is correct, as the cmd is in global frame
        ret.translation() += cmd.tv * dTsec;
        ret.rotate(cmd.rv * dTsec);

        return ret;
    };

    Twist2d curCmd{.rv = 0, .tv = Eigen::Vector2d::Zero()};

    for (size_t i = 0; i < maxIterations; i++)
    {
        curTime += std::chrono::milliseconds(5);
        curPose = computePoseInDt(std::chrono::milliseconds(5), curPose, curCmd);
        pf.update(curPose, curTime);

        curTime += std::chrono::milliseconds(5);
        curPose = computePoseInDt(std::chrono::milliseconds(5), curPose, curCmd);
        curCmd = pf.computeNextCmd(curTime);

        const Eigen::Isometry2d diffToEnd(world2End * curPose);

        if ((diffToEnd.translation().squaredNorm() < accuracy * accuracy) &&
            (Eigen::Rotation2Dd(diffToEnd.rotation()).angle() < accuracy))
        {
            std::cout << "Reached end of path" << std::endl;
            break;
        }
    }

    return EXIT_SUCCESS;
}
