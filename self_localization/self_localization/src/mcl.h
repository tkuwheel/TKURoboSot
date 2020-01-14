#ifndef MCL_H
#define MCL_H

#include <vector>
#include <random>
#include <tuple>
#include <mutex>

class MCL
{
public:
    typedef std::tuple<double,double,double,double,double,double> Particle;
    typedef std::vector<Particle> Particles;
    typedef std::pair<double,double> SensorData;
    typedef std::tuple<double,double,double> State;
    struct FieldMatrix;

    MCL();
    void updateMotion(double vx, double vy, double dw);
    void updateSensor(const std::vector<SensorData> &data);
    void updateCompass(double compass);
    double getCompass() { return cmps; }
    double get_a_fast() { return a_fast; }
    double get_a_slow() { return a_slow; }
    double get_w_vis() { return wvision; }
    double get_w_cmps() { return wcmps; }
    double get_sd() { return sd; }
    void setAugmentParam(double a_fast, double a_slow);
    void setCmpsWeight(double w);
    Particles getParticles()
    {
        mutex.lock();
        auto ret = particles;
        mutex.unlock();
        return ret;
    }
    State estimation();
    State weighted_estimation();
    void resetParticles(bool init, double xpos, double ypos, double wpos);
    void setRandomParameter(double xv, double yv, double wv);
    FieldMatrix* getField() { return &field; }

    static inline double& x(MCL::Particle& particle)
    {
        return std::get<0>(particle);
    }

    static inline double& y(MCL::Particle& particle)
    {
        return std::get<1>(particle);
    }

    static inline double& w(MCL::Particle& particle)
    {
        return std::get<2>(particle);
    }

    static inline double& vis_weight(MCL::Particle& particle)
    {
        return std::get<3>(particle);
    }

    static inline double& cmps_weight(MCL::Particle& particle)
    {
        return std::get<4>(particle);
    }

    static inline double& total_weight(MCL::Particle& particle)
    {
        return std::get<5>(particle);
    }

    static inline double& x(MCL::SensorData& sensor)
    {
        return sensor.first;
    }

    static inline double& y(MCL::SensorData& sensor)
    {
        return sensor.second;
    }

public:
    struct FieldMatrix
    {
        FieldMatrix();
        std::vector<int> xline;
        std::vector<int> yline;
        int start_x;
        int start_y;
        int end_x;
        int end_y;
        int x_length;
        int y_length;
        double *distance_matrix;
        double distance(double x, double y);
    };

private:
    void resample();
    double cmps_error(double& angle);

private:
    Particles particles;
    FieldMatrix field;
    std::mutex mutex;
    Particle pose_estimation;
    double xvar;
    double yvar;
    double wvar;
    double cmps;
    double w_fast;
    double w_slow;
    double a_fast;
    double a_slow;
    double wcmps;
    double wvision;
    double sd;
};


#endif // MCL_H
