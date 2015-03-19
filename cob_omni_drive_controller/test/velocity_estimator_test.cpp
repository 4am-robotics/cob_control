#include <cob_omni_drive_controller/VelocityEstimator.h>
#include <gtest/gtest.h>
#include "boost/random.hpp"

class Randomizer : public boost::variate_generator<boost::mt19937, boost::uniform_real<double> >{
public:
    Randomizer() : boost::variate_generator<boost::mt19937, boost::uniform_real<double> >(boost::mt19937(std::time(0)), boost::uniform_real<double>(std::numeric_limits<double>::min(),std::numeric_limits<double>::max())) {}
    Randomizer(double limit) : boost::variate_generator<boost::mt19937, boost::uniform_real<double> >(boost::mt19937(std::time(0)), boost::uniform_real<double>(-limit, limit)) {}
    Randomizer(double min, double max) : boost::variate_generator<boost::mt19937, boost::uniform_real<double> >(boost::mt19937(std::time(0)), boost::uniform_real<double>(min,max)) {}
};

TEST(VelocityEstimatorTest, ZeroOverflowTest)
{
    VelocityEstimator est(0.0);

    Randomizer rnd;

    for(int i=0; i < 100; ++i){
        double vel = rnd();
        EXPECT_DOUBLE_EQ(vel, est.estimateVelocity(rnd(), vel, 1.0));
    }

}

TEST(VelocityEstimatorTest, InfiniteOverflowTest)
{
    double half_max = std::numeric_limits<double>::max()/2.0;
    Randomizer rnd;

    {
        VelocityEstimator est(-1);
        ASSERT_DOUBLE_EQ(0.0, est.estimateVelocity(0,0,1.0));
        ASSERT_DOUBLE_EQ(-half_max,est.estimateVelocity(-half_max ,rnd(),1.0));
        ASSERT_EQ(std::numeric_limits<double>::max(), est.estimateVelocity(half_max,rnd(),1.0));
    }
    {
        VelocityEstimator est(half_max);
        ASSERT_DOUBLE_EQ(0.0, est.estimateVelocity(0,0,1.0));
        ASSERT_DOUBLE_EQ(-half_max,est.estimateVelocity(-half_max ,rnd(),1.0));
        ASSERT_EQ(0, est.estimateVelocity(half_max,rnd(),1.0));  // zeor means overflow
    }
}

TEST(VelocityEstimatorTest, TestOverflowPos)
{
    Randomizer rnd;
    Randomizer time_rnd(0,10.0);

    for(int n=1; n < 100; ++n)
    {
        double overflow = Randomizer(0, 1000)();
        Randomizer half(1e-6,overflow/2.0);

        VelocityEstimator est(overflow);

        double start = Randomizer(overflow)();
        ASSERT_DOUBLE_EQ(0.0, est.estimateVelocity(start,0,1.0));

        double p = start, step, dt;
        while(1){
            step = half();
            dt = time_rnd();
            if(step == 0.0) continue;
            if(p + step > overflow) break;
            p += step;
            EXPECT_NEAR(step/dt, est.estimateVelocity(p, rnd(), dt), 1e-6);
        }
        p -= 2*overflow;
        while(1){
            step = half();
            dt = time_rnd();
            if(step == 0.0) continue;
            p += step;
            if(p + step > start) break;
            EXPECT_NEAR(step/dt, est.estimateVelocity(p, rnd(), dt), 1e-6);
        }

    }
}

TEST(VelocityEstimatorTest, TestOverflowNeg)
{
    Randomizer rnd;
    Randomizer time_rnd(0,10.0);

    for(int n=1; n < 100; ++n)
    {
        double overflow = Randomizer(0, 1000)();
        Randomizer half(-overflow/2.0,0.0);

        VelocityEstimator est(overflow);

        double start = Randomizer(overflow)();
        ASSERT_DOUBLE_EQ(0.0, est.estimateVelocity(start,0,1.0));

        double p = start, step, dt;
        while(1){
            step = half();
            dt = time_rnd();
            if(step == 0.0) continue;
            if(p + step < -overflow) break;
            p += step;
            EXPECT_NEAR(step/dt, est.estimateVelocity(p, rnd(), dt), 1e-6);
        }
        p += 2*overflow;
        while(1){
            step = half();
            dt = time_rnd();
            if(step == 0.0) continue;
            p += step;
            if(p + step < start) break;
            EXPECT_NEAR(step/dt, est.estimateVelocity(p, rnd(), dt), 1e-6);
        }

    }

}


TEST(VelocityEstimatorTest, TestNoMove)
{
    VelocityEstimator est(10.0);
    Randomizer rnd;

    ASSERT_DOUBLE_EQ(0.0, est.estimateVelocity(0,0,0));

    for(int j=0; j < 100; ++j){
        EXPECT_DOUBLE_EQ(0.0, est.estimateVelocity(0, rnd(), 1.0));
    }

}

TEST(VelocityEstimatorTest, TestReset)
{
    VelocityEstimator est(10.0);
    Randomizer pos_rnd(10.0);
    Randomizer rnd;

    for(int i=0; i < 100; ++i){
        double vel = rnd();
        EXPECT_DOUBLE_EQ(vel, est.estimateVelocity(pos_rnd(), vel, 1.0));
        est.reset();
    }

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
