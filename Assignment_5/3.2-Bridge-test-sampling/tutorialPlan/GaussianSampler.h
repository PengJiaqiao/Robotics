#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/UniformSampler.h>
#include <random>

namespace rl
{
    namespace plan
    {
        /**
         * Uniform random sampling strategy and gaussian
         */
        class YourSampler : public UniformSampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generateCollision();

            virtual void seed(const ::std::mt19937::result_type& value);

	    ::rl::math::Vector* sigma;

        protected:
            //::std::uniform_real_distribution< ::rl::math::Real>::result_type rand();

            //::std::uniform_real_distribution< ::rl::math::Real> randDistribution;

	    ::std::normal_distribution< ::rl::math::Real>::result_type gaussian();

            ::std::normal_distribution< ::rl::math::Real> gaussianDistribution;

            //::std::mt19937 randEngine;

	    ::std::mt19937 gaussianEngine;

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_ 


