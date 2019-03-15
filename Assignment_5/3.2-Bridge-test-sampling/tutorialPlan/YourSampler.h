#ifndef _YOURSAMPLER_H_
#define _YOURSAMPLER_H_


#include <rl/plan/GaussianSampler.h>
#include <random>

#include "GaussianSampler.h"

namespace rl
{
    namespace plan
    {
        /**
         * Bridge sampling
         */
        class YourSampler : public GaussianSampler
        {
        public:
            YourSampler();

            virtual ~YourSampler();

            ::rl::math::Vector generateCollision();

	    ::rl::math::Real ratio;

        protected:

        private:

        };
    }
}


#endif // _YOURSAMPLER_H_ 


