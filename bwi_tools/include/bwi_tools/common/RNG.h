#ifndef BWI_TOOLS_RNG_H
#define BWI_TOOLS_RNG_H

#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/poisson_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>

#include <cstdlib>
#include <vector>

class RNG : public boost::mt19937 {

  public:
    RNG (unsigned int seed) : boost::mt19937(seed) {}

    inline float randomFloat() {
      boost::uniform_real<float> dist;
      boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > gen(*this, dist);
      return gen();
    }

    inline int randomInt(int min, int max) {
      boost::uniform_int<int> dist(min, max);
      boost::variate_generator<boost::mt19937&, boost::uniform_int<int> > gen(*this, dist);
      return gen();
    }

    inline int randomInt(int max = std::numeric_limits<int>::max()) {
      return randomInt(0, max);
    }

    inline int randomUInt(unsigned int max = std::numeric_limits<unsigned int>::max()) {
      boost::uniform_int<unsigned int> dist(0, max);
      boost::variate_generator<boost::mt19937&, boost::uniform_int<unsigned int> > gen(*this, dist);
      return gen();
    }

    inline int poissonInt(int mean) {
      boost::poisson_distribution<int> dist(mean);
      boost::variate_generator<boost::mt19937&, boost::poisson_distribution<int> > gen(*this, dist);
      return gen();
    }

    inline void randomOrdering(std::vector<unsigned int> &inds) {
      unsigned int j;
      unsigned int temp;
      for (unsigned int i = 0; i < inds.size(); i++) {
        inds[i] = i;
      }
      for (int i = (int)inds.size() - 1; i >= 0; i--) {
        j = randomInt(i + 1);
        temp = inds[i];
        inds[i] = inds[j];
        inds[j] = temp;
      }
    }

    inline int select(const std::vector<float>& probabilities) {
      float random_value = randomFloat();
      float prob_sum = probabilities[0];
      for (int i = 1; i < probabilities.size(); ++i) {
        if (random_value < prob_sum) return i - 1;
        prob_sum += probabilities[i];
      }
      return probabilities.size() - 1;
    }
};

#endif /* end of include guard: BWI_TOOLS_RNG_H */
