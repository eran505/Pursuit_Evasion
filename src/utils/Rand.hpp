//
// Created by eranhe on 6/6/21.
//

#ifndef PE_RAND_HPP
#define PE_RAND_HPP
#include <random>

struct Randomizer{
public:
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution;

    explicit Randomizer(int seed):
            generator(seed),
            distribution(0.0,1.0){}
    double get_double(){return this->distribution(this->generator);}
};


class RandomSingleton  {

private:
    Randomizer rand;
    RandomSingleton(int seed) : rand(Randomizer(seed)){}
    static std::shared_ptr<RandomSingleton> s_pSingleton;

public:
    double random(){ return this->rand.get_double();}
    static std::shared_ptr<RandomSingleton> get_random_instances(int seed)
    {
        if( s_pSingleton == 0 ) {
            s_pSingleton = std::shared_ptr<RandomSingleton>(new RandomSingleton(seed));
        }
        return s_pSingleton;
    }

};


#endif //PE_RAND_HPP
