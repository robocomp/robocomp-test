#include <iostream>
#include <thread>
#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>
#include <random>
#include <future>
#include <chrono>
#include <shared_mutex>

#include "inner.h"
#include "nodes.h"

using namespace std::chrono_literals;

using NODEPtr = std::shared_ptr<NODE>; 
using TRANSFORMPtr = std::shared_ptr<TRANSFORM>;
using JOINTPtr = std::shared_ptr<JOINT>; 

namespace mapExt
{
    template<typename myMap>
    std::vector<typename myMap::key_type> Keys(const myMap& m)
    {
        std::vector<typename myMap::key_type> r;
        r.reserve(m.size());
        for (const auto&kvp : m)
        {
            r.push_back(kvp.first);
        }
        return r;
    }

    template<typename myMap>
    std::vector<typename myMap::mapped_type> Values(const myMap& m)
    {
        std::vector<typename myMap::mapped_type> r;
        r.reserve(m.size());
        for (const auto&kvp : m)
        {
            r.push_back(kvp.second);
        }
        return r;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

void readThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::vector<std::string> keys = mapExt::Keys(inner->hash);
	std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
	while(true)
	{
		//inner->print();
		auto target = keys[uniform_dist(e1)];
		auto node = inner->getNode<NODE>(target);
		std::cout <<  "thread " << node->getId2() << " and " << node->getId() << std::endl;
		std::this_thread::sleep_for(1ms);
	}
}
void writeThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::vector<std::string> keys = mapExt::Keys(inner->hash);
	std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
	
	while(true)
	{
		auto target = keys[uniform_dist(e1)];
		auto node = inner->getNode<TRANSFORM>(target);
		auto id = node->getId();
		std::this_thread::sleep_for(1ms);		
		node->setId(id);
		std::this_thread::sleep_for(1ms);		
	}
}

int main()
{
	std::cout << std::boolalpha;   	
	auto inner = std::make_shared<Inner>();
	
	auto a = inner->newNode<TRANSFORM>("root");
	inner->setRoot( a );
	auto t1 = inner->newNode<TRANSFORM>("t1", inner->hash.at("root")); t1->setId2("caca1");
	auto t2 = inner->newNode<TRANSFORM>("t2", inner->hash.at("root")); t2->setId2("caca2");
	auto t3 = inner->newNode<TRANSFORM>("t3", inner->hash.at("root")); t3->setId2("caca3");
	auto t4 = inner->newNode<TRANSFORM>("t4", inner->hash.at("root")); t4->setId2("caca4");

	inner->newNode<JOINT>("j1", inner->hash.at("t1"));
	inner->newNode<JOINT>("j2", inner->hash.at("t2"));
	inner->newNode<JOINT>("j3", inner->hash.at("t3"));
	
	inner->newNode<TRANSFORM>("t5", inner->hash.at("t4"));
	inner->newNode<TRANSFORM>("t6", inner->hash.at("t4"));
	
	inner->print();
	
	std::cout << "----------------getNode---------------------" << std::endl;
	auto j = inner->getNode<JOINT>("j1");
	j->print();
	std::shared_ptr<Proxy<TRANSFORM>>  t = inner->getNode<TRANSFORM>("t1");
	t->print();
	t->print();
	
	std::cout << "-----------threads-------------------------" << std::endl;
  	auto task1 = std::async(std::launch::async, readThread, inner);
//  	auto task2 = std::async(std::launch::async, writeThread, inner);
  	task1.wait();
// 	task2.wait();
	
}
