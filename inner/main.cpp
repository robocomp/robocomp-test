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


////////////////////////////////////////////////////////////////////////////////////////////////////////

void readThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::vector<std::string> keys = inner->hash.keys();
	std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
	while(true)
	{
		
		inner->print();
// 		auto target = keys[uniform_dist(e1)];
// 		auto node = inner->getNode<NODE>(target);
// 		if(node.operator bool() )
// 			std::cout <<  "thread " << node->getId2() << " and " << node->getId() << std::endl;
		std::this_thread::sleep_for(100ms);
	}
}
void writeThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	
	while(true)
	{
		std::vector<std::string> keys = inner->hash.keys();
		std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
		auto target = keys[uniform_dist(e1)];
		auto node = inner->getNode<TRANSFORM>(target);
		if(node.operator bool())
		{
			auto id = node->getId();
			std::this_thread::sleep_for(1ms);		
			node->setId(id);
			std::this_thread::sleep_for(100ms);		
		}
	}
}

void createThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::uniform_int_distribution<int> name_dist(0, 10000);
	
	while(true)
	{
		std::vector<std::string> keys = inner->hash.keys();
		std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
		auto parent = keys[uniform_dist(e1)];
		std::string id = std::to_string(name_dist(e1));
		std::cout << "new id " << id << std::endl;
		try
		{	auto n = inner->newNode<TRANSFORM>(id, inner, inner->hash.at(parent)); }
		catch(const std::exception &e) 
		{	std::cout << e.what() << std::endl;}
		std::this_thread::sleep_for(100ms);		
	}
}

int main()
{
	std::cout << std::boolalpha;   	
	auto inner = std::make_shared<Inner>();
	
	auto a = inner->newNode<TRANSFORM>("root", inner);
	inner->setRoot( a );
	auto t1 = inner->newNode<TRANSFORM>("t1", inner, inner->hash.at("root")); t1->setId2("caca1");
	auto t2 = inner->newNode<TRANSFORM>("t2", inner, inner->hash.at("root")); t2->setId2("caca2");
	auto t3 = inner->newNode<TRANSFORM>("t3", inner, inner->hash.at("root")); t3->setId2("caca3");
	auto t4 = inner->newNode<TRANSFORM>("t4", inner, inner->hash.at("root")); t4->setId2("caca4");

	inner->newNode<JOINT>("j1", inner, inner->hash.at("t1"));
	inner->newNode<JOINT>("j2", inner, inner->hash.at("t2"));
	inner->newNode<JOINT>("j3", inner, inner->hash.at("t3"));
	
	inner->newNode<TRANSFORM>("t5", inner, inner->hash.at("t4"));
	inner->newNode<TRANSFORM>("t6", inner, inner->hash.at("t4"));
	
	std::cout << "Created, now printing" << std::endl;
	inner->print();
	
	std::cout << "----------------getNode---------------------" << std::endl;
	auto j = inner->getNode<JOINT>("j1");
	j->print();
	std::shared_ptr<Proxy<TRANSFORM>>  t = inner->getNode<TRANSFORM>("t1");
	t->print();
	t->print();
	
	std::cout << "-----------threads-------------------------" << std::endl;
	std::vector<int> RN = {0,1}, WN = {0,1,2,3,4,5}, CN = {0};
	std::future<void> threadsR[RN.size()], threadsW[WN.size()], threadsC[CN.size()];
	
	for (auto&& i : RN)
		threadsR[i] = std::async(std::launch::async, readThread, inner);

	
// 	for (auto&& i : WN)
// 		threadsW[i] = std::async(std::launch::async, writeThread, inner);
// 	for (auto&& i: WN)
// 		threadsW[i].wait();
	
	for (auto&& i : CN)
		threadsC[i] = std::async(std::launch::async, createThread, inner);
  	for (auto&& i: CN)
		threadsC[i].wait();
	for (auto&& i: RN)
		threadsR[i].wait();  	
}
