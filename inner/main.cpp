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
		inner->printIter();
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
		std::cout << "size " << inner->hash.size() << std::endl;
		std::vector<std::string> keys = inner->hash.keys();
		std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
		auto parent = keys[uniform_dist(e1)];
		std::string id = std::to_string(name_dist(e1));
		std::cout << "THREAD: creating id " << id << std::endl;
		try
		{	auto n = inner->newNode<TRANSFORM>(id, inner, parent); }
		catch(const std::exception &e) 
		{	std::cout << e.what() << std::endl;}
		std::this_thread::sleep_for(100ms);		
	}
}

void chainThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::uniform_int_distribution<int> chain_dist(1, 6);
	
	while(true)
	{
		std::vector<std::string> keys = inner->hash.keys();
		std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
		auto id = keys[uniform_dist(e1)];
		auto running_id = id, parent_id=id;
		auto levelsUp = chain_dist(e1);
		auto levelsDown = chain_dist(e1);		
		int cont = 0;
		do
		{
			running_id = parent_id;
			parent_id = inner->getNode<NODE>(running_id)->getParentId();
			cont++;
		}
		while(parent_id != "" or cont>levelsUp);
		cont = 0;
		auto child_id = running_id;
		do
		{
			running_id = child_id;
			child_id = inner->getNode<NODE>(running_id)->getChildId(0);
			cont++;
		}
		while(child_id != "" or cont>levelsDown);
		std::cout << "THREAD transform: up " << levelsUp << " down "  << levelsDown << std::endl;
		std::this_thread::sleep_for(100ms);		
	}
}

void deleteThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	
	while(true)
	{
		std::vector<std::string> keys = inner->hash.keys();
		std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
		auto id = keys[uniform_dist(e1)];
		std::cout << "THREAD: deleting id " << id << std::endl;
		inner->deleteNode(id);
		std::cout << "THREAD: id  deleted ok" << id << std::endl;
		std::this_thread::sleep_for(100ms);		
	}
}

int main()
{
	std::cout << std::boolalpha;   	
	auto inner = std::make_shared<Inner>();
	
	inner->setRoot("root");
	inner->newNode<TRANSFORM>("t1", inner, "root"); 
	inner->newNode<TRANSFORM>("t2", inner, "root"); 
	inner->newNode<TRANSFORM>("t3", inner, "root"); 
	inner->newNode<TRANSFORM>("t4", inner, "root"); 

	inner->newNode<JOINT>("j1", inner, "t1");
	inner->newNode<JOINT>("j2", inner, "t2");
	inner->newNode<JOINT>("j3", inner, "t3");
	
	inner->newNode<TRANSFORM>("t5", inner, "t4");
	inner->newNode<TRANSFORM>("t6", inner, "t4");
	
	std::cout << "---------------Created, now printing--------------" << std::endl;
	inner->printIter();
	
	std::cout << "----------------Get Node---------------------" << std::endl;
	auto j = inner->getNode<JOINT>("j1");
	j->print();
	j.reset();
	auto t = inner->getNode<TRANSFORM>("t1");
	t->print();
	t.reset();
	
	std::cout << "----------------Delete Node---------------------" << std::endl;
	//std::cout << "COUNTER " << inner->getNode<NODE>("t5").use_count() << std::endl;
	//auto node = inner->getNode<NODE>("t5");
	//std::cout << "COUNTER " << node.use_count() << std::endl;
	//node->markForDelete();
	
	//inner->deleteNode("t5");
	//node.reset();
	//std::cout << "asdfa " << node.use_count() << std::endl;
	//inner->hash.erase("t5");
	
	/////////////////////
	
	std::cout << "-----------threads-------------------------" << std::endl;
	std::vector<int> RN = {0}, WN = {}, CN = {0}, TN = {}, DN = {};
	std::future<void> threadsR[RN.size()], threadsW[WN.size()], threadsC[CN.size()], threadsT[TN.size()], threadsD[DN.size()];
	
	for (auto&& i : RN)
		threadsR[i] = std::async(std::launch::async, readThread, inner);
	
 	for (auto&& i : WN)
 		threadsW[i] = std::async(std::launch::async, writeThread, inner);
	
	for (auto&& i : CN)
		threadsC[i] = std::async(std::launch::async, createThread, inner);
	
	for (auto&& i : TN)
		threadsT[i] = std::async(std::launch::async, chainThread, inner);
	
	for (auto&& i : DN)
		threadsD[i] = std::async(std::launch::async, deleteThread, inner);
	
	for (auto&& i: DN)
		threadsD[i].wait();
	for (auto&& i: CN)
		threadsC[i].wait();
	for (auto&& i: RN)
		threadsR[i].wait();  	
	for (auto&& i: WN)
		threadsW[i].wait();
	for (auto&& i: TN)
		threadsT[i].wait();
	
}
