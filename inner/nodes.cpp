#include "nodes.h"
#include "inner.h"

void NODE::addChild(const std::string &nodeId)
{
	//std::unique_lock<std::shared_timed_mutex> lock(mymutex);
	children.push_back(nodeId); 
};

void NODE::print() const
{
	for( auto&& i : children)	
	{
		auto node = inner->hash.at(i);
		node->print(); 
	}
	//std::cout << "PRINT: id " << id << " children: " << children.size() << std::endl;	
}
