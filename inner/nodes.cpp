#include "nodes.h"
#include "inner.h"

void NODE::addChild(const std::string &nodeId)
{
	children.push_back(nodeId); 
};

void NODE::print() const
{
	for( auto&& i : children)	
	{
		std::cout << "i " << i << (inner == nullptr) << std ::endl;
		auto node = inner->hash.at(i);
		node->print(); 
	}
	std::cout << "PRINT: " << id << " " << children.size() << std::endl;	
}
