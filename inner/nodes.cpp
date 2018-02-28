#include "nodes.h"
#include "inner.h"

void NODE::addChildToParent(const std::string &parentId)
{
	//std::unique_lock<std::shared_timed_mutex> lock(mymutex);
	auto parentNode = inner->getNode<NODE>(parentId);
	if( parentNode != nullptr)
		parentNode->children.push_back(id); 
};

void NODE::addChild(const std::string &childId)
{
	children.push_back(childId);
}

void NODE::print() const
{
	for( auto&& i : children)	
	{
		auto node = inner->hash.at(i);
		node->print(); 
	}
	//std::cout << "PRINT: id " << id << " children: " << children.size() << std::endl;	
}

std::string NODE::getChildId(unsigned i) const			
{ 
	if(i < children.size())
		return children[i];
	else 
		return "";
}