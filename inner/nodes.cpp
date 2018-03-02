#include "nodes.h"
#include "inner.h"

std::string NODE::getId() const
{ 
	return id; 
	
}
	
void NODE::setId(const std::string &id_)
{ 
	id = id_;
	
}

void NODE::addChildToParent(const std::string &parentId)
{
	//std::unique_lock<std::shared_timed_mutex> lock(mymutex);
	if(parentId == "root")
		parentNode = inner->rootptr;
	else
		parentNode = inner->getNode<NODE>(parentId);
	
	if( parentNode )
	{
		parentNode->children.push_back(id); 
		std::cout << "Inner::addChildToParent added chldren " << id << " to " << parentNode->getId() << std::endl;
	}
};

void NODE::addChild(const std::string &childId)
{
	children.push_back(childId);
}

std::string NODE::getParentId() const	
{ 
	return parent;
}

void NODE::print() const
{
// 	for( auto&& i : children)	
// 	{
// 		auto node = inner->hash.at(i);
// 		node->print(); 
// 	}
	std::cout << "PRINT: id " << id << " children: " << children.size() << std::endl;	
}

std::string NODE::getChildId(uint ix) const
{
	if( !children.empty() and ix < children.size())
		return children[ix];
	else
		return "";
}

std::vector<std::string> NODE::getChildren() const
{
	return children;
}
