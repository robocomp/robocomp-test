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

bool NODE::addChildToParent(const std::string &parentId)
{
    //std::cout << "----------"  << getId() << " xxxxxxxxxx" << inner->getRootId();
    if(getId() == inner->getRootId())
    //if(getId() == "root")
    {    
        parent = "";
        return true;
    }
	auto parentNode = inner->getNode<NODE>(parentId);
	if( parentNode )
		parentNode->addChild(id); 
	else
    {	
        std::cout << "Inner::addChildToParent - " << getId() << "'s parent not found" << std::endl;
        return false;
    }        
    return true;
};


void NODE::addChild(const std::string &childId)
{
	children.push_back(childId);
}

void NODE::removeChild(const std::string &childId)
{
	try 
	{ children.erase(std::remove(children.begin(), children.end(), childId), children.end()); }
	catch(const std::exception &e) { std::cout << e.what() << std::endl;}
}

std::string NODE::getParentId() const	
{ 
	return parent;
}

void NODE::print() const
{
	std::cout << "NODE PRINT: id " << id << " children: " << children.size() << std::endl;	
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

std::ostream& operator<< (std::ostream &out, const std::shared_ptr<NODE> &node)
{
	out << node->getId();
	return out;	
};
