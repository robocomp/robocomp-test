#include "inner.h"

void Inner::setRoot(const std::string &r)
{ 
	rootptr = newNode<TRANSFORM>(r, std::shared_ptr<Inner>(this), "");
	if(rootptr != nullptr)
	{
		rootid = r;
		std::cout << "Inner::setRoot Create root node as " << r << std::endl;
	}
	else
		std::cout << "ERROR: Inner::setRoot could not create root node as " << r << std::endl;
};

void Inner::print() const 										
{ 
// 	std::cout << "---------------------------------------" << std::endl; 
// 	std::cout << "Elements in tree: " << hash.size() << std::endl;
// 	root->print();
}

void Inner::printIter() 
{
	if (rootid == "")
	{
		std::cout << "Inner::printIter Cannot print cause root is empty" << std::endl;
		return;
	}
	std::stack<NODEPtr> s;
	s.push(rootptr);
	while(not s.empty())
	{
		NODEPtr node = s.top();
		std::cout << "OOOPS" << node->getId() << " " << node->getChildren().size() << std::endl;
		node->print();
		s.pop();
		for(auto&& nn : node->getChildren())
			s.push(getNode<NODE>(nn));
	}
}

void Inner::deleteNode(const std::string &id)
{
	if(id == rootid) //CHECK WHAT HAPPENS WITHOUT HIS
		return;
	auto node = getNode<NODE>(id);
	//std::cout << "entering DELETE: " << node.use_count() << std::endl;
	node->markForDelete();
	while( hash.at(id)->getWaiting() > 1);
	if(node.unique())
	{
		node.reset();
		hash.erase(id);
		//std::cout << "DELETE: node " << id << " deleted OK " << node.use_count() << " references" << std::endl;
	}
	else
	{
		std::cout << "DELETE: could not delete node " << node.use_count() << std::endl;
	}
}
