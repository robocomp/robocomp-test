#include "inner.h"

void Inner::setRootId(const std::string &r)
{ 
	//rootptr = newNode<TRANSFORM>(r, std::shared_ptr<Inner>(this), "");
	rootptr = new TRANSFORM("root", std::shared_ptr<Inner>(this), "");
	rootid = r;
	bool ok = hash.insert({r, std::shared_ptr<NODE>(rootptr)}).second;
	if(ok)
		std::cout << "Inner::setRoot Create root node as " << r << std::endl;
	else
		std::cout << "Inner::setRoot ERROR Could nor create existing root node" << r << std::endl;	
	
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
	std::stack<std::shared_ptr<Proxy<NODE>>> s;
	auto r = getNode<NODE>("root");
	s.push(r);
	std::stack<int> level;
	int l=0;
	level.push(l);
	while(not s.empty())
	{
		std::shared_ptr<Proxy<NODE>> node = s.top();
		std::cout << std::setfill('_') << std::setw (level.top()*5) << node << std::endl;
		s.pop();
		int aux = level.top();
		level.pop();
		if(node->getChildren().size()>0)
		{
			l=aux + 1;
			for(auto&& nn : node->getChildren())
			{
				s.push(getNode<NODE>(nn));
				level.push(l);
			}
		}
	}
}

void Inner::deleteNode(const std::string &id)
{
	if(id == rootid) 
		return;
	auto node = getNode<NODE>(id);
	//std::cout << "entering DELETE: " << node.use_count() << std::endl;
	node->markForDelete();
	while( hash.at(id)->getWaiting() > 1);
	if(node.unique())
	{
		auto p = getNode<NODE>(node->getParentId());
		if(p)
			p->removeChild(id);
		node.reset();
		hash.erase(id);
		//std::cout << "DELETE: node " << id << " deleted OK " << node.use_count() << " references" << std::endl;
	}
	else
	{
		std::cout << "DELETE: could not delete node " << node.use_count() << std::endl;
	}
}

std::ostream& operator<< (std::ostream &out, const std::shared_ptr<Proxy<NODE>> &node)
{
	out << node->node;
	return out;	
};
