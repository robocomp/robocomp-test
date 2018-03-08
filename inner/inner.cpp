#include "inner.h"

void Inner::setRootId(const std::string &r)
{ 
	//rootptr = newNode<TRANSFORM>(r, std::shared_ptr<Inner>(this), "");
    //std::cout << "----------" << this->rootid  << std::endl; 
	rootid = r;
	rootptr = new TRANSFORM("root", std::shared_ptr<Inner>(this), "");
	bool ok = hash->insert({r, std::shared_ptr<NODE>(rootptr)}).second;
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
	std::cout << "-------------------------------- PRINTING TREE ----------------------------------" << std::endl;
	std::stack<std::shared_ptr<Proxy<NODE>>> s;
	auto r = getNode<NODE>("root");
	s.push(r);
	std::stack<int> level;
	int l=0, count=1;
	level.push(l);
	while(not s.empty())
	{
		std::shared_ptr<Proxy<NODE>> node = s.top();
        if (!node) { std::cout << "Fary" << std::endl; exit(-1);}
		std::cout << std::setfill('_') << std::setw (level.top()*5) << node->getId() << std::endl;
		s.pop();
		int localLevel = level.top();
		level.pop();
		if(node->getChildren().size()>0)
		{
			l = localLevel + 1;
			for(auto&& nn : node->getChildren())
			{
                if(auto temp = getNode<NODE>(nn))
                {
                    s.push(temp);
                    level.push(l);
                }
			}
		}
		count++;
	}
	std::cout << count << " elems" << std::endl;
        std::cout << "-------------------------------- END TREE --------------------------------------" << std::endl;

}

void Inner::deleteNode(const std::string &id)
{
	if(id == rootid) 
		return;
	auto node = getNode<NODE>(id);
        if(!node)
        {
            std::cout << "Inner::deleteNode: returning without deleting" << std::endl;
            return;
        }
	//std::cout << "entering DELETE: " << node.use_count() << std::endl;
	node->markForDelete();
	while( node->getWaiting() > 1);
	if(node.unique())
	{
		auto p = getNode<NODE>(node->getParentId());
		if(p)
			p->removeChild(id);
		node.reset();
		hash->erase(id);
		//std::cout << "DELETE: node " << id << " deleted OK " << node.use_count() << " references" << std::endl;
	}
	else
	{
		std::cout << "DELETE: could not delete node " << node.use_count() << std::endl;
	}
}

void Inner::removeSubTree(const std::string id, std::vector<std::string> &l)
{
    auto node = getNode<NODE>(id);
    if(!node) throw std::runtime_error("Attempt to delete a not existing node.");
    for(auto&& n : node->getChildren())
      removeSubTree(n, l);  
    node->markForDelete();
    while( node->getWaiting() > 1)
        std::this_thread::sleep_for(1ms);
    if(auto p = getNode<NODE>(node->getParentId()))
        p->removeChild(id);
    hash->erase(id);
    l.push_back(id);
    node.reset();
}   




// SEG FAULTS when accesing an a deleted node

std::ostream& operator<< (std::ostream &out, const std::shared_ptr<Proxy<NODE>> &node)
{
	out << node->node;
	return out;	
};
