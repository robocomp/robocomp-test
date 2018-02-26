
#ifndef NODES_H
#define NODES_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>


class Inner;

class NODE
{
	public:
		NODE(const std::string& id_, const std::shared_ptr<NODE> &parent_ = nullptr) : id(std::move(id_)) , parent(parent_) 
		{
			if( parent != nullptr)
			{
				parent->addChild(this);
				//std::cout << "Soy el TRANSFORM " << id << " con padre " << parent->getId() << std::endl;
			}
			//else
				//std::cout << "Soy el TRANSFORM " << id << " sin padre " << std::endl;
		}
		virtual ~NODE(){};
		std::string getId() const 						{ return id; }
		std::string getId2() const 						{ return id2; }
		void setId(const std::string &id_) 				{ id = id_;}
		void setId2(const std::string &id_) 			{ id2 = id_;}
		void addChild(NODE *node)						{ children.push_back(node);};
		void print()
		{
			std::cout << "PRINT: enter to print " << id << " " << children.size() << std::endl;
			std::vector<NODE*>::iterator i;
			for (i=children.begin(); i!=children.end(); i++)
			{
				(*i)->print(); 
			}
			std::cout << "PRINT: ID:" << id << std::endl;
		}
		void lock() 	{ mymutex.lock();};
		void unlock()   { mymutex.unlock();};
		
		
	protected:
		mutable std::mutex mymutex;
		std::string id, id2;
		std::shared_ptr<NODE> parent;
		std::vector<NODE*> children;
};

class TRANSFORM : public NODE
{
	public:
		TRANSFORM(const std::string& id_, const std::shared_ptr<NODE> &parent_ = nullptr) : NODE(id_, parent_)
		{}
};

class JOINT: public TRANSFORM
{
	public:
		JOINT(const std::string& id_, const std::shared_ptr<NODE> &parent_ = nullptr): TRANSFORM(id_, parent_)
		{}
	private:
		
};


#endif // NODES_H
