
#ifndef NODES_H
#define NODES_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <vector>

using namespace std::chrono_literals;

class Inner;

class NODE
{
	public:
		NODE(){};
		NODE(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::shared_ptr<NODE> &parent_ = nullptr) : id(id_) , parent(parent_) 
		{
			inner = inner_;
			if( parent != nullptr)
				parent->addChild(id);
		}
		virtual ~NODE(){};
		std::string getId() const 						{ return id; }
		std::string getId2() const 						{ return id2; }
		void setId(const std::string &id_) 				{ id = id_;}
		void setId2(const std::string &id_) 			{ id2 = id_;}
		void addChild(const std::string &nodeId);					
		void print() const;
		bool lock() 									{ return mymutex.try_lock_shared_for(10ms);};
		void unlock()   								{ mymutex.unlock();};
		
	protected:
		mutable std::shared_timed_mutex mymutex;
		std::string id, id2;
		std::shared_ptr<NODE> parent;
		std::vector<std::string> children;
		std::shared_ptr<Inner> inner;
};

class TRANSFORM : public NODE
{
	public:
		TRANSFORM(){};
		TRANSFORM(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::shared_ptr<NODE> &parent_ = nullptr) : NODE(id_, inner_, parent_)
		{}
};

class JOINT: public TRANSFORM
{
	public:
		JOINT(){};
		JOINT(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::shared_ptr<NODE> &parent_ = nullptr): TRANSFORM(id_, inner_, parent_)
		{}
	private:
		
};


#endif // NODES_H
