
#ifndef NODES_H
#define NODES_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include <atomic>
#include <algorithm>

using namespace std::chrono_literals;

class Inner;

class NODE
{
	public:
		NODE(){};
		NODE(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::string &parent_ = "") : id(id_) , parent(parent_) 
		{
			inner = inner_;
			if(!addChildToParent( parent ))
                throw std::runtime_error("NODE() - cannot create a node without parent");
		}
		virtual ~NODE(){};
		std::string getId() const ;					
		void setId(const std::string &id_);
		void addChild(const std::string &nodeId);;		
		bool addChildToParent(const std::string &parentId);
		void removeChild(const std::string &childId);
		std::string getParentId() const;				
		std::string getChildId(unsigned int i) const;
		std::vector<std::string> getChildren() const;
		void print() const;
		void lock() 					{ return mymutex.lock();};
		void unlock()   				{ mymutex.unlock();};
		bool isMarkedForDelete() const	{ return markedForDelete.load(); };
		void markForDelete() 			{ markedForDelete.store(true);  };
		void incWaiting() 				{ lockWaiting++;}
		void decWaiting() 				{ lockWaiting--;}
		ulong getWaiting() const 		{ return lockWaiting;}
		friend std::ostream& operator<< (std::ostream &out, const std::shared_ptr<NODE> &node);
		
	protected:
		mutable std::recursive_mutex mymutex;
		std::string id;
		std::string parent;
		std::vector<std::string> children;
		std::shared_ptr<Inner> inner;
		std::atomic<bool> markedForDelete{false};
		std::atomic<ulong> lockWaiting{0};
};

class TRANSFORM : public NODE
{
	public:
		TRANSFORM(){};
		TRANSFORM(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::string &parent_ = "") : NODE(id_, inner_, parent_)
		{}
};

class JOINT: public TRANSFORM
{
	public:
		JOINT(){};
		JOINT(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::string &parent_ = ""): TRANSFORM(id_, inner_, parent_)
		{}
	private:
		
};


#endif // NODES_H
