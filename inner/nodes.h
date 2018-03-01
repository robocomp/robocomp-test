
#ifndef NODES_H
#define NODES_H

#include <string>
#include <iostream>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <vector>
#include <atomic>

using namespace std::chrono_literals;

class Inner;

class NODE
{
	public:
		NODE(){};
		NODE(const std::string& id_, const std::shared_ptr<Inner> &inner_, const std::string &parent_ = "") : id(id_) , parent(parent_) 
		{
			inner = inner_;
			addChildToParent( parent );
		}
		virtual ~NODE(){};
		std::string getId() const 						{ return id; }
		std::string getId2() const 						{ return id2; }
		void setId(const std::string &id_) 				{ id = id_;}
		void setId2(const std::string &id_) 			{ id2 = id_;}
		void addChild(const std::string &nodeId);		
		void addChildToParent(const std::string &parentId);
		std::string getParentId() const					{ return parent;}
		std::string getChildId(unsigned int i) const;
		void print() const;
		bool lock() 									{ return mymutex.try_lock_shared_for(10ms);};
		void unlock()   								{ mymutex.unlock();};
		bool isMarkedForDelete() const					{ return markedForDelete.load(); };
		void markForDelete() 							{ markedForDelete.store(true);  };
		void incWaiting() 								{ lockWaiting++;}
		void decWaiting() 								{ lockWaiting--;}
		ulong getWaiting() const 						{ return lockWaiting;}
		
	protected:
		mutable std::shared_timed_mutex mymutex;
		std::string id, id2;
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
