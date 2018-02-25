#include <iostream>
#include <thread>
#include <unordered_map>
#include <string>
#include <vector>
#include <mutex>
#include <random>
#include <future>
#include <chrono>

using namespace std::chrono_literals;

class NODE;
class TRANSFORM;
class JOINT;
class Inner;
using NODEPtr = std::shared_ptr<NODE>; 
using TRANSFORMPtr = std::shared_ptr<TRANSFORM>;
using JOINTPtr = std::shared_ptr<JOINT>; 
using INNERPtr = std::shared_ptr<Inner>;

namespace mapExt
{
    template<typename myMap>
    std::vector<typename myMap::key_type> Keys(const myMap& m)
    {
        std::vector<typename myMap::key_type> r;
        r.reserve(m.size());
        for (const auto&kvp : m)
        {
            r.push_back(kvp.first);
        }
        return r;
    }

    template<typename myMap>
    std::vector<typename myMap::mapped_type> Values(const myMap& m)
    {
        std::vector<typename myMap::mapped_type> r;
        r.reserve(m.size());
        for (const auto&kvp : m)
        {
            r.push_back(kvp.second);
        }
        return r;
    }
}


class NODE
{
	public:
		NODE(const std::string& id_, const INNERPtr &inner_, const NODEPtr &parent_ = nullptr) : id(id_), parent(parent_) 
		{
			inner = inner_;
			if( parent != nullptr)
			{
				parent->addChild(id);
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
		void addChild(const std::string &id)			{ children.push_back(id);};
		void print()
		{
			std::cout << "PRINT: enter to print " << id << " " << children.size() << std::endl;
			for(auto&& id : children)
			{
				auto child = inner->getNode<NODE>(id);
				child->print();
			}
			
			std::vector<std::string>::iterator i;
			//for (i=children.begin(); i!=children.end(); i++)
			//{
			//	(*i)->print(); 
			//}
			std::cout << "PRINT: ID:" << id << std::endl;
		}
		
	protected:
		mutable std::mutex mymutex;
		std::string id, id2;
		NODEPtr parent;
		std::vector<std::string> children;
		std::shared_ptr<Inner> inner;
};

class TRANSFORM : public NODE
{
	public:
		TRANSFORM(const std::string& id_, const INNERPtr &inner_,const NODEPtr &parent_ = nullptr) : NODE(id_, inner, parent_)
		{}
};

class JOINT: public TRANSFORM
{
	public:
		JOINT(const std::string& id_, const INNERPtr &inner_, const NODEPtr &parent_ = nullptr): TRANSFORM(id_, inner, parent_)
		{}
	private:
		
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////7

template <typename T>
class Proxy : public T
{
	public:
		Proxy(const std::shared_ptr<T> &node_) : T(node_->getId())
		//Proxy(T *node_) : T(node_->getId())
		{
			node = node_;
			T::mymutex.lock();
			//std::cout << node->getId() << " dentro " << node->getId2() << std::endl;
		}
		~Proxy()							{ T::mymutex.unlock();	}
		//std::shared_ptr<T> operator ->() 	{ std::cout << node->getId() << " dentro " << node->getId2() << std::endl; return node;}
		//std::string getId() const 			{ return node->getId(); }
		std::string getId2T() const 		{ return node->getId2(); }
		
		std::shared_ptr<T> node;
		//T* node;
};

class Inner
{
	
	public:
		Inner(){};
		////////////////////////////////
		/// Factory constructor
		///////////////////////////////
		template<typename T, typename... Ts>
		std::shared_ptr<T> newNode(Ts&&... params)
		{
			auto t = std::make_tuple(params...);
			std::string id = (std::get<0>(t));
			std::shared_ptr<T> node(new T(std::forward<Ts>(params)...)); 
			bool ok = hash.insert({id, std::static_pointer_cast<NODE>(node)}).second;
			if(ok)
				return node;
			else 
				throw;
		}
		//////////////////////////////////////
		/// Node getter
		/////////////////////////////////////
		template <typename N> 
		std::shared_ptr<Proxy<N>> getNode(const std::string &id) 
		{
			try 
			{ 
				auto n = hash.at(id);
				std::cout << "shit" << std::endl;
				auto nn = std::static_pointer_cast<N>(n);
				return std::shared_ptr<Proxy<N>>(new Proxy<N>(nn));
			}
			catch(const std::exception &e)
			{ 
				std::cout << e.what() << std::endl;
				return std::shared_ptr<Proxy<N>>();
			}
		}
		void setRoot(const TRANSFORMPtr &r)						{ root = r;};
		void print() const 										{ root->print();}
		
		std::unordered_map<std::string, std::shared_ptr<NODE>> hash;
		
	private:
		TRANSFORMPtr root;
};


////////////////////////////////////////////////////////////////////////////////////////////////////////

void readThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::vector<std::string> keys = mapExt::Keys(inner->hash);
	std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
	while(true)
	{
		//inner->print();
		auto target = keys[uniform_dist(e1)];
		auto node = inner->getNode<TRANSFORM>(target);
		std::cout <<  node->getId2T() << " " << node->getId() << std::endl;
		std::this_thread::sleep_for(1ms);
	}
}
void writeThread(const std::shared_ptr<Inner> &inner)
{
	std::random_device r;
	std::default_random_engine e1(r());
	std::vector<std::string> keys = mapExt::Keys(inner->hash);
	std::uniform_int_distribution<int> uniform_dist(0, keys.size()-1);
	
	while(true)
	{
		auto target = keys[uniform_dist(e1)];
		auto node = inner->getNode<TRANSFORM>(target);
		auto id = node->getId();
		std::this_thread::sleep_for(1ms);		
		node->setId(id);
		std::this_thread::sleep_for(1ms);		
	}
}

int main()
{
	std::cout << std::boolalpha;   	
	auto inner = std::make_shared<Inner>();
	
	auto a = inner->newNode<TRANSFORM>("root");
	inner->setRoot( a );
	auto t1 = inner->newNode<TRANSFORM>("t1", inner, inner->hash.at("root")); t1->setId2("caca1");
	auto t2 = inner->newNode<TRANSFORM>("t2", inner, inner->hash.at("root")); t2->setId2("caca2");
	auto t3 = inner->newNode<TRANSFORM>("t3", inner, inner->hash.at("root")); t3->setId2("caca3");
	auto t4 = inner->newNode<TRANSFORM>("t4", inner, inner->hash.at("root")); t4->setId2("caca4");

	inner->newNode<JOINT>("j1", inner, inner->hash.at("t1"));
	inner->newNode<JOINT>("j2", inner, inner->hash.at("t2"));
	inner->newNode<JOINT>("j3", inner, inner->hash.at("t3"));
	
	inner->newNode<TRANSFORM>("t5", inner, inner->hash.at("t4"));
	inner->newNode<TRANSFORM>("t6", inner, inner->hash.at("t4"));
	
	inner->print();
	
	std::cout << "----------------getNode---------------------" << std::endl;
	auto j = inner->getNode<JOINT>("j1");
	j->print();
	auto t = inner->getNode<TRANSFORM>("c1");
	t->print();
	
	std::cout << "-----------threads-------------------------" << std::endl;
//  	auto task1 = std::async(std::launch::async, readThread, inner);
//  	auto task2 = std::async(std::launch::async, writeThread, inner);
//  	task1.wait();
// 	task2.wait();
	
}
