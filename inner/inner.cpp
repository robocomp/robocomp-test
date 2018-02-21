#include <iostream>
#include <thread>
#include <unordered_map>
#include <string>
#include <vector>

class NODE;
class TRANSFORM;
class JOINT;
using NODEPtr = std::shared_ptr<NODE>; 
using TRANSFORMPtr = std::shared_ptr<TRANSFORM>;
using JOINTPtr = std::shared_ptr<JOINT>; 

class NODE
{
	public:
		NODE(std::string&& id_, const NODEPtr &parent_ = nullptr) : id(std::move(id_)) , parent(parent_) 
		{}
		virtual ~NODE(){};
		std::string getId() const 	{ return id; }
		void addChild(NODE *node)	{ children.push_back(node);};
		void print()
		{
			std::cout << "enter to print " << id << " " << children.size() << std::endl;
			std::vector<NODE*>::iterator i;
			for (i=children.begin(); i!=children.end(); i++)
			{
				(*i)->print(); 
			}
			std::cout << "ID:" << id << std::endl;
		}
		
	protected:
		std::string id;
		NODEPtr parent;
		std::vector<NODE*> children;
};

class TRANSFORM : public NODE
{
	public:
		TRANSFORM(std::string&& id_, const NODEPtr &parent_ = nullptr) : NODE(std::move(id_), parent_)
		{
			if( parent != nullptr)
			{
				parent->addChild(this);
				std::cout << "Soy el TRANSFORM " << id << " con padre " << parent->getId() << std::endl;
			}
			else
				std::cout << "Soy el TRANSFORM " << id << " sin padre " << std::endl;
		}
};

class JOINT: public TRANSFORM
{
	public:
		JOINT(std::string&& id_, const NODEPtr &parent_ = nullptr): TRANSFORM(std::move(id_), parent_)
		{
			if( parent != nullptr)
			{
				parent->addChild(this);
				std::cout << "Soy JOINT " << id << " con padre " << parent->getId() << std::endl;
			}
			else
				std::cout << "Soy JOINT " << id << " sin padre " << std::endl;
		}
	private:
		
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
		
		void setRoot(const TRANSFORMPtr &r)						{ root = r;};
		void print() const 										{ root->print();}
		
		std::unordered_map<std::string, std::shared_ptr<NODE>> hash;
		
	private:
		TRANSFORMPtr root;
};

int main()
{
	std::cout << std::boolalpha;   	
	auto inner = std::make_shared<Inner>();
	
	auto a = inner->newNode<TRANSFORM>("root");
	inner->setRoot( a );
	inner->newNode<TRANSFORM>("t1", inner->hash.at("root"));
	inner->newNode<TRANSFORM>("t2", inner->hash.at("root"));
	inner->newNode<JOINT>("c1", inner->hash.at("t2"));
	
	inner->print();
}
