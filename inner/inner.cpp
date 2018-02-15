#include <iostream>
#include <thread>
#include <unordered_map>
#include <string>

class NODE;
class TRANSFORM;
using NODEPtr = std::shared_ptr<NODE>; 
using TRANSFORMPtr = std::shared_ptr<TRANSFORM>;
//using CAMERAPtr = std::shared_ptr<CAMERA>; 

class NODE
{
	public:
		NODE(std::string&& id_, const NODEPtr &parent_) : id(std::move(id_)) , parent(parent_)
		{
		}
		virtual ~NODE(){};
		std::string getId() const { return id; }
	protected:
		std::string id;
		NODEPtr parent;
};

class TRANSFORM : public NODE
{
	public:
		TRANSFORM(std::string&& id_, const NODEPtr &parent_ = nullptr) : NODE(std::move(id_), parent_)
		{
			std::cout << "Soy el TRANSFORM: " << id << std::endl;
			std::cout << "gola" << std::endl;
		}	
};

// class CAMERA : public NODE
// {
// 	public:
// 		CAMERA(std::string&& id_): NODE(std::move(id_))
// 		{
// 			//id = std::move(id_);
// 			std::cout << "Soy CAMERA" << std::endl;
// 		}
// 	private:
// 		
// };

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
			std::cout << "gola1" << std::endl;
			NODEPtr kk = std::static_pointer_cast<NODE>(node);
			std::cout << "gola2 " << kk->getId() << " " << (kk==nullptr) << std::endl;
			NODEPtr kkk;
			hash.insert({id, kkk});
			//bool ok = hash.insert({id, kk}).second;
			std::cout << "gola3" << std::endl;
			return node;
		}
		//void setRoot(TRANSFORMPtr&& r)						{ std::cout << "gola" << std::endl ; root = std::move(r);};
		void setRoot(const TRANSFORMPtr &r)						{ std::cout << "gola" << std::endl ; root = r;};
		
		std::unordered_map<std::string, std::shared_ptr<NODE>> hash;
		std::unordered_map<std::string, int> ihash = {{"gola",5}};
		
		
	private:
		TRANSFORMPtr root;
};

int main()
{
	std::cout << std::boolalpha;   	
	Inner *inner = new Inner();
	std::cout << inner->ihash.size() << std::endl;
	inner->ihash.insert({"gola", 5});
	std::cout << "gola3.5" << std::endl;
	inner->hash.insert({"gola", std::shared_ptr<NODE>()});
	
	auto a = inner->newNode<TRANSFORM>("root");
	std::cout << "gola4" << std::endl;
	inner->setRoot( a );
	std::cout << "gola5" << std::endl;
	inner->newNode<TRANSFORM>("t1", inner->hash.at("root"));
}
