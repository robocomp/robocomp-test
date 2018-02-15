#include <iostream>
#include <thread>
#include <unordered_map>
#include <string>

class ROOT
{
	ROOT()
	{
		std::cout << "Soy ROOT" << std::endl;
	}
};

class TRANSFORM
{
	TRANSFORM()
	{
		std::cout << "Soy TRANSFORM" << std::endl;
	}
};

class CAMERA
{
	CAMERA()
	{
		std::cout << "Soy CAMERA" << std::endl;
	}
};

class Inner
{
	
	public:
		using ROOTPtr = std::shared_ptr<ROOT>; 
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
			hash.try_emplace(id, std::static_pointer_cast<ROOT>(node));
			return node;
		}
		void setRoot(RootPtr r)		{ root = r;};
	private:
		std::unordered_map<std::string, ROOT> hash;
		ROOTPtr root;
};

int main()
{

	Inner inner = std::shared_ptr<Inner>();
	inner->setRoot( inner->newNode<TRANSFORM>() );
	
	
	

}
