

using namespace std;

class ROOT
{
	using namespace std;

	ROOT()
	{
		cout << "Soy ROOT" << endl;
	}
};

class TRANSFORM
{
	TRANSFORM()
	{
		cout << "Soy TRANSFORM" << endl;
	}
};

class CAMERA
{
	CAMERA()
	{
		cout << "Soy CAMERA" << endl;
	}
};


////////////////////////////////
/// Factory constructor
///////////////////////////////

template<typename T, typename... Ts>
std::shared_ptr<T> newNode(Ts&&... params)
	{
		auto t = std::make_tuple(params...);
		QString id = QString(std::get<0>(t));
		if(hash->contains(id))
			throw InnerModelException("InnerModel::newNode Error: Cannot insert new node with already existing name" + id.toStdString());
		
		std::shared_ptr<T> node(new T(std::forward<Ts>(params)...)); 
		hash->insert(id, std::static_pointer_cast<InnerModelNode>(node));
		return node;
	}


int main()
{



}
