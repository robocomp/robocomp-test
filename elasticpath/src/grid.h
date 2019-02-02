/*
 * Copyright 2018 <copyright holder> <email>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GRID_H
#define GRID_H

#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <iostream> 
#include <fstream>
#include <cppitertools/zip.hpp>

template<class T> auto operator<<(std::ostream& os, const T& t) -> decltype(t.save(os), os) 
{ 
    t.save(os); 
    return os; 
};
template<class T> auto operator>>(std::istream& is, T& t) -> decltype(t.read(is), is) 
{ 
    t.read(is); 
    return is; 
};

template <typename T>
class Grid
{     
	public:
		
		struct Dimensions
		{
			int TILE_SIZE = 200;
			float HMIN=-2500, VMIN=-2500, WIDTH=2500, HEIGHT=2500;
		};
		
		struct Key
		{
			long int x;
			long int z;
		
			public:
				Key(): x(0), z(0) {};
				Key(long int &&x, long int &&z): x(std::move(x)), z(std::move(z)){};
				Key(long int &x, long int &z): x(x), z(z){};
				Key(const long int &x, const long int &z): x(x), z(z){};
				Key(const QPointF &p){ x=p.x(); z=p.y();};
				bool operator==(const Key &other) const
					{ return x == other.x && z == other.z; };
				void save(std::ostream &os) const { os << x << " " << z << " "; };	//method to save the keys
				void read(std::istream &is)  { is >> x  >> z; };	//method to read the keys
		};

		struct KeyHasher
			{
				std::size_t operator()(const Key& k) const
				{
					using boost::hash_value;
					using boost::hash_combine;

					// Start with a hash value of 0    .
					std::size_t seed = 0;

					// Modify 'seed' by XORing and bit-shifting in one member of 'Key' after the other:
					hash_combine(seed,hash_value(k.x));
					hash_combine(seed,hash_value(k.z));
					return seed;
				};
			};	
			
		using FMap = std::unordered_map<Key, T, KeyHasher>;
		
		//Grid(){};
		
		std::tuple<bool,T&> getCell(long int x, long int z) 											
		{
 			if(!(x >= dim.HMIN and x < dim.HMIN+dim.WIDTH and z >= dim.VMIN and z < dim.VMIN + dim.HEIGHT))
 				return std::forward_as_tuple(false, T());
			else
				return std::forward_as_tuple(true, fmap.at(pointToGrid(x,z)));
		}
		std::tuple<bool,T&> getCell(const Key &k) 			//overladed version								
		{
 			if(!(k.x >= dim.HMIN and k.x < dim.HMIN+dim.WIDTH and k.z >= dim.VMIN and k.z < dim.VMIN + dim.HEIGHT))
 				return std::forward_as_tuple(false, T());
			else
				return std::forward_as_tuple(true, fmap.at(pointToGrid(k.x,k.z)));
		}
		T at(const Key &k) const 									{ return fmap.at(k);};
		T& at(const Key &k) 										{ return fmap.at(k);};
		typename FMap::iterator begin() 							{ return fmap.begin(); };
		typename FMap::iterator end() 								{ return fmap.end();   };
		typename FMap::const_iterator begin() const  				{ return fmap.begin(); };
		typename FMap::const_iterator end() const 	 				{ return fmap.begin(); };
		size_t size() const 										{ return fmap.size();  };
		
		void initialize(const Dimensions &dim_, T &&initValue)
		{
			dim = dim_;
			fmap.clear();
			for( int i = dim.HMIN ; i < dim.HMIN + dim.WIDTH ; i += dim.TILE_SIZE)
				for( int j = dim.VMIN ; j < dim.VMIN + dim.HEIGHT ; j += dim.TILE_SIZE)
					fmap.emplace( Key(i,j), initValue);
	
			// list of increments to access the neighboors of a given position
			I = dim.TILE_SIZE;
			xincs = {I,I,I,0,-I,-I,-I,0};
			zincs = {I,0,-I,-I,-I,0,I,I};	
		
			std::cout << "Grid::Initialize. Grid initialized to map size: " << fmap.size() << std::endl;	
		}
		
		template<typename Q>
		void insert(const Key &key, const Q &value)
		{
				fmap.insert(std::make_pair(key,value));
		}
		
		void clear()
		{
				fmap.clear();
		}
		
		void saveToFile(const std::string &fich)
		{
			std::ofstream myfile;
			myfile.open (fich);
			for(auto &[k, v] : fmap)
			{
				myfile << k << v << std::endl;
			}
			myfile.close();
			std::cout << fmap.size() << " elements written to " << fich << std::endl;
		}
     
		std::list<QVec> djikstra(const Key &source_, const Key &target_)
		{
			qDebug() << __FUNCTION__;
			Key source = source_;
			Key target = target_;

			// Admission rules
			if( !(target.x >= dim.HMIN and target.x < dim.HMIN+dim.WIDTH and target.z >= dim.VMIN and target.z < dim.VMIN + dim.HEIGHT))
			{
				qDebug() << __FUNCTION__ << "Target out of limits. Returning empty path";
				return std::list<QVec>();
			}
			if(fmap.find(source) == fmap.end())
				source = pointToGrid(source_.x, source.z);
			if(fmap.find(target) == fmap.end())
				target = pointToGrid(target_.x, target.z);

			std::vector<std::uint32_t> min_distance(fmap.size(), INT_MAX);
			std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));
			
			//min_distance[std::get<T&>(getCell(source)).id] = 0;
			auto id = std::get<T&>(getCell(source)).id;
			min_distance[id] = 0;
			
			auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y)
				{ return x.first < y.first or (!(y.first < x.first) and std::get<T&>(getCell(x.second)).id < std::get<T&>(getCell(y.second)).id);};
			std::set< std::pair<std::uint32_t, Key>, decltype(comp)> active_vertices(comp);
			
			active_vertices.insert({0,source});
			while (!active_vertices.empty()) 
			{
				Key where = active_vertices.begin()->second;		
				if (where == target) 
				{
					qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point 
					return orderPath(previous, source, target);
				}
				active_vertices.erase( active_vertices.begin() );
				for (auto ed : neighboors(where)) 
				{
					//qDebug() << __FILE__ << __FUNCTION__ << "antes del if" << ed.first.x << ed.first.z << ed.second.id << fmap[where].id << min_distance[ed.second.id] << min_distance[fmap[where].id];
					if (min_distance[ed.second.id] > min_distance[fmap[where].id] + ed.second.cost) 
					{
						active_vertices.erase( { min_distance[ed.second.id], ed.first } );
						min_distance[ed.second.id] = min_distance[fmap[where].id] + ed.second.cost;
						previous[ed.second.id] = std::make_pair(fmap[where].id, where);
						active_vertices.insert( { min_distance[ed.second.id], ed.first } );
					}
				}
			}
			qDebug() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") not  found. Returning empty path";
			return std::list<QVec>();
		};

	std::vector<std::pair<Key, T>> neighboors(const Key &k) const
	{
		qDebug() << __FUNCTION__;
		std::vector<std::pair<Key, T>> neigh;
		// list of increments to access the neighboors of a given position
		const int I = dim.TILE_SIZE;
		const std::vector<int> xincs = {I,I,I,0,-I,-I,-I,0};
		const std::vector<int> zincs = {I,0,-I,-I,-I,0,I,I};

		for(auto &&[itx, itz] : iter::zip(xincs, zincs))
		{
			Key lk{k.x + itx, k.z + itz}; 
			qDebug() << lk.x << lk.z;
	        typename FMap::const_iterator it = fmap.find(lk);
			if( it != fmap.end() and it->second.free )
			{
				T v(it->second);					// bacause iterator is const
				if(itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)))
					v.cost = v.cost * 1.41;		// if neighboor in diagonal, cost is sqrt(2)
				neigh.emplace_back(std::make_pair(lk,v));
			}
		}
		return neigh;
	}

	/**
	 * @brief Recovers the optimal path from the list of previous nodes
	 * 
	 */
	std::list<QVec> orderPath(const std::vector<std::pair<std::uint32_t,Key>> &previous, const Key &source, const Key &target)
	{
		std::list<QVec> res;
		Key k = target;
		std::uint32_t u = fmap.at(k).id;
		while(previous[u].first != (std::uint32_t)-1)
		{
			res.push_front(QVec::vec3(k.x, 0, k.z));
			u = previous[u].first;
			k = previous[u].second;
		}
		qDebug() << __FILE__ << __FUNCTION__ << "Path length:" << res.size();  //exit point 
		return res;
	};

	private:
		FMap fmap;
		Dimensions dim;
		
		// list of increments to access the neighboors of a given position
		int I;
		std::vector<int> xincs;
		std::vector<int> zincs;	
		
		auto pointToGrid(long int x, long int z) const -> decltype(Key())
		{
			int kx = (x-dim.HMIN)/dim.TILE_SIZE;
			int kz = (z-dim.VMIN)/dim.TILE_SIZE;
			return Key(dim.HMIN + kx*dim.TILE_SIZE, dim.VMIN + kz*dim.TILE_SIZE);
		};

		

};

#endif // FLOORMETER_H
