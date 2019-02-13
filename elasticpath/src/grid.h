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
     
		std::list<QVec> computePath(const Key &source_, const Key &target_)
		{
			Key source = pointToGrid(source_.x, source_.z);
			Key target = pointToGrid(target_.x, target_.z);

			// Admission rules
			if( !(target.x >= dim.HMIN and target.x < dim.HMIN+dim.WIDTH and target.z >= dim.VMIN and target.z < dim.VMIN + dim.HEIGHT))
			{
				qDebug() << __FUNCTION__ << "Target out of limits. Returning empty path";
				return std::list<QVec>();
			}
			if( !(source.x >= dim.HMIN and source.x < dim.HMIN+dim.WIDTH and source.z >= dim.VMIN and source.z < dim.VMIN + dim.HEIGHT))
			{
				qDebug() << __FUNCTION__ << "Robot out of limits. Returning empty path";
				return std::list<QVec>();
			}
			if( source == target)
			{
				qDebug() << __FUNCTION__ << "Robot already at target. Returning empty path";
				return std::list<QVec>();
			}

			std::vector<double> min_distance(fmap.size(), DBL_MAX);
			auto id = std::get<T&>(getCell(source)).id;
			min_distance[id] = 0;
			std::vector<std::pair<std::uint32_t, Key>> previous(fmap.size(), std::make_pair(-1, Key()));			
			auto comp = [this](std::pair<std::uint32_t, Key> x, std::pair<std::uint32_t, Key> y)
				{ if(x.first < y.first) 
					return true;
				  else if(x.first == y.first)
				  	return std::get<T&>(getCell(x.second)).id <= std::get<T&>(getCell(y.second)).id;
				  else return false;
				};
			
			std::set< std::pair<double, Key>, decltype(comp)> active_vertices(comp);
			active_vertices.insert({0,source});
			
			while (!active_vertices.empty()) 
			{
				Key where = active_vertices.begin()->second;		
				if (where == target) 
				{
					//qDebug() << __FILE__ << __FUNCTION__  << "Min distance found:" << min_distance[fmap.at(where).id];  //exit point 
					auto p = orderPath(previous, source, target);
					if(p.size() > 1) return p;
					else return std::list<QVec>();
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
						//active_vertices.insert( { min_distance[ed.second.id], ed.first } );    // Djikstra
						active_vertices.insert( { min_distance[ed.second.id] + heuristicL2(ed.first, target), ed.first } );
					}
				}
			}
			qDebug() << __FUNCTION__ << "Path from (" << source.x << "," << source.z << ") not  found. Returning empty path";
			return std::list<QVec>();
		};
		
		auto pointToGrid(long int x, long int z) const -> decltype(Key())
		{
			int kx = (x-dim.HMIN)/dim.TILE_SIZE;
			int kz = (z-dim.VMIN)/dim.TILE_SIZE;
			return Key(dim.HMIN + kx*dim.TILE_SIZE, dim.VMIN + kz*dim.TILE_SIZE);
		};
//TODO: vector must be used with mutex control		
		std::list<Key> occupied;
		void removeOccupiedKey(Key k)
		{
			occupied.push_back(k);
		}
		void addOccupiedKey(Key k)
		{
			occupied.remove(k);
		}
		
	private:
		FMap fmap;
		Dimensions dim;
		
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
			//qDebug() << __FILE__ << __FUNCTION__ << "Path length:" << res.size();  //exit point 
			return res;
		};

		inline double heuristicL2(const Key &a, const Key &b) const
		{
			return sqrt((a.x-b.x)*(a.x-b.x) + (a.z-b.z)*(a.z-b.z));
		}

		std::vector<std::pair<Key, T>> neighboors(const Key &k) 
		{
			std::vector<std::pair<Key, T>> neigh;
			// list of increments to access the neighboors of a given position
			const int &I = dim.TILE_SIZE;
			static const std::vector<int> xincs = {I,I,I,0,-I,-I,-I,0};
			static const std::vector<int> zincs = {I,0,-I,-I,-I,0,I,I};

			for(auto &&[itx, itz] : iter::zip(xincs, zincs))
			{
				Key lk{k.x + itx, k.z + itz}; 
				try
				{
					T &p = fmap.at(Key(lk.x,lk.z));
					auto iter = std::find(occupied.begin(), occupied.end(), Key(lk.x,lk.z));
					if(iter != occupied.end())
					{
						p.cost = 10.f;
						neigh.emplace_back(std::make_pair(lk,p));
					}
					else
					{
						if(p.free)
						{
							if(itx != 0 and itz != 0 and (fabs(itx) == fabs(itz)))
								p.cost = 1.41;		// if neighboor in diagonal, cost is sqrt(2) Should be computed over the initial value
							//if neigh node is close to a occupied node, increase its cost
							for(auto &&[dx, dz] : iter::zip(xincs, zincs))
								try{ if( !(fmap.at(Key(lk.x+dx, lk.z+dz)).free)) p.cost = 10.f; }
								catch(const std::exception& e){	};
							neigh.emplace_back(std::make_pair(lk,p));
						}
					}
				}
				catch(const std::exception& e)
				{
					std::cout << e.what() << " neighbour not found in grid " << lk.x << " " << lk.z << '\n';
				}
			}
			return neigh;
		}
};

#endif // GRID_H
